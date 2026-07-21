/**
 * @file dj_motor_ctrl.c
 * @brief DJI 电机总线/实例生命周期与写齐再发状态机实现。
 *
 * 本文件维护调用方静态分配的 dj_motor_bus_t 与 dj_motor_t，并通过 bsp_can
 * 的订阅式接收接口处理标准数据帧。发送语义对齐 RMMotor 的 pack-and-send：
 *
 * - set_command 限幅并写 group.tx_buff[slot]，置 pending 对应位；
 * - 当 pending_mask == group_mask 时，在临界区外打包发送并清 pending；
 * - 发送失败不清 pending，调用方可重写或 force_flush；
 * - zero_and_flush 将已注册槽命令置 0 后立即发送，供安全停机。
 *
 * @note RX 回调运行在 CAN 中断上下文：只解码原始整数并更新 tick，禁止浮点、
 *       日志、动态分配与二次发送。STM32CAN_Send 始终在临界区外调用。
 */
#include "dj_motor_ctrl.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>


 //! 在初始化总线时调用，或在注销所有电机后调用
 //清空单条 CAN 总线上的所有电机组状态(不影响控制组 ID)(用作清零并建表)
static void dj_motor_groups_reset(dj_motor_bus_t *bus) {
  static const dj_motor_group_e ids[DJ_MOTOR_GROUP_COUNT] = {
      DJ_MOTOR_GROUP_200, DJ_MOTOR_GROUP_1FF, DJ_MOTOR_GROUP_2FF};

  for (uint8_t i = 0U; i < DJ_MOTOR_GROUP_COUNT; ++i) {
    dj_motor_group_state_t *group = &bus->groups[i];
    memset(group, 0, sizeof(*group));
    group->id = ids[i];
  }
}


 //根据控制组 ID 取得 bus 内对应的 group_state 指针（0x200、0x1FF、0x2FF 各一组）
static dj_motor_group_state_t *dj_motor_group_state(dj_motor_bus_t *bus,
                                                    dj_motor_group_e group) {
  const uint8_t index = dj_motor_drv_group_index(group);
  if ((bus == NULL) || (index >= DJ_MOTOR_GROUP_COUNT)) {
    return NULL;
  }
  return &bus->groups[index];
}

//在临界区外根据 是否在 pending_mask 中置位来发送命令快照，成功时可选清零 pending
static err_t dj_motor_send_snapshot(dj_motor_bus_t *bus,
                                    dj_motor_group_state_t *group,
                                    const int16_t commands[4],
                                    bool clear_pending_on_ok) {
  const err_t result =
      dj_motor_drv_send_group(bus->can, group->id, commands);
  if ((result == OK) && clear_pending_on_ok) {
    taskENTER_CRITICAL();
    group->pending_mask = 0U;
    taskEXIT_CRITICAL();
  }
  return result;
}

/**
 * @brief 总线级 CAN RX 回调：按 feedback_id 匹配电机并更新原始反馈。
 * @param can 触发回调的 CAN 对象。
 * @param frame 完整帧快照，仅本次回调有效。
 * @param context 注册订阅时传入的 bus 指针。
 *
 * 过滤条件：context 有效、can 指针一致、标准数据帧、DLC=8。
 * 命中后解码到局部 raw，再整体写回 motor，保证并发读不会看到半帧。
 * 同一帧最多匹配一个电机；匹配成功后立即返回。
 */
static void dj_motor_rx_callback(STM32CAN_t *can, const BSP_CAN_Frame_t *frame,
                                 void *context) {
  dj_motor_bus_t *bus = (dj_motor_bus_t *)context;
  if ((bus == NULL) || (can == NULL) || (frame == NULL) || (bus->can != can) ||
      (frame->ide_ != CAN_ID_STD) || (frame->rtr_ != CAN_RTR_DATA) ||
      (frame->size_ != BSP_CAN_DATA_SIZE)) {
    return;
  }

  for (uint8_t index = 0U; index < bus->motor_count; ++index) {
    dj_motor_t *motor = bus->motors[index];
    if ((motor == NULL) || !motor->initialized ||
        (motor->route.feedback_id != frame->id_)) {
      continue;
    }

    dj_motor_raw_feedback_t raw;
    if (dj_motor_drv_decode_feedback(frame->data_, &raw) != OK) {
      return;
    }

    motor->raw_feedback = raw;
    motor->last_feedback_tick = HAL_GetTick();
    motor->feedback_valid = true;
    return;
  }
}

/**
 * @brief 绑定 CAN、复位控制组并注册 RX 订阅。
 *
 * 订阅失败时清零整个 bus，避免 initialized 与订阅状态不一致。
 */
err_t dj_motor_bus_init(dj_motor_bus_t *bus, STM32CAN_t *can) {
  if ((bus == NULL) || (can == NULL)) {
    return PTR_NULL;
  }

  memset(bus, 0, sizeof(*bus));
  bus->can = can;
  dj_motor_groups_reset(bus);

  const err_t result =
      STM32CAN_SubscribeRx(can, dj_motor_rx_callback, bus);
  if (result != OK) {
    memset(bus, 0, sizeof(*bus));
    return result;
  }

  bus->initialized = true;
  return OK;
}

/**
 * @brief 注册单个电机：路由推导 → 冲突检测 → 挂入 bus 与 group。
 *
 * 冲突检测包括：同 bus 重复 feedback_id、重复 (group,slot)、槽位已被占用。
 * CAN 已 Start 后拒绝继续注册，保证运行期注册表只读、RX 可安全遍历指针。
 */
err_t dj_motor_init(dj_motor_t *motor, dj_motor_bus_t *bus,
                    dj_motor_type_e type, uint8_t device_id, bool reversed) {
  if ((motor == NULL) || (bus == NULL)) {
    return PTR_NULL;
  }
  if (!bus->initialized || (bus->can == NULL)) {
    return STATE_ERR;
  }
  if (bus->can->started_) {
    return STATE_ERR;
  }
  if (bus->motor_count >= DJ_MOTOR_BUS_CAPACITY) {
    return FULL;
  }
  if (motor->initialized) {
    return STATE_ERR;
  }

  //根据设备 ID 推导路由
  dj_motor_route_t route;
  const err_t route_result =
      dj_motor_drv_derive_route(type, device_id, &route);
  if (route_result != OK) {
    return route_result;
  }

  dj_motor_group_state_t *group = dj_motor_group_state(bus, route.group);
  if ((group == NULL) || (route.slot >= DJ_MOTOR_GROUP_SLOT_COUNT)) {
    return ARG_ERR;
  }

  /* 同 bus 地址冲突：反馈 ID 或 (control_group, slot) 任一重复即拒绝 */
  for (uint8_t index = 0U; index < bus->motor_count; ++index) {
    const dj_motor_t *existing = bus->motors[index];
    if (existing == NULL) {
      continue;
    }
    if (existing->route.feedback_id == route.feedback_id) {
      return ARG_ERR;
    }
    if ((existing->route.group == route.group) &&
        (existing->route.slot == route.slot)) {
      return ARG_ERR;
    }
  }

  if (group->members[route.slot] != NULL) {
    return ARG_ERR;
  }

  memset(motor, 0, sizeof(*motor));
  motor->bus = bus;
  motor->type = type;
  motor->device_id = device_id;
  motor->reversed = reversed;
  motor->route = route;
  motor->initialized = true;

  bus->motors[bus->motor_count] = motor;
  ++bus->motor_count;
  group->members[route.slot] = motor;
  group->group_mask =
      (uint8_t)(group->group_mask | (uint8_t)(1U << route.slot));
  group->tx_buff[route.slot] = 0;
  return OK;
}

/**
 * @brief 写命令并在组内写齐时自动发送。
 *
 * 步骤：
 * 1. 校验实例与所属组；
 * 2. 逻辑限幅，按 reversed 得到物理命令；
 * 3. 临界区内写 command、tx_buff、pending 位；
 * 4. 若 pending == group_mask，拷贝四槽快照到栈上；
 * 5. 临界区外发送；成功则清 pending。
 */
err_t dj_motor_set_command(dj_motor_t *motor, int16_t command) {
  if (motor == NULL) {
    return PTR_NULL;
  }
  if (!motor->initialized || (motor->bus == NULL) ||
      !motor->bus->initialized) {
    return STATE_ERR;
  }

  dj_motor_group_state_t *group =
      dj_motor_group_state(motor->bus, motor->route.group);
  if ((group == NULL) || (group->group_mask == 0U) ||
      (group->members[motor->route.slot] != motor)) {
    return NOT_FOUND;
  }

  int16_t logical = dj_motor_drv_clamp_command(motor->type, command);
  int16_t physical = logical;
  if (motor->reversed) {
    physical = (int16_t)(-physical);
  }

  bool should_send = false;
  int16_t snapshot[DJ_MOTOR_GROUP_SLOT_COUNT] = {0};

  taskENTER_CRITICAL();
  motor->command = logical;
  group->tx_buff[motor->route.slot] = physical;
  group->pending_mask =
      (uint8_t)(group->pending_mask | (uint8_t)(1U << motor->route.slot));
  if ((group->group_mask != 0U) &&
      (group->pending_mask == group->group_mask)) {
    memcpy(snapshot, group->tx_buff, sizeof(snapshot));
    should_send = true;
  }
  taskEXIT_CRITICAL();

  if (!should_send) {
    return OK;
  }

  return dj_motor_send_snapshot(motor->bus, group, snapshot, true);
}

/**
 * @brief 忽略写齐条件，立即发送当前缓存。
 *
 * 用于调试或需要“不等待其他成员”的冲刷场景；安全停机更推荐 zero_and_flush。
 */
err_t dj_motor_force_flush_group(dj_motor_bus_t *bus,
                                 dj_motor_group_e group_id) {
  if (bus == NULL) {
    return PTR_NULL;
  }
  if (!bus->initialized) {
    return STATE_ERR;
  }

  dj_motor_group_state_t *group = dj_motor_group_state(bus, group_id);
  if (group == NULL) {
    return ARG_ERR;
  }
  if (group->group_mask == 0U) {
    return NOT_FOUND;
  }

  int16_t snapshot[DJ_MOTOR_GROUP_SLOT_COUNT] = {0};
  taskENTER_CRITICAL();
  memcpy(snapshot, group->tx_buff, sizeof(snapshot));
  taskEXIT_CRITICAL();

  return dj_motor_send_snapshot(bus, group, snapshot, true);
}

/**
 * @brief 安全停机：命令与缓存清零后强制发送全零帧。
 *
 * 无论 pending 是否写齐都会发送。成功与否均已在临界区内清零命令与 pending，
 * 避免半写状态残留导致后续意外拼包。
 */
err_t dj_motor_zero_and_flush(dj_motor_bus_t *bus, dj_motor_group_e group_id) {
  if (bus == NULL) {
    return PTR_NULL;
  }
  if (!bus->initialized) {
    return STATE_ERR;
  }

  dj_motor_group_state_t *group = dj_motor_group_state(bus, group_id);
  if (group == NULL) {
    return ARG_ERR;
  }
  if (group->group_mask == 0U) {
    return NOT_FOUND;
  }

  int16_t snapshot[DJ_MOTOR_GROUP_SLOT_COUNT] = {0};

  taskENTER_CRITICAL();
  for (uint8_t slot = 0U; slot < DJ_MOTOR_GROUP_SLOT_COUNT; ++slot) {
    group->tx_buff[slot] = 0;
    if (group->members[slot] != NULL) {
      group->members[slot]->command = 0;
    }
  }
  group->pending_mask = 0U;
  memcpy(snapshot, group->tx_buff, sizeof(snapshot));
  taskEXIT_CRITICAL();

  return dj_motor_send_snapshot(bus, group, snapshot, true);
}

/**
 * @brief 拷贝原始反馈并在任务上下文完成方向与单位换算。
 *
 * reversed 时对 rpm、current、position 取反；encoder_raw 保持硬件值。
 * speed_rad_s 使用逻辑方向下的 rpm 与型号减速比计算输出轴角速度。
 */
err_t dj_motor_get_feedback(const dj_motor_t *motor,
                            dj_motor_feedback_t *feedback) {
  if ((motor == NULL) || (feedback == NULL)) {
    return PTR_NULL;
  }
  if (!motor->initialized) {
    return STATE_ERR;
  }

  dj_motor_raw_feedback_t raw;
  uint32_t tick;
  bool valid;

  taskENTER_CRITICAL();
  raw = motor->raw_feedback;
  tick = motor->last_feedback_tick;
  valid = motor->feedback_valid;
  taskEXIT_CRITICAL();

  int16_t speed = raw.speed_rpm;
  int16_t current = raw.current;
  float position = dj_motor_drv_encoder_to_rad(raw.encoder);
  if (motor->reversed) {
    speed = (int16_t)(-speed);
    current = (int16_t)(-current);
    position = -position;
  }

  feedback->encoder_raw = raw.encoder;
  feedback->speed_rpm = speed;
  feedback->current = current;
  feedback->temperature = raw.temperature;
  feedback->position_rad = position;
  feedback->speed_rad_s = dj_motor_drv_rpm_to_rad_s(motor->type, speed);
  feedback->last_feedback_tick = tick;
  feedback->valid = valid;
  return OK;
}

/**
 * @brief 在线判定：initialized ∧ feedback_valid ∧ 未超时。
 *
 * (now_tick - last_tick) 使用无符号减法，兼容 HAL tick 回绕。
 */
bool dj_motor_is_online(const dj_motor_t *motor, uint32_t now_tick,
                        uint32_t timeout_ticks) {
  if ((motor == NULL) || !motor->initialized) {
    return false;
  }

  taskENTER_CRITICAL();
  const bool valid = motor->feedback_valid;
  const uint32_t last_tick = motor->last_feedback_tick;
  taskEXIT_CRITICAL();

  return valid && ((uint32_t)(now_tick - last_tick) <= timeout_ticks);
}
