/**
 * @file dj_motor_drv.c
 * @brief DJI 电机 CAN 协议编解码、路由推导与发送适配实现。
 *
 * 本文件全部为无状态函数：不持有全局注册表，不管理 pending_mask。
 * 所有 CAN 发送通过 bsp_can 的 STM32CAN_Send() 完成，使用 11 bit 标准数据帧、
 * DLC=8。控制帧 ID 直接使用 dj_motor_group_e 的数值。
 *
 * @note 发送失败（BUSY / FAILED 等）原样返回给 ctrl 层；本层不缓存待发帧。
 */
#include "dj_motor_drv.h"

#include <string.h>

/** 圆周率常量，用于编码器与 RPM 的弧度换算。 */
#define DJ_MOTOR_PI (3.14159265358979323846f)

/**
 * @brief 按大端序读取 16 位无符号整数。
 * @param data 至少 2 字节的缓冲，data[0] 为高字节。
 * @return 组合后的 u16 值。
 *
 * DJI 反馈帧中 encoder / rpm / current 均使用大端字节序。
 */
static uint16_t dj_motor_drv_read_u16_be(const uint8_t data[2]) {
  return (uint16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
}

/**
 * @brief 按型号返回转子到输出轴的减速比。
 * @param type 电机型号。
 * @return 减速比；未知型号回退为 1.0f。
 */
static float dj_motor_drv_reduction_ratio(dj_motor_type_e type) {
  switch (type) {
  case DJ_MOTOR_M3508:
    return DJ_MOTOR_M3508_REDUCTION_RATIO;
  case DJ_MOTOR_M2006:
    return DJ_MOTOR_M2006_REDUCTION_RATIO;
  case DJ_MOTOR_GM6020:
    return DJ_MOTOR_GM6020_REDUCTION_RATIO;
  default:
    return 1.0f;
  }
}

/**
 * @brief 判断控制组是否为 0x200 / 0x1FF / 0x2FF 之一。
 */
bool dj_motor_drv_group_is_valid(dj_motor_group_e group) {
  return (group == DJ_MOTOR_GROUP_200) || (group == DJ_MOTOR_GROUP_1FF) ||
         (group == DJ_MOTOR_GROUP_2FF);
}

/**
 * @brief 控制组 → groups 数组下标。
 * @return 0..2 或 0xFF（非法）。
 */
uint8_t dj_motor_drv_group_index(dj_motor_group_e group) {
  switch (group) {
  case DJ_MOTOR_GROUP_200:
    return 0U;
  case DJ_MOTOR_GROUP_1FF:
    return 1U;
  case DJ_MOTOR_GROUP_2FF:
    return 2U;
  default:
    return 0xFFU;
  }
}

//根据设备 ID 推导路由，返回反馈 ID、控制组与槽位
err_t dj_motor_drv_derive_route(dj_motor_type_e type, uint8_t device_id,
                                dj_motor_route_t *route) {
  if (route == NULL) {
    return PTR_NULL;
  }

  switch (type) {
  case DJ_MOTOR_M3508:
  case DJ_MOTOR_M2006:
    if ((device_id < 1U) || (device_id > 8U)) {
      return ARG_ERR;
    }
    route->feedback_id = (uint16_t)(0x200U + device_id);
    if (device_id <= 4U) {
      route->group = DJ_MOTOR_GROUP_200;
      route->slot = (uint8_t)(device_id - 1U);
    } else {
      route->group = DJ_MOTOR_GROUP_1FF;
      route->slot = (uint8_t)(device_id - 5U);
    }
    return OK;

  case DJ_MOTOR_GM6020:
    /* GM6020 无设备 ID 8；控制组为 0x1FF / 0x2FF */
    if ((device_id < 1U) || (device_id > 7U)) {
      return ARG_ERR;
    }
    route->feedback_id = (uint16_t)(0x204U + device_id);
    if (device_id <= 4U) {
      route->group = DJ_MOTOR_GROUP_1FF;
      route->slot = (uint8_t)(device_id - 1U);
    } else {
      route->group = DJ_MOTOR_GROUP_2FF;
      route->slot = (uint8_t)(device_id - 5U);
    }
    return OK;

  default:
    return ARG_ERR;
  }
}

/**
 * @brief 返回型号对应的对称限幅绝对值。
 */
int16_t dj_motor_drv_command_limit(dj_motor_type_e type) {
  switch (type) {
  case DJ_MOTOR_M3508:
    return (int16_t)DJ_MOTOR_M3508_LIMIT;
  case DJ_MOTOR_M2006:
    return (int16_t)DJ_MOTOR_M2006_LIMIT;
  case DJ_MOTOR_GM6020:
    return (int16_t)DJ_MOTOR_GM6020_LIMIT;
  default:
    return 0;
  }
}

/**
 * @brief 将 command 钳位到 [-limit, +limit]。
 */
int16_t dj_motor_drv_clamp_command(dj_motor_type_e type, int16_t command) {
  const int16_t limit = dj_motor_drv_command_limit(type);
  if (limit <= 0) {
    return 0;
  }
  if (command > limit) {
    return limit;
  }
  if (command < (int16_t)(-limit)) {
    return (int16_t)(-limit);
  }
  return command;
}

/**
 * @brief 四槽大端打包：slot0→[0:1] … slot3→[6:7]。
 */
void dj_motor_drv_pack_commands(const int16_t commands[4],
                                uint8_t payload[8]) {
  if ((commands == NULL) || (payload == NULL)) {
    return;
  }

  for (uint8_t index = 0U; index < 4U; ++index) {
    const uint16_t raw = (uint16_t)commands[index];
    payload[index * 2U] = (uint8_t)(raw >> 8U);
    payload[index * 2U + 1U] = (uint8_t)raw;
  }
}

/**
 * @brief 解码反馈帧原始整数（不做方向与物理量处理）。
 */
err_t dj_motor_drv_decode_feedback(const uint8_t data[8],
                                   dj_motor_raw_feedback_t *feedback) {
  if ((data == NULL) || (feedback == NULL)) {
    return PTR_NULL;
  }

  /* 帧布局：enc[0:1] rpm[2:3] cur[4:5] temp[6]，[7] 保留 */
  feedback->encoder = dj_motor_drv_read_u16_be(&data[0]);
  feedback->speed_rpm = (int16_t)dj_motor_drv_read_u16_be(&data[2]);
  feedback->current = (int16_t)dj_motor_drv_read_u16_be(&data[4]);
  feedback->temperature = data[6];
  return OK;
}

/**
 * @brief encoder / 8192 * 2π → [0, 2π)。
 */
float dj_motor_drv_encoder_to_rad(uint16_t encoder) {
  return ((float)encoder / (float)DJ_MOTOR_ENCODER_RESOLUTION) * 2.0f *
         DJ_MOTOR_PI;
}

/**
 * @brief (rpm / 60 * 2π) / reduction → 输出轴 rad/s。
 */
float dj_motor_drv_rpm_to_rad_s(dj_motor_type_e type, int16_t rpm) {
  const float reduction = dj_motor_drv_reduction_ratio(type);
  if (reduction == 0.0f) {
    return 0.0f;
  }
  return (((float)rpm / 60.0f) * 2.0f * DJ_MOTOR_PI) / reduction;
}

/**
 * @brief 组帧后调用 STM32CAN_Send；邮箱忙等错误原样透传。
 */
err_t dj_motor_drv_send_group(STM32CAN_t *can, dj_motor_group_e group,
                              const int16_t commands[4]) {
  if ((can == NULL) || (commands == NULL)) {
    return PTR_NULL;
  }
  if (!dj_motor_drv_group_is_valid(group)) {
    return ARG_ERR;
  }

  uint8_t payload[BSP_CAN_DATA_SIZE] = {0};
  dj_motor_drv_pack_commands(commands, payload);
  return STM32CAN_Send(can, (uint32_t)group, payload, sizeof(payload));
}
