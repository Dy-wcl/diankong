/**
 * @file gimbal_control.h
 * @brief 固定双轴 GM6020 云台控制 Facade。
 */
#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include "bsp_can.h"
#include "control_common.h"

#include <stdbool.h>
#include <stdint.h>

#ifndef GIMBAL_ACTUATION_ENABLED
#define GIMBAL_ACTUATION_ENABLED (0)
#endif

#define GIMBAL_FEEDBACK_TIMEOUT_MS (20U)
#define GIMBAL_ZERO_HEARTBEAT_MS (20U)

typedef struct {
  float yaw_speed_rpm;
  float pitch_speed_rpm;
  bool enable;
  bool source_online;
} gimbal_control_input_t;

typedef struct {
  control_state_e state;
  bool initialized;
  bool source_online;
  bool yaw_online;
  bool pitch_online;
  bool zero_retry_pending;
  float yaw_target_rpm;
  float pitch_target_rpm;
  int16_t yaw_feedback_rpm;
  int16_t pitch_feedback_rpm;
  int16_t yaw_command;
  int16_t pitch_command;
  err_t last_error;
  uint32_t last_step_tick;
  uint32_t last_zero_tick;
} gimbal_control_status_t;

/**
 * @brief 在指定 CAN1 对象上注册 yaw/pitch 两台 GM6020。
 *
 * Args:
 *   can: 已初始化但尚未启动的 CAN1 BSP 对象。
 *
 * Returns:
 *   初始化成功返回 OK，否则返回参数、状态或 DJ 电机错误码。
 */
err_t gimbal_control_init(STM32CAN_t *can);

/**
 * @brief 执行一次双轴反馈检查、速度闭环和安全输出。
 *
 * Args:
 *   input: 本周期通用云台输入。
 *   now_tick: 当前毫秒 tick。
 *
 * Returns:
 *   本周期成功返回 OK；非法输入、反馈超时或发送失败返回对应错误码。
 */
err_t gimbal_control_step(const gimbal_control_input_t *input,
                          uint32_t now_tick);

/**
 * @brief 立即复位双轴 PID 并尝试发送 0x1FF 全零帧。
 *
 * Returns:
 *   零帧发送结果；未初始化返回 STATE_ERR。
 */
err_t gimbal_control_force_stop(void);

/**
 * @brief 读取当前云台控制状态快照。
 *
 * Args:
 *   status: 状态输出缓冲。
 *
 * Returns:
 *   成功返回 OK，空指针返回 PTR_NULL。
 */
err_t gimbal_control_get_status(gimbal_control_status_t *status);

#endif /* GIMBAL_CONTROL_H */
