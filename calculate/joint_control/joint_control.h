/**
 * @file joint_control.h
 * @brief 六台达妙关节电机的被动初始化与订阅 Facade。
 */
#ifndef JOINT_CONTROL_H
#define JOINT_CONTROL_H

#include "bsp_can.h"
#include "control_common.h"

#include <stdbool.h>

typedef struct {
  control_state_e state;
  bool initialized;
  bool can_attached;
  err_t last_error;
} joint_control_status_t;

/**
 * @brief 初始化 Motor1-6 默认参数并在 CAN1 上订阅反馈。
 *
 * Args:
 *   can: 已初始化但尚未启动的 CAN1 BSP 对象。
 *
 * Returns:
 *   初始化和订阅成功返回 OK；重复调用或失败返回对应错误码。
 */
err_t joint_control_init(STM32CAN_t *can);

/**
 * @brief 读取当前关节被动监听状态。
 *
 * Args:
 *   status: 状态输出缓冲。
 *
 * Returns:
 *   成功返回 OK，空指针返回 PTR_NULL。
 */
err_t joint_control_get_status(joint_control_status_t *status);

#endif /* JOINT_CONTROL_H */
