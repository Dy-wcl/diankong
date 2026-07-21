/**
 * @file control_platform.h
 * @brief 固定拓扑控制平台：集中持有 CAN1/CAN2 并完成安全启动。
 */
#ifndef CONTROL_PLATFORM_H
#define CONTROL_PLATFORM_H

#include "comp_cmd.h"
#include "control_common.h"

#include <stdbool.h>

/**
 * @brief 控制平台状态快照。
 */
typedef struct {
  control_state_e state;
  bool initialized;
  bool can1_started;
  bool can2_started;
  err_t last_error;
} control_platform_status_t;

/**
 * @brief 初始化 DWT、两路 CAN、过滤器、三控制 Facade，并启动 CAN。
 *
 * 须在调度器启动前、HAL 外设初始化完成后调用一次。
 *
 * Returns:
 *   成功返回 OK；任一步失败返回对应错误码并保持零输出策略。
 */
err_t control_platform_init(void);

/**
 * @brief 强制底盘与云台进入零输出。
 *
 * Returns:
 *   两路 force_stop 的合成结果；未初始化返回 STATE_ERR。
 */
err_t control_platform_force_stop(void);

/**
 * @brief 读取平台状态。
 */
err_t control_platform_get_status(control_platform_status_t *status);

#endif /* CONTROL_PLATFORM_H */
