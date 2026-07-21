/**
 * @file control_platform.c
 * @brief 集中 CAN 协调与控制 Facade 安全启动。
 */
#include "control_platform.h"

#include "bsp_can.h"
#include "bsp_dwt.h"
#include "can.h"
#include "chassis_control.h"
#include "gimbal_control.h"
#include "joint_control.h"

#include <stddef.h>
#include <string.h>

typedef struct {
  STM32CAN_t can1;
  STM32CAN_t can2;
  control_platform_status_t status;
  bool init_attempted;
} control_platform_context_t;

static control_platform_context_t platform = {
    .status =
        {
            .state = CONTROL_STATE_UNINITIALIZED,
            .last_error = PENDING,
        },
};

/**
 * @brief 配置单路 CAN 的全通掩码过滤器（开发期安全收发验证）。
 */
static err_t platform_config_accept_all(STM32CAN_t *can, uint32_t filter_bank) {
  CAN_FilterTypeDef filter = {0};
  filter.FilterIdHigh = 0x0000U;
  filter.FilterIdLow = 0x0000U;
  filter.FilterMaskIdHigh = 0x0000U;
  filter.FilterMaskIdLow = 0x0000U;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterBank = filter_bank;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;
  return STM32CAN_ConfigFilter(can, &filter);
}

/**
 * @brief 初始化 DWT、两路 CAN、过滤器、三控制 Facade，并启动 CAN。
 */
err_t control_platform_init(void) {
  if (platform.init_attempted) {
    return STATE_ERR;
  }
  platform.init_attempted = true;
  memset(&platform.status, 0, sizeof(platform.status));
  platform.status.state = CONTROL_STATE_UNINITIALIZED;
  platform.status.last_error = PENDING;

  DWT_Init(168U);

  err_t result = STM32CAN_Init(&platform.can1, &hcan1);
  if (result != OK) {
    platform.status.state = CONTROL_STATE_FAULT;
    platform.status.last_error = result;
    return result;
  }

  result = STM32CAN_Init(&platform.can2, &hcan2);
  if (result != OK) {
    platform.status.state = CONTROL_STATE_FAULT;
    platform.status.last_error = result;
    return result;
  }

  result = platform_config_accept_all(&platform.can1, 0U);
  if (result != OK) {
    platform.status.state = CONTROL_STATE_FAULT;
    platform.status.last_error = result;
    return result;
  }
  result = platform_config_accept_all(&platform.can2, 14U);
  if (result != OK) {
    platform.status.state = CONTROL_STATE_FAULT;
    platform.status.last_error = result;
    return result;
  }

  /* 订阅必须在 Start 之前完成。 */
  result = chassis_control_init(&platform.can2);
  if (result != OK) {
    platform.status.state = CONTROL_STATE_FAULT;
    platform.status.last_error = result;
    return result;
  }
  result = gimbal_control_init(&platform.can1);
  if (result != OK) {
    platform.status.state = CONTROL_STATE_FAULT;
    platform.status.last_error = result;
    return result;
  }
  result = joint_control_init(&platform.can1);
  if (result != OK) {
    platform.status.state = CONTROL_STATE_FAULT;
    platform.status.last_error = result;
    return result;
  }

  result = STM32CAN_Start(&platform.can1);
  if (result != OK) {
    platform.status.state = CONTROL_STATE_FAULT;
    platform.status.last_error = result;
    return result;
  }
  platform.status.can1_started = true;

  result = STM32CAN_Start(&platform.can2);
  if (result != OK) {
    platform.status.state = CONTROL_STATE_FAULT;
    platform.status.last_error = result;
    return result;
  }
  platform.status.can2_started = true;

  /* 上电后立即尝试发送安全零帧。 */
  (void)chassis_control_force_stop();
  (void)gimbal_control_force_stop();

  platform.status.initialized = true;
  platform.status.state = CONTROL_STATE_SAFE_DISABLED;
  platform.status.last_error = OK;
  return OK;
}

/**
 * @brief 强制底盘与云台进入零输出。
 */
err_t control_platform_force_stop(void) {
  if (!platform.status.initialized) {
    return STATE_ERR;
  }

  const err_t chassis_result = chassis_control_force_stop();
  const err_t gimbal_result = gimbal_control_force_stop();
  if (chassis_result != OK) {
    platform.status.state = CONTROL_STATE_FAULT;
    platform.status.last_error = chassis_result;
    return chassis_result;
  }
  if (gimbal_result != OK) {
    platform.status.state = CONTROL_STATE_FAULT;
    platform.status.last_error = gimbal_result;
    return gimbal_result;
  }

  platform.status.state = CONTROL_STATE_SAFE_DISABLED;
  platform.status.last_error = OK;
  return OK;
}

/**
 * @brief 读取平台状态。
 */
err_t control_platform_get_status(control_platform_status_t *status) {
  if (status == NULL) {
    return PTR_NULL;
  }
  *status = platform.status;
  return OK;
}
