/**
 * @file joint_control.c
 * @brief 六台达妙关节电机的只初始化、只订阅实现。
 */
#include "joint_control.h"

#include "dm_motor_ctrl.h"

typedef struct {
  joint_control_status_t status;
  bool init_attempted;
} joint_control_context_t;

static joint_control_context_t joint_context = {
    .status = {
        .state = CONTROL_STATE_UNINITIALIZED,
        .initialized = false,
        .can_attached = false,
        .last_error = PENDING,
    },
    .init_attempted = false,
};

/**
 * @brief 初始化 Motor1-6 默认参数并在 CAN1 上订阅反馈。
 */
err_t joint_control_init(STM32CAN_t *can) {
  if (joint_context.init_attempted) {
    joint_context.status.last_error = STATE_ERR;
    return STATE_ERR;
  }
  if (can == NULL) {
    joint_context.status.last_error = PTR_NULL;
    return PTR_NULL;
  }
  if (can->id_ != BSP_CAN1) {
    joint_context.status.last_error = ARG_ERR;
    return ARG_ERR;
  }

  joint_context.init_attempted = true;
  dm_motor_init();
  joint_context.status.initialized = true;

  const err_t result = dm_motor_attach_can(can);
  joint_context.status.last_error = result;
  if (result != OK) {
    joint_context.status.state = CONTROL_STATE_FAULT;
    return result;
  }

  joint_context.status.can_attached = true;
  joint_context.status.state = CONTROL_STATE_SAFE_DISABLED;
  return OK;
}

/**
 * @brief 读取当前关节被动监听状态。
 */
err_t joint_control_get_status(joint_control_status_t *status) {
  if (status == NULL) {
    return PTR_NULL;
  }
  *status = joint_context.status;
  return OK;
}
