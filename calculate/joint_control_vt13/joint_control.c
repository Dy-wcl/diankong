/**
 * @file joint_control.c
 * @brief 六台达妙关节电机控制实现
 */
#include "joint_control.h"

#include "bsp_can.h"
#include "can.h"
#include "cmsis_os2.h"
#include "comp_utils.h"
#include "dm_motor_drv.h"
#include "vt13.h"

extern motor_t motor[num];
extern vt13_cmd_rc_t vt13_cmd_rc;
extern uint16_t TB6210_angle;

/* CAN1 实例用于关节电机通信 */
static STM32CAN_t* can1_instance = NULL;

uint8_t joint_enable_single = 0U;
uint8_t joint_mode = 0U;
uint8_t joint_send_phase = 0U;
Joint_mode_t joint_mode_t;

float pos[num] = {1.2f, 0.0f, 0.0f, -2.7f, -1.15f, 0.5f};

static const float joint_angle_limit_min[num] = {
    JOINT_MOTOR1_ANGLE_LIMIT_MIN, JOINT_MOTOR2_ANGLE_LIMIT_MIN,
    JOINT_MOTOR3_ANGLE_LIMIT_MIN, JOINT_MOTOR4_ANGLE_LIMIT_MIN,
    JOINT_MOTOR5_ANGLE_LIMIT_MIN, JOINT_MOTOR6_ANGLE_LIMIT_MIN,
};

static const float joint_angle_limit_max[num] = {
    JOINT_MOTOR1_ANGLE_LIMIT_MAX, JOINT_MOTOR2_ANGLE_LIMIT_MAX,
    JOINT_MOTOR3_ANGLE_LIMIT_MAX, JOINT_MOTOR4_ANGLE_LIMIT_MAX,
    JOINT_MOTOR5_ANGLE_LIMIT_MAX, JOINT_MOTOR6_ANGLE_LIMIT_MAX,
};

static void joint_constrain_target(motor_num motor_id, float* target_angle) {
  ASSERT(motor_id < num);
  ASSERT(target_angle != NULL);
  JOINT_CONSTRAIN_TARGET_BY_ID(target_angle, motor_id);
}

void one_return(void) {
  if (vt13_cmd_rc.B == 1) pos[0] = 1.2f;
}

static void joint_init_can(void) {
  if (can1_instance == NULL) {
    BSP_CAN_t can_id = BSP_CAN_get_id(CAN1);
    if (can_id != BSP_CAN_ID_ERROR) {
      can1_instance = STM32CAN_GetInstance(can_id);
    }
  }
}

void joint_enable(void) {
  joint_init_can();
  if (can1_instance == NULL) return;

  dm_motor_enable(can1_instance, &motor[0]);
  dm_motor_enable(can1_instance, &motor[1]);
  osDelay(1);
  dm_motor_enable(can1_instance, &motor[2]);
  dm_motor_enable(can1_instance, &motor[3]);
  osDelay(1);
  dm_motor_enable(can1_instance, &motor[4]);
  dm_motor_enable(can1_instance, &motor[5]);
  osDelay(1);
  joint_send_phase = 0U;
}

static void joint_disable(void) {
  joint_enable_single = 1U;
  joint_send_phase = 0U;
  if (can1_instance == NULL) return;

  for (uint8_t i = 0; i < num; i++) {
    dm_motor_disable(can1_instance, &motor[i]);
    osDelay(1);
  }
}

void joint_mouse_ctrl(void) {
  if (joint_mode_t.A == 3U) {
    TB6210_angle += 20;
    CONSTRAIN_PTR(&TB6210_angle, 0, 180);
  }
  if (joint_mode_t.D == 3U) {
    TB6210_angle -= 20;
    CONSTRAIN_PTR(&TB6210_angle, 0, 180);
  }
}

void joint_up_ctrl(void) {
  pos[3] += vt13_cmd_rc.ch.l.x * 0.0005;
  pos[4] -= vt13_cmd_rc.ch.l.y * 0.0005;
  pos[5] -= vt13_cmd_rc.ch.r.x * 0.0005;
}

void joint_send_pos(void) {
  if (can1_instance == NULL) return;

  switch (joint_send_phase) {
    case 0:
      pos_ctrl(can1_instance, motor[Motor1].id, pos[0], 0.5f);
      pos_ctrl(can1_instance, motor[Motor2].id, pos[1], 0.5f);
      break;
    case 1:
      pos_ctrl(can1_instance, motor[Motor3].id, pos[2], 0.5f);
      pos_ctrl(can1_instance, motor[Motor4].id, pos[3], 0.5f);
      break;
    default:
      pos_ctrl(can1_instance, motor[Motor5].id, pos[4], 0.5f);
      pos_ctrl(can1_instance, motor[Motor6].id, pos[5], 0.5f);
      break;
  }
  joint_send_phase = (uint8_t)((joint_send_phase + 1U) % 3U);
}

void joint_down_ctrl(void) {
  pos[0] -= vt13_cmd_rc.ch.l.x * 0.0005;
  pos[1] -= vt13_cmd_rc.ch.l.y * 0.0005;
  pos[2] += vt13_cmd_rc.ch.r.y * 0.0005;
  joint_constrain_target(0, &pos[0]);
  joint_constrain_target(1, &pos[1]);
  joint_constrain_target(2, &pos[2]);
}

void joint_mode_change(void) {
  if (vt13_cmd_rc.func.fn_1 == 1) {
    joint_mode = 1U;
  }
  if (vt13_cmd_rc.func.fn_2 == 1) {
    joint_mode = 2U;
  }
}

void Joint_Mode(void) {
  one_return();
  if (vt13_cmd_rc.mode_sw == vt13_CMD_SW_UP) {
    joint_mode = 0U;
    joint_disable();
  }

  if (vt13_cmd_rc.mode_sw == vt13_CMD_SW_MID) {
    joint_enable();
    joint_mode = 0U;
  }

  joint_mouse_ctrl();
  if (vt13_cmd_rc.mode_sw == vt13_CMD_SW_DOWN) {
    joint_mode_change();
    if (joint_mode == 1U) {
      joint_down_ctrl();
    }
    if (joint_mode == 2U) {
      joint_up_ctrl();
    }
    joint_send_pos();
  }
}
