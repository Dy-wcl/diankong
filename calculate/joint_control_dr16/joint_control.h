/**
 * @file joint_control.h
 * @brief 六台达妙关节电机控制接口
 */
#ifndef _JOINT_CONTROL_H_
#define _JOINT_CONTROL_H_

#include "convert.h"
#include "process.h"

#define JOINT_MOTOR1_ANGLE_LIMIT_MIN (-0.3f)
#define JOINT_MOTOR1_ANGLE_LIMIT_MAX (2.4f)
#define JOINT_MOTOR2_ANGLE_LIMIT_MIN (-3.14f)
#define JOINT_MOTOR2_ANGLE_LIMIT_MAX (0.0f)
#define JOINT_MOTOR3_ANGLE_LIMIT_MIN (0.06f)
#define JOINT_MOTOR3_ANGLE_LIMIT_MAX (1.2f)
#define JOINT_MOTOR4_ANGLE_LIMIT_MIN (1.3f)
#define JOINT_MOTOR4_ANGLE_LIMIT_MAX (3.00f)
#define JOINT_MOTOR5_ANGLE_LIMIT_MIN (-2.5f)
#define JOINT_MOTOR5_ANGLE_LIMIT_MAX (-0.5f)
#define JOINT_MOTOR6_ANGLE_LIMIT_MIN (-0.5f)
#define JOINT_MOTOR6_ANGLE_LIMIT_MAX (0.66f)

#define JOINT_FIXED_TARGET_SPEED 1.0f
#define JOINT_CONSTRAIN_TARGET_BY_ID(target_ptr, motor_id) \
  CONSTRAIN_PTR((target_ptr), joint_angle_limit_min[(motor_id)], \
    joint_angle_limit_max[(motor_id)])

typedef struct {
  uint16_t W;
  uint16_t S;
  uint16_t A;
  uint16_t D;
  uint16_t shift;
  uint16_t ctrl;
  uint16_t Q;
  uint16_t E;
  uint16_t R;
  uint16_t F;
  uint16_t G;
  uint16_t Z;
  uint16_t X;
  uint16_t C;
  uint16_t V;
  uint16_t B;
} Joint_mode_t;

void Joint_Mode(void);
void joint_down_ctrl(void);
void joint_up_ctrl(void);
void joint_enable(void);
void joint_mouse_ctrl(void);
void one_return(void);

#endif
