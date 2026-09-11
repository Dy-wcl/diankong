/**
 * @file chassis_control.c
 * @brief 麦克纳姆轮底盘控制实现
 */
#include "chassis_control.h"
#include "bsp_can.h"
#include "can.h"
#include "dj_motor_ctrl.h"
#include "pid_location.h"
#include "vt13.h"
#include "chassis_dynamics.h"
#include "process.h"
#include <math.h>

extern vt13_cmd_rc_t vt13_cmd_rc;
extern uint8_t joint_enable_single;

/* 底盘电机总线与实例 */
static dj_motor_bus_t chassis_bus;
static dj_motor_t chassis_motors[CHASSIS_MOTOR_COUNT];

/* 速度环 PID 实例 */
static PIDInstance pid_speed[CHASSIS_MOTOR_COUNT];

/* 底盘状态与目标速度 */
static chassis_control_state_t chassis_control_state_;
static float motor_target_speed[CHASSIS_MOTOR_COUNT];
static float torque_ff_current[CHASSIS_MOTOR_COUNT];
#define CHASSIS_GRAVITY_N 20.0f
#define CHASSIS_TORQUE_TO_CURRENT 1000.0f
#define CHASSIS_WHEEL_RADIUS_M 0.076f

/**
 * @brief 初始化底盘 CAN 总线与四个 M3508 电机
 * @return OK 或错误码
 */
err_t chassis_control_init(void) {
  /* 获取 CAN2 的 BSP 对象（假设 CAN2 用于底盘） */
  BSP_CAN_t can_id = BSP_CAN_get_id(CAN2);
  if (can_id == BSP_CAN_ID_ERROR) {
    return NOT_FOUND;
  }

  STM32CAN_t *can2 = STM32CAN_GetInstance(can_id);
  if (can2 == NULL) {
    return PTR_NULL;
  }

  /* 初始化底盘总线 */
  err_t result = dj_motor_bus_init(&chassis_bus, can2);
  if (result != OK) {
    return result;
  }

  /* 初始化四个 M3508 底盘电机（设备 ID 1-4，控制组 0x200）
   * reversed 参数根据实际机械安装方向设置 */
  result = dj_motor_init(&chassis_motors[CHASSIS_MOTOR_FL], &chassis_bus,
                         DJ_MOTOR_M3508, 1, false);
  if (result != OK)
    return result;

  result = dj_motor_init(&chassis_motors[CHASSIS_MOTOR_FR], &chassis_bus,
                         DJ_MOTOR_M3508, 2, false);
  if (result != OK)
    return result;

  result = dj_motor_init(&chassis_motors[CHASSIS_MOTOR_RL], &chassis_bus,
                         DJ_MOTOR_M3508, 3, false);
  if (result != OK)
    return result;

  result = dj_motor_init(&chassis_motors[CHASSIS_MOTOR_RR], &chassis_bus,
                         DJ_MOTOR_M3508, 4, false);
  if (result != OK)
    return result;

  return OK;
}

static void chassis_speed_pid_init_single(PIDInstance *pid, float kp, float ki,
                                          float kd, float max_out) {
  PIDInit(pid, kp, ki, kd, max_out, 3000.0f, 0.0f,
          PID_Integral_Limit | PID_Derivative_On_Measurement |
              PID_OutputFilter | PID_DerivativeFilter,
          0.0f, 0.0f, 0.0002f, 0.0002f);
}

void chassis_speed_pid_init(void) {
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_FL], 12.0f, 0.0f, 0.0f,
                                12000.0f);
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_FR], 8.0f, 0.0f, 0.0f,
                                12000.0f);
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_RL], 8.0f, 0.0f, 0.0f,
                                12000.0f);
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_RR], 14.0f, 2.0f, 0.0f,
                                12000.0f);
}

static void chassis_motor_pid_control_speed(uint8_t motor_index,
                                            float target_speed) {
  if (motor_index >= CHASSIS_MOTOR_COUNT)
    return;

  /* 获取电机反馈 */
  dj_motor_feedback_t feedback;
  if (dj_motor_get_feedback(&chassis_motors[motor_index], &feedback) != OK) {
    return;
  }

  /* 速度环 PID 控制（反馈为 rad/s，需要转换或统一单位）
   * 这里假设目标速度与反馈速度单位一致（RPM） */
  float current_speed = feedback.speed_rpm;
  float motor_current =
      PIDCalculate(&pid_speed[motor_index], current_speed, target_speed);

  /* 限幅到 M3508 允许范围 */
  motor_current += torque_ff_current[motor_index];
  motor_current = CONSTRAIN(motor_current, -16384.0f, 16384.0f);

  /* 发送命令（写齐后自动发送） */
  dj_motor_set_command(&chassis_motors[motor_index], (int16_t)motor_current);
}

static void chassis_stop(void) {
  chassis_control_state_.command.vx = 0.0f;
  chassis_control_state_.command.vy = 0.0f;
  chassis_control_state_.command.wz = 0.0f;

  /* 使用 zero_and_flush 安全停机（推荐的停机方式） */
  dj_motor_zero_and_flush(&chassis_bus, DJ_MOTOR_GROUP_200);
}

static void chassis_control(void) {
  chassis_dynamics_feedforward(torque_ff_current);
  chassis_control_state_.command.vx = -vt13_cmd_rc.ch.l.x * 3000;
  chassis_control_state_.command.vy = -vt13_cmd_rc.ch.l.y * 3000;
  chassis_control_state_.command.wz = -vt13_cmd_rc.ch.r.x * 3000;

  // 麦克纳姆轮逆运动学：将底盘速度 (vx, vy, wz) 分解为 4 个电机目标转速。
  //                      +vy(前)
  //                         ^
  //          II 象限       |       I 象限
  //       Motor1(左前)     |    Motor2(右前)
  //  -vx(左) <--------------+--------------> +vx(右)
  //       Motor3(左后)     |    Motor4(右后)
  //         III 象限       |      IV 象限
  //                         v
  //                      -vy(后)
  /* 麦克纳姆轮运动学解算 */
  chassis_dynamics_inverse(chassis_control_state_.command.vx, chassis_control_state_.command.vy, chassis_control_state_.command.wz, motor_target_speed);

  /* 执行速度环 PID 控制（写齐后自动发送 CAN 帧） */
  for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
    chassis_motor_pid_control_speed(i, motor_target_speed[i]);
  }
}

void Chassis_Mode(void) {
  if (vt13_cmd_rc.mode_sw == vt13_CMD_SW_MID) {
    if (joint_enable_single == 1) {
      chassis_control();
    }
  } else if (vt13_cmd_rc.mode_sw == vt13_CMD_SW_UP ||
             vt13_cmd_rc.mode_sw == vt13_CMD_SW_DOWN) {
    chassis_stop();
  }
}
