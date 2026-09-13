/**
 * @file chassis_control.h
 * @brief 麦克纳姆轮底盘控制接口
 */
#ifndef CALUCATE_CHASSIS_CONTROL_H
#define CALUCATE_CHASSIS_CONTROL_H

#include "comp_cmd.h"
#include "main.h"

/** 底盘电机数量（麦克纳姆轮四轮） */
#define CHASSIS_MOTOR_COUNT (4U)

/** 底盘电机索引枚举 */
typedef enum {
  CHASSIS_MOTOR_FL = 0, /**< 左前轮 */
  CHASSIS_MOTOR_FR = 1, /**< 右前轮 */
  CHASSIS_MOTOR_RL = 2, /**< 左后轮 */
  CHASSIS_MOTOR_RR = 3  /**< 右后轮 */
} chassis_motor_index_e;

typedef struct {
  float vx;
  float vy;
  float wz;
} chassis_control_command_t;

typedef struct {
  chassis_control_command_t command;
  float command_limit;
  uint8_t enabled;
} chassis_control_state_t;

/**
 * @brief 初始化底盘总线与电机（须在 CAN Start 之前调用）
 * @return OK 成功；其他为错误码
 */
err_t chassis_control_init(void);

/**
 * @brief 初始化底盘电机速度 PID 参数
 */
void chassis_speed_pid_init(void);

/**
 * @brief 底盘模式控制（周期调用）
 */
/** 底盘模式控制（周期调用）。左下档进入 DR16 小陀螺模式。 */
void Chassis_Mode(void);

#endif /* CALUCATE_CHASSIS_CONTROL_H */
