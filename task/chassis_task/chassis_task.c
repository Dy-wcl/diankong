/**
 * @file chassis_task.c
 * @brief 底盘控制任务实现
 */
#include "chassis_task.h"

#include "FreeRTOS.h"
#include "bsp_can.h"
#include "can.h"
#include "chassis_control.h"
#include "task.h"

/* 底盘任务状态（用于调试和错误监控） */
volatile err_t chassis_status = PENDING;

/* CAN2 实例（用于底盘电机通信） */
extern STM32CAN_t can2_instance;

/**
 * @brief 底盘控制任务入口函数
 * @param argument FreeRTOS 任务参数（未使用）
 *
 * @details 任务执行流程：
 *   1. 初始化底盘 CAN 总线与电机
 *   2. 初始化电机速度环 PID 参数
 *   3. 启动 CAN 通信
 *   4. 周期性调用底盘控制函数（2ms 周期）
 */
void chassis_task(void* argument) {
  RM_UNUSED(argument);

  /* 初始化底盘总线与电机实例 */
  chassis_status = chassis_control_init();
  if (chassis_status != OK) {
    /* 初始化失败，任务挂起 */
    vTaskSuspend(NULL);
    return;
  }

  /* 初始化底盘电机速度环 PID */
  chassis_speed_pid_init();

  /* 启动 CAN2（之后不能再注册新电机） */
  err_t can_start_result = STM32CAN_Start(&can2_instance);
  if (can_start_result != OK) {
    chassis_status = can_start_result;
    vTaskSuspend(NULL);
    return;
  }

  /* 主控制循环 */
  while (1) {
    /* 执行底盘模式控制（根据遥控器状态） */
    Chassis_Mode();

    /* 2ms 控制周期 */
    vTaskDelay(pdMS_TO_TICKS(2U));
  }
}
