/**
 * @file chassis_task.h
 * @brief 底盘控制任务头文件
 */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "comp_cmd.h"

/**
 * @brief 底盘任务状态
 * @details 记录底盘任务最近一次的初始化或控制状态
 */
extern volatile err_t chassis_status;

/**
 * @brief 底盘任务入口函数
 * @param argument FreeRTOS 任务参数（未使用）
 */
void start_chassis_task(void *argument);

#endif /* CHASSIS_TASK_H */
