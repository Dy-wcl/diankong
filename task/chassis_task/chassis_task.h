#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "comp_cmd.h"

/* 底盘任务最近一次控制/初始化状态（便于调试与联调观察） */
extern volatile err_t chassis_status;

void chassis_task(void *argument);

#endif /* CHASSIS_TASK_H */
