#ifndef DM_MOTOR_CTRL_H
#define DM_MOTOR_CTRL_H

#include "dm_motor_drv.h"
#include "dm_motor_param.h"

extern motor_t motor[num];

//! 初始化六路默认 DM 电机配置。
void dm_motor_init(void);

//! 将 DM 反馈订阅者注册到 CAN 对象。
err_t dm_motor_attach_can(STM32CAN_t *can);

#endif // DM_MOTOR_CTRL_H
