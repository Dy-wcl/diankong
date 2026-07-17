#ifndef DM_MOTOR_PARAM_H
#define DM_MOTOR_PARAM_H

#include "dm_motor_drv.h"

//! 按 read_flag 指向的寄存器发送下一条读取命令。
err_t read_all_motor_data(STM32CAN_t *can, motor_t *motor);

//! 解析一条寄存器响应并推进 read_flag。
void receive_motor_data(motor_t *motor, const uint8_t data[8]);

#endif // DM_MOTOR_PARAM_H
