#ifndef DJ_MOTOR_CTRL_H
#define DJ_MOTOR_CTRL_H

#include "dj_motor_drv.h"

extern dj_motor_t dj_motor[DJ_MOTOR_MAX];

#ifdef __cplusplus
extern "C" {
#endif

//! 初始化默认的八路 DJI 电机配置。
void dj_motor_system_init(void);

//! 向 1~4 号电机发送当前控制电流。
err_t dj_motor_control_send(STM32CAN_t *can);

//! 将 DJI 反馈处理器注册到 CAN 对象。
err_t dj_motor_attach_can(STM32CAN_t *can);

//! 按反馈标准帧 ID 查找电机对象。
dj_motor_t *dj_motor_get_feedback(uint16_t motor_id);

//! 设置 1~8 号电机的限幅电流。
void dj_motor_set_current_by_id(uint8_t motor_id, int16_t current);

#ifdef __cplusplus
}
#endif

#endif // DJ_MOTOR_CTRL_H
