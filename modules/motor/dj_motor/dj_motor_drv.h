#ifndef DJ_MOTOR_DRV_H
#define DJ_MOTOR_DRV_H

#include "bsp_can.h"
#include "dj_motor_def.h"

#include <stdint.h>

typedef enum
{
  DJ_MOTOR1 = 0,
  DJ_MOTOR2,
  DJ_MOTOR3,
  DJ_MOTOR4,
  DJ_MOTOR5,
  DJ_MOTOR6,
  DJ_MOTOR7,
  DJ_MOTOR8,
  DJ_MOTOR_MAX
} dj_motor_num_e;

#ifdef __cplusplus
extern "C" {
#endif

//! 初始化单个 DJI 电机对象。
void dj_motor_init(dj_motor_t *motor,
                   dj_motor_type_e type,
                   uint16_t motor_id);

//! 发送一组四路 DJI 电流，固定使用 8 字节数据帧。
err_t dj_CAN_Send_Data(STM32CAN_t *can,
                       uint32_t ctrl_id,
                       const int16_t current[4]);

//! 以四个独立参数发送一组电机电流。
err_t dj_motor_set_current(STM32CAN_t *can,
                           dj_motor_group_e group,
                           int16_t current1,
                           int16_t current2,
                           int16_t current3,
                           int16_t current4);

//! 解析单帧 DJI 电机反馈。
void dj_motor_parse_feedback(dj_motor_t *motor, const uint8_t data[8]);

uint16_t dj_motor_get_encoder_res(dj_motor_type_e type);
float dj_motor_get_reduction_ratio(dj_motor_type_e type);
float dj_motor_encoder_to_angle(uint16_t encoder, uint16_t encoder_res);
float dj_motor_rpm_to_rads(int16_t rpm, float reduction_ratio);
void dj_motor_clear_control(dj_motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif // DJ_MOTOR_DRV_H
