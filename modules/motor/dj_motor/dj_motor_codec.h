#ifndef DJ_MOTOR_CODEC_H
#define DJ_MOTOR_CODEC_H

#include "dj_motor_def.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! 将四路有符号电流按 DJI 协议编码为大端字节序。
void dj_motor_codec_pack_currents(const int16_t current[4],
                                  uint8_t payload[8]);

//! 解析 8 字节 DJI 反馈并更新派生的角度和速度量。
void dj_motor_codec_parse_feedback(dj_motor_feedback_t *feedback,
                                   dj_motor_type_e type,
                                   const uint8_t data[8]);

//! 返回指定电机的编码器分辨率。
uint16_t dj_motor_codec_encoder_resolution(dj_motor_type_e type);

//! 返回指定电机的减速比。
float dj_motor_codec_reduction_ratio(dj_motor_type_e type);

//! 将编码器值转换为弧度。
float dj_motor_codec_encoder_to_angle(uint16_t encoder,
                                      uint16_t encoder_resolution);

//! 将电机转速转换为输出轴角速度。
float dj_motor_codec_rpm_to_rads(int16_t rpm, float reduction_ratio);

#ifdef __cplusplus
}
#endif

#endif // DJ_MOTOR_CODEC_H
