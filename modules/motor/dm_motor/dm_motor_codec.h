#ifndef DM_MOTOR_CODEC_H
#define DM_MOTOR_CODEC_H

#include "dm_motor_def.h"

#include <stdint.h>

uint32_t dm_motor_float_to_uint(float value,
                                float minimum,
                                float maximum,
                                uint8_t bits);
float dm_motor_uint_to_float(uint32_t value,
                             float minimum,
                             float maximum,
                             uint8_t bits);
void dm_motor_codec_pack_mit(const motor_t *motor,
                             float position,
                             float velocity,
                             float kp,
                             float kd,
                             float torque,
                             uint8_t data[8]);
void dm_motor_codec_pack_pos(float position, float velocity, uint8_t data[8]);
void dm_motor_codec_pack_spd(float velocity, uint8_t data[4]);
void dm_motor_codec_pack_psi(float position,
                             float velocity,
                             float current,
                             uint8_t data[8]);
void dm_motor_codec_parse_feedback(motor_t *motor, const uint8_t data[8]);

#endif // DM_MOTOR_CODEC_H
