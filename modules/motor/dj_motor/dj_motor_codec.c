#include "dj_motor_codec.h"

#include <stddef.h>

#define DJ_MOTOR_PI (3.14159265358979323846f)

static uint16_t dj_motor_codec_read_u16_be(const uint8_t data[2])
{
  return (uint16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
}

static int16_t dj_motor_codec_read_i16_be(const uint8_t data[2])
{
  return (int16_t)dj_motor_codec_read_u16_be(data);
}

void dj_motor_codec_pack_currents(const int16_t current[4],
                                  uint8_t payload[8])
{
  if ((current == NULL) || (payload == NULL))
  {
    return;
  }

  for (uint8_t index = 0U; index < 4U; ++index)
  {
    uint16_t raw = (uint16_t)current[index];
    payload[index * 2U] = (uint8_t)(raw >> 8U);
    payload[index * 2U + 1U] = (uint8_t)raw;
  }
}

uint16_t dj_motor_codec_encoder_resolution(dj_motor_type_e type)
{
  switch (type)
  {
    case DJ_MOTOR_M3508:
      return M3508_ENCODER_RES;
    case DJ_MOTOR_M6020:
      return M6020_ENCODER_RES;
    case DJ_MOTOR_M2006:
      return M2006_ENCODER_RES;
    default:
      return M3508_ENCODER_RES;
  }
}

float dj_motor_codec_reduction_ratio(dj_motor_type_e type)
{
  switch (type)
  {
    case DJ_MOTOR_M3508:
      return M3508_REDUCTION_RATIO;
    case DJ_MOTOR_M6020:
      return M6020_REDUCTION_RATIO;
    case DJ_MOTOR_M2006:
      return M2006_REDUCTION_RATIO;
    default:
      return 1.0f;
  }
}

float dj_motor_codec_encoder_to_angle(uint16_t encoder,
                                      uint16_t encoder_resolution)
{
  if (encoder_resolution == 0U)
  {
    return 0.0f;
  }

  return ((float)encoder / (float)encoder_resolution) * 2.0f * DJ_MOTOR_PI;
}

float dj_motor_codec_rpm_to_rads(int16_t rpm, float reduction_ratio)
{
  if (reduction_ratio == 0.0f)
  {
    return 0.0f;
  }

  return (((float)rpm / 60.0f) * 2.0f * DJ_MOTOR_PI) / reduction_ratio;
}

void dj_motor_codec_parse_feedback(dj_motor_feedback_t *feedback,
                                   dj_motor_type_e type,
                                   const uint8_t data[8])
{
  uint16_t encoder_resolution;

  if ((feedback == NULL) || (data == NULL))
  {
    return;
  }

  feedback->last_temp = feedback->temp;
  feedback->encoder = dj_motor_codec_read_u16_be(&data[0]);
  feedback->rpm_speed = dj_motor_codec_read_i16_be(&data[2]);
  feedback->current = dj_motor_codec_read_i16_be(&data[4]);
  feedback->temp = data[6];

  encoder_resolution = dj_motor_codec_encoder_resolution(type);
  feedback->angle = dj_motor_codec_encoder_to_angle(feedback->encoder,
                                                     encoder_resolution);
  feedback->actual_angle_deg =
      ((float)feedback->encoder / (float)encoder_resolution) * 360.0f;
  feedback->speed_rads =
      dj_motor_codec_rpm_to_rads(feedback->rpm_speed,
                                 dj_motor_codec_reduction_ratio(type));
}
