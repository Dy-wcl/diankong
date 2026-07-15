#include "dj_motor_drv.h"

#include "dj_motor_codec.h"

#include <stddef.h>
#include <string.h>

void dj_motor_init(dj_motor_t *motor,
                   dj_motor_type_e type,
                   uint16_t motor_id)
{
  if (motor == NULL)
  {
    return;
  }

  memset(motor, 0, sizeof(*motor));
  motor->motor_id = (dj_motor_id_e)motor_id;
  motor->type = type;
  motor->control.motor_id = motor_id;
  motor->control.type = type;
}

err_t dj_CAN_Send_Data(STM32CAN_t *can,
                       uint32_t ctrl_id,
                       const int16_t current[4])
{
  uint8_t payload[BSP_CAN_DATA_SIZE];

  if ((can == NULL) || (current == NULL))
  {
    return PTR_NULL;
  }

  dj_motor_codec_pack_currents(current, payload);
  return STM32CAN_Send(can, ctrl_id, payload, sizeof(payload));
}

err_t dj_motor_set_current(STM32CAN_t *can,
                           dj_motor_group_e group,
                           int16_t current1,
                           int16_t current2,
                           int16_t current3,
                           int16_t current4)
{
  const int16_t current[4] = {current1, current2, current3, current4};

  return dj_CAN_Send_Data(can, (uint32_t)group, current);
}

void dj_motor_parse_feedback(dj_motor_t *motor, const uint8_t data[8])
{
  if (motor == NULL)
  {
    return;
  }

  dj_motor_codec_parse_feedback(&motor->feedback, motor->type, data);
}

uint16_t dj_motor_get_encoder_res(dj_motor_type_e type)
{
  return dj_motor_codec_encoder_resolution(type);
}

float dj_motor_get_reduction_ratio(dj_motor_type_e type)
{
  return dj_motor_codec_reduction_ratio(type);
}

float dj_motor_encoder_to_angle(uint16_t encoder, uint16_t encoder_res)
{
  return dj_motor_codec_encoder_to_angle(encoder, encoder_res);
}

float dj_motor_rpm_to_rads(int16_t rpm, float reduction_ratio)
{
  return dj_motor_codec_rpm_to_rads(rpm, reduction_ratio);
}

void dj_motor_clear_control(dj_motor_t *motor)
{
  if (motor == NULL)
  {
    return;
  }

  motor->control.current_set = 0;
  motor->control.speed_set = 0.0f;
  motor->control.angle_set = 0.0f;
}
