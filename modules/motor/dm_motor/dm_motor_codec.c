#include "dm_motor_codec.h"

#include <stddef.h>
#include <string.h>

#define DM_KP_MIN (0.0f)
#define DM_KP_MAX (500.0f)
#define DM_KD_MIN (0.0f)
#define DM_KD_MAX (5.0f)

uint32_t dm_motor_float_to_uint(float value,
                                float minimum,
                                float maximum,
                                uint8_t bits)
{
  if ((maximum <= minimum) || (bits == 0U) || (bits > 31U))
  {
    return 0U;
  }

  if (value < minimum)
  {
    value = minimum;
  }
  else if (value > maximum)
  {
    value = maximum;
  }

  const uint32_t span = (1UL << bits) - 1UL;
  return (uint32_t)((value - minimum) * (float)span /
                    (maximum - minimum));
}

float dm_motor_uint_to_float(uint32_t value,
                             float minimum,
                             float maximum,
                             uint8_t bits)
{
  if ((maximum <= minimum) || (bits == 0U) || (bits > 31U))
  {
    return minimum;
  }

  const uint32_t span = (1UL << bits) - 1UL;
  if (value > span)
  {
    value = span;
  }
  return ((float)value * (maximum - minimum) / (float)span) + minimum;
}

static void dm_motor_codec_write_float_le(float value, uint8_t data[4])
{
  uint8_t bytes[sizeof(float)];
  memcpy(bytes, &value, sizeof(bytes));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  memcpy(data, bytes, sizeof(bytes));
#else
  for (uint8_t i = 0U; i < sizeof(bytes); ++i)
  {
    data[i] = bytes[sizeof(bytes) - 1U - i];
  }
#endif
}

void dm_motor_codec_pack_mit(const motor_t *motor,
                             float position,
                             float velocity,
                             float kp,
                             float kd,
                             float torque,
                             uint8_t data[8])
{
  if ((motor == NULL) || (data == NULL))
  {
    return;
  }

  const uint32_t p = dm_motor_float_to_uint(
      position, -motor->tmp.PMAX, motor->tmp.PMAX, 16U);
  const uint32_t v = dm_motor_float_to_uint(
      velocity, -motor->tmp.VMAX, motor->tmp.VMAX, 12U);
  const uint32_t t = dm_motor_float_to_uint(
      torque, -motor->tmp.TMAX, motor->tmp.TMAX, 12U);
  const uint32_t kp_raw =
      dm_motor_float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12U);
  const uint32_t kd_raw =
      dm_motor_float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12U);

  data[0] = (uint8_t)(p >> 8U);
  data[1] = (uint8_t)p;
  data[2] = (uint8_t)(v >> 4U);
  data[3] = (uint8_t)((v << 4U) | (kp_raw >> 8U));
  data[4] = (uint8_t)kp_raw;
  data[5] = (uint8_t)(kd_raw >> 4U);
  data[6] = (uint8_t)((kd_raw << 4U) | (t >> 8U));
  data[7] = (uint8_t)t;
}

void dm_motor_codec_pack_pos(float position, float velocity, uint8_t data[8])
{
  dm_motor_codec_write_float_le(position, &data[0]);
  dm_motor_codec_write_float_le(velocity, &data[4]);
}

void dm_motor_codec_pack_spd(float velocity, uint8_t data[4])
{
  dm_motor_codec_write_float_le(velocity, data);
}

void dm_motor_codec_pack_psi(float position,
                             float velocity,
                             float current,
                             uint8_t data[8])
{
  const uint16_t velocity_raw = (uint16_t)(velocity * 100.0f);
  const uint16_t current_raw = (uint16_t)(current * 10000.0f);
  dm_motor_codec_write_float_le(position, &data[0]);
  data[4] = (uint8_t)velocity_raw;
  data[5] = (uint8_t)(velocity_raw >> 8U);
  data[6] = (uint8_t)current_raw;
  data[7] = (uint8_t)(current_raw >> 8U);
}

void dm_motor_codec_parse_feedback(motor_t *motor, const uint8_t data[8])
{
  if ((motor == NULL) || (data == NULL))
  {
    return;
  }

  motor->para.id = data[0] & 0x0F;
  motor->para.state = data[0] >> 4U;
  motor->para.p_int = ((int)data[1] << 8U) | data[2];
  motor->para.v_int = ((int)data[3] << 4U) | (data[4] >> 4U);
  motor->para.t_int = ((int)(data[4] & 0x0FU) << 8U) | data[5];
  motor->para.pos = dm_motor_uint_to_float(
      (uint32_t)motor->para.p_int, -motor->tmp.PMAX, motor->tmp.PMAX, 16U);
  motor->para.vel = dm_motor_uint_to_float(
      (uint32_t)motor->para.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12U);
  motor->para.tor = dm_motor_uint_to_float(
      (uint32_t)motor->para.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12U);
  motor->para.Tmos = (float)data[6];
  motor->para.Tcoil = (float)data[7];
}
