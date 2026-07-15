#include "dm_motor_param.h"

#include <stddef.h>
#include <string.h>

static const uint8_t dm_motor_register_sequence[] = {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U,
    10U, 11U, 12U, 13U, 14U, 15U, 16U, 17U, 18U, 19U,
    20U, 21U, 22U, 23U, 24U, 25U, 26U, 27U, 28U, 29U,
    30U, 31U, 32U, 33U, 34U, 35U, 36U,
    50U, 51U, 52U, 53U, 54U, 55U, 80U, 81U,
};

static float dm_motor_param_read_float(const uint8_t data[4])
{
  float value;
  uint8_t bytes[sizeof(float)];
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  memcpy(bytes, data, sizeof(bytes));
#else
  for (uint8_t i = 0U; i < sizeof(bytes); ++i)
  {
    bytes[sizeof(bytes) - 1U - i] = data[i];
  }
#endif
  memcpy(&value, bytes, sizeof(value));
  return value;
}

static uint32_t dm_motor_param_read_u32(const uint8_t data[4])
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

err_t read_all_motor_data(STM32CAN_t *can, motor_t *motor)
{
  if ((can == NULL) || (motor == NULL))
  {
    return PTR_NULL;
  }

  if (motor->tmp.read_flag == 0U)
  {
    return EMPTY;
  }

  const size_t index = (size_t)motor->tmp.read_flag - 1U;
  if (index >= sizeof(dm_motor_register_sequence))
  {
    return OUT_OF_RANGE;
  }

  return read_motor_data(can, motor->id, dm_motor_register_sequence[index]);
}

void receive_motor_data(motor_t *motor, const uint8_t data[8])
{
  if ((motor == NULL) || (data == NULL) ||
      (motor->tmp.read_flag == 0U) || (data[2] != 0x33U))
  {
    return;
  }

  const float f = dm_motor_param_read_float(&data[4]);
  const uint32_t u = dm_motor_param_read_u32(&data[4]);

  switch (data[3])
  {
    case 0U: motor->tmp.UV_Value = f; motor->tmp.read_flag = 2U; break;
    case 1U: motor->tmp.KT_Value = f; motor->tmp.read_flag = 3U; break;
    case 2U: motor->tmp.OT_Value = f; motor->tmp.read_flag = 4U; break;
    case 3U: motor->tmp.OC_Value = f; motor->tmp.read_flag = 5U; break;
    case 4U: motor->tmp.ACC = f; motor->tmp.read_flag = 6U; break;
    case 5U: motor->tmp.DEC = f; motor->tmp.read_flag = 7U; break;
    case 6U: motor->tmp.MAX_SPD = f; motor->tmp.read_flag = 8U; break;
    case 7U: motor->tmp.MST_ID = u; motor->tmp.read_flag = 9U; break;
    case 8U: motor->tmp.ESC_ID = u; motor->tmp.read_flag = 10U; break;
    case 9U: motor->tmp.TIMEOUT = u; motor->tmp.read_flag = 11U; break;
    case 10U: motor->tmp.cmode = u; motor->tmp.read_flag = 12U; break;
    case 11U: motor->tmp.Damp = f; motor->tmp.read_flag = 13U; break;
    case 12U: motor->tmp.Inertia = f; motor->tmp.read_flag = 14U; break;
    case 13U: motor->tmp.hw_ver = u; motor->tmp.read_flag = 15U; break;
    case 14U: motor->tmp.sw_ver = u; motor->tmp.read_flag = 16U; break;
    case 15U: motor->tmp.SN = u; motor->tmp.read_flag = 17U; break;
    case 16U: motor->tmp.NPP = u; motor->tmp.read_flag = 18U; break;
    case 17U: motor->tmp.Rs = f; motor->tmp.read_flag = 19U; break;
    case 18U: motor->tmp.Ls = f; motor->tmp.read_flag = 20U; break;
    case 19U: motor->tmp.Flux = f; motor->tmp.read_flag = 21U; break;
    case 20U: motor->tmp.Gr = f; motor->tmp.read_flag = 22U; break;
    case 21U: motor->tmp.PMAX = f; motor->tmp.read_flag = 23U; break;
    case 22U: motor->tmp.VMAX = f; motor->tmp.read_flag = 24U; break;
    case 23U: motor->tmp.TMAX = f; motor->tmp.read_flag = 25U; break;
    case 24U: motor->tmp.I_BW = f; motor->tmp.read_flag = 26U; break;
    case 25U: motor->tmp.KP_ASR = f; motor->tmp.read_flag = 27U; break;
    case 26U: motor->tmp.KI_ASR = f; motor->tmp.read_flag = 28U; break;
    case 27U: motor->tmp.KP_APR = f; motor->tmp.read_flag = 29U; break;
    case 28U: motor->tmp.KI_APR = f; motor->tmp.read_flag = 30U; break;
    case 29U: motor->tmp.OV_Value = f; motor->tmp.read_flag = 31U; break;
    case 30U: motor->tmp.GREF = f; motor->tmp.read_flag = 32U; break;
    case 31U: motor->tmp.Deta = f; motor->tmp.read_flag = 33U; break;
    case 32U: motor->tmp.V_BW = f; motor->tmp.read_flag = 34U; break;
    case 33U: motor->tmp.IQ_cl = f; motor->tmp.read_flag = 35U; break;
    case 34U: motor->tmp.VL_cl = f; motor->tmp.read_flag = 36U; break;
    case 35U: motor->tmp.can_br = u; motor->tmp.read_flag = 37U; break;
    case 36U: motor->tmp.sub_ver = u; motor->tmp.read_flag = 38U; break;
    case 50U: motor->tmp.u_off = f; motor->tmp.read_flag = 39U; break;
    case 51U: motor->tmp.v_off = f; motor->tmp.read_flag = 40U; break;
    case 52U: motor->tmp.k1 = f; motor->tmp.read_flag = 41U; break;
    case 53U: motor->tmp.k2 = f; motor->tmp.read_flag = 42U; break;
    case 54U: motor->tmp.m_off = f; motor->tmp.read_flag = 43U; break;
    case 55U: motor->tmp.dir = f; motor->tmp.read_flag = 44U; break;
    case 80U: motor->tmp.p_m = f; motor->tmp.read_flag = 45U; break;
    case 81U: motor->tmp.x_out = f; motor->tmp.read_flag = 0U; break;
    default: break;
  }
}
