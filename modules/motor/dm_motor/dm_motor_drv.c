#include "dm_motor_drv.h"

#include "dm_motor_codec.h"

#include <stddef.h>
#include <string.h>

static uint16_t dm_motor_mode_offset(uint8_t mode)
{
  switch ((mode_e)mode)
  {
    case mit_mode: return MIT_MODE;
    case pos_mode: return POS_MODE;
    case spd_mode: return SPD_MODE;
    case psi_mode: return PSI_MODE;
    default: return UINT16_MAX;
  }
}

static err_t dm_motor_send_special(STM32CAN_t *can,
                                   uint16_t id,
                                   uint8_t command)
{
  uint8_t data[8];
  memset(data, 0xFF, sizeof(data));
  data[7] = command;
  return STM32CAN_Send(can, id, data, sizeof(data));
}

err_t dm_motor_enable(STM32CAN_t *can, motor_t *motor)
{
  if ((can == NULL) || (motor == NULL)) return PTR_NULL;
  const uint16_t offset = dm_motor_mode_offset(motor->ctrl.mode);
  return (offset == UINT16_MAX) ? ARG_ERR
                                : enable_motor_mode(can, motor->id, offset);
}

err_t dm_motor_disable(STM32CAN_t *can, motor_t *motor)
{
  if ((can == NULL) || (motor == NULL)) return PTR_NULL;
  const uint16_t offset = dm_motor_mode_offset(motor->ctrl.mode);
  if (offset == UINT16_MAX) return ARG_ERR;
  const err_t error = disable_motor_mode(can, motor->id, offset);
  if (error == OK) dm_motor_clear_para(motor);
  return error;
}

err_t dm_motor_ctrl_send(STM32CAN_t *can, motor_t *motor)
{
  if ((can == NULL) || (motor == NULL)) return PTR_NULL;
  switch ((mode_e)motor->ctrl.mode)
  {
    case mit_mode:
      return mit_ctrl(can, motor, motor->id, motor->ctrl.pos_set,
                      motor->ctrl.vel_set, motor->ctrl.kp_set,
                      motor->ctrl.kd_set, motor->ctrl.tor_set);
    case pos_mode:
      return pos_ctrl(can, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
    case spd_mode:
      return spd_ctrl(can, motor->id, motor->ctrl.vel_set);
    case psi_mode:
      return psi_ctrl(can, motor->id, motor->ctrl.pos_set,
                      motor->ctrl.vel_set, motor->ctrl.cur_set);
    default:
      return ARG_ERR;
  }
}

void dm_motor_clear_para(motor_t *motor)
{
  if (motor == NULL) return;
  motor->ctrl.pos_set = 0.0f;
  motor->ctrl.vel_set = 0.0f;
  motor->ctrl.tor_set = 0.0f;
  motor->ctrl.cur_set = 0.0f;
  motor->ctrl.kp_set = 0.0f;
  motor->ctrl.kd_set = 0.0f;
}

err_t dm_motor_clear_err(STM32CAN_t *can, motor_t *motor)
{
  if ((can == NULL) || (motor == NULL)) return PTR_NULL;
  const uint16_t offset = dm_motor_mode_offset(motor->ctrl.mode);
  return (offset == UINT16_MAX) ? ARG_ERR : clear_err(can, motor->id, offset);
}

void dm_motor_fbdata(motor_t *motor, const uint8_t data[8])
{
  dm_motor_codec_parse_feedback(motor, data);
}

err_t enable_motor_mode(STM32CAN_t *can, uint16_t motor_id, uint16_t mode_id)
{
  return dm_motor_send_special(can, motor_id + mode_id, 0xFCU);
}

err_t disable_motor_mode(STM32CAN_t *can, uint16_t motor_id, uint16_t mode_id)
{
  return dm_motor_send_special(can, motor_id + mode_id, 0xFDU);
}

err_t save_pos_zero(STM32CAN_t *can, uint16_t motor_id, uint16_t mode_id)
{
  return dm_motor_send_special(can, motor_id + mode_id, 0xFEU);
}

err_t clear_err(STM32CAN_t *can, uint16_t motor_id, uint16_t mode_id)
{
  return dm_motor_send_special(can, motor_id + mode_id, 0xFBU);
}

err_t mit_ctrl(STM32CAN_t *can, motor_t *motor, uint16_t motor_id,
               float pos, float vel, float kp, float kd, float tor)
{
  uint8_t data[8];
  dm_motor_codec_pack_mit(motor, pos, vel, kp, kd, tor, data);
  return STM32CAN_Send(can, motor_id + MIT_MODE, data, sizeof(data));
}

err_t pos_ctrl(STM32CAN_t *can, uint16_t motor_id, float pos, float vel)
{
  uint8_t data[8];
  dm_motor_codec_pack_pos(pos, vel, data);
  return STM32CAN_Send(can, motor_id + POS_MODE, data, sizeof(data));
}

err_t spd_ctrl(STM32CAN_t *can, uint16_t motor_id, float vel)
{
  uint8_t data[4];
  dm_motor_codec_pack_spd(vel, data);
  return STM32CAN_Send(can, motor_id + SPD_MODE, data, sizeof(data));
}

err_t psi_ctrl(STM32CAN_t *can, uint16_t motor_id,
               float pos, float vel, float cur)
{
  uint8_t data[8];
  dm_motor_codec_pack_psi(pos, vel, cur, data);
  return STM32CAN_Send(can, motor_id + PSI_MODE, data, sizeof(data));
}

err_t read_motor_data(STM32CAN_t *can, uint16_t id, uint8_t rid)
{
  const uint8_t data[4] = {(uint8_t)id, (uint8_t)((id >> 8U) & 0x07U),
                           0x33U, rid};
  return STM32CAN_Send(can, 0x7FFU, data, sizeof(data));
}

err_t read_motor_ctrl_fbdata(STM32CAN_t *can, uint16_t id)
{
  const uint8_t data[4] = {(uint8_t)id, (uint8_t)((id >> 8U) & 0x07U),
                           0xCCU, 0U};
  return STM32CAN_Send(can, 0x7FFU, data, sizeof(data));
}

err_t write_motor_data(STM32CAN_t *can, uint16_t id, uint8_t rid,
                       uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
  const uint8_t data[8] = {(uint8_t)(id & 0x0FU),
                           (uint8_t)((id >> 4U) & 0x0FU),
                           0x55U, rid, d0, d1, d2, d3};
  return STM32CAN_Send(can, 0x7FFU, data, sizeof(data));
}

err_t save_motor_data(STM32CAN_t *can, uint16_t id)
{
  const uint8_t data[4] = {(uint8_t)id, (uint8_t)((id >> 8U) & 0x07U),
                           0xAAU, 0x01U};
  return STM32CAN_Send(can, 0x7FFU, data, sizeof(data));
}
