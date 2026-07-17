#include "dm_motor_ctrl.h"

#include <stddef.h>
#include <string.h>

motor_t motor[num];

typedef struct
{
  uint16_t id;
  uint16_t mst_id;
  float vmax;
  float tmax;
} dm_motor_default_config_t;

static const dm_motor_default_config_t dm_motor_default_config[num] = {
    {0x10U, 0x20U, 45.0f, 20.0f},
    {0x11U, 0x21U, 45.0f, 40.0f},
    {0x12U, 0x22U, 45.0f, 40.0f},
    {0x13U, 0x23U, 30.0f, 10.0f},
    {0x14U, 0x24U, 30.0f, 10.0f},
    {0x15U, 0x25U, 30.0f, 10.0f},
};

static void dm_motor_rx_callback(STM32CAN_t *can,
                                 const BSP_CAN_Frame_t *frame,
                                 void *context)
{
  (void)can;
  (void)context;

  if ((frame == NULL) ||
      (frame->ide_ != CAN_ID_STD) ||
      (frame->rtr_ != CAN_RTR_DATA) ||
      (frame->size_ != BSP_CAN_DATA_SIZE) ||
      (frame->id_ < 0x20U) ||
      (frame->id_ > 0x25U))
  {
    return;
  }

  motor_t *target = &motor[frame->id_ - 0x20U];
  dm_motor_fbdata(target, frame->data_);
  receive_motor_data(target, frame->data_);
}

void dm_motor_init(void)
{
  memset(motor, 0, sizeof(motor));
  for (uint8_t i = 0U; i < (uint8_t)num; ++i)
  {
    motor[i].id = dm_motor_default_config[i].id;
    motor[i].mst_id = dm_motor_default_config[i].mst_id;
    motor[i].tmp.read_flag = 1U;
    motor[i].ctrl.mode = (uint8_t)pos_mode;
    motor[i].tmp.PMAX = 3.141592f;
    motor[i].tmp.VMAX = dm_motor_default_config[i].vmax;
    motor[i].tmp.TMAX = dm_motor_default_config[i].tmax;
  }
}

err_t dm_motor_attach_can(STM32CAN_t *can)
{
  if (can == NULL)
  {
    return PTR_NULL;
  }
  return STM32CAN_SubscribeRx(can, dm_motor_rx_callback, NULL);
}
