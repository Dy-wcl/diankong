#include "dj_motor_ctrl.h"

#include <stddef.h>

dj_motor_t dj_motor[DJ_MOTOR_MAX];

typedef struct
{
  dj_motor_type_e type;
  dj_motor_id_e id;
} dj_motor_default_config_t;

static const dj_motor_default_config_t dj_motor_default_config[DJ_MOTOR_MAX] = {
    {DJ_MOTOR_M3508, DJ_MOTOR_1},
    {DJ_MOTOR_M3508, DJ_MOTOR_2},
    {DJ_MOTOR_M3508, DJ_MOTOR_3},
    {DJ_MOTOR_M3508, DJ_MOTOR_4},
    {DJ_MOTOR_M6020, DJ_MOTOR_5},
    {DJ_MOTOR_M6020, DJ_MOTOR_6},
    {DJ_MOTOR_M6020, DJ_MOTOR_7},
    {DJ_MOTOR_M6020, DJ_MOTOR_8},
};

static int16_t dj_motor_get_current_limit(dj_motor_type_e type)
{
  switch (type)
  {
    case DJ_MOTOR_M3508:
      return M3508_MAX_CURRENT;
    case DJ_MOTOR_M6020:
      return M6020_MAX_CURRENT;
    case DJ_MOTOR_M2006:
      return M2006_MAX_CURRENT;
    default:
      return M3508_MAX_CURRENT;
  }
}

static void dj_motor_rx_callback(STM32CAN_t *can,
                                 const BSP_CAN_Frame_t *frame,
                                 void *context)
{
  dj_motor_t *motor;

  (void)can;
  (void)context;

  if ((frame == NULL) ||
      (frame->ide_ != CAN_ID_STD) ||
      (frame->rtr_ != CAN_RTR_DATA) ||
      (frame->size_ != BSP_CAN_DATA_SIZE) ||
      (frame->id_ < (uint32_t)DJ_MOTOR_1) ||
      (frame->id_ > (uint32_t)DJ_MOTOR_8))
  {
    return;
  }

  motor = dj_motor_get_feedback((uint16_t)frame->id_);
  if (motor != NULL)
  {
    dj_motor_parse_feedback(motor, frame->data_);
  }
}

void dj_motor_system_init(void)
{
  for (uint8_t index = 0U; index < (uint8_t)DJ_MOTOR_MAX; ++index)
  {
    dj_motor_init(&dj_motor[index],
                  dj_motor_default_config[index].type,
                  (uint16_t)dj_motor_default_config[index].id);
  }
}

err_t dj_motor_control_send(STM32CAN_t *can)
{
  const int16_t current[4] = {
      dj_motor[DJ_MOTOR1].control.current_set,
      dj_motor[DJ_MOTOR2].control.current_set,
      dj_motor[DJ_MOTOR3].control.current_set,
      dj_motor[DJ_MOTOR4].control.current_set,
  };

  return dj_CAN_Send_Data(can, DJ_MOTOR_GROUP_1, current);
}

err_t dj_motor_attach_can(STM32CAN_t *can)
{
  if (can == NULL)
  {
    return PTR_NULL;
  }

  return STM32CAN_SubscribeRx(can, dj_motor_rx_callback, NULL);
}

dj_motor_t *dj_motor_get_feedback(uint16_t motor_id)
{
  if ((motor_id < (uint16_t)DJ_MOTOR_1) ||
      (motor_id > (uint16_t)DJ_MOTOR_8))
  {
    return NULL;
  }

  return &dj_motor[motor_id - (uint16_t)DJ_MOTOR_1];
}

void dj_motor_set_current_by_id(uint8_t motor_id, int16_t current)
{
  dj_motor_t *motor;
  int16_t limit;

  if ((motor_id == 0U) || (motor_id > (uint8_t)DJ_MOTOR_MAX))
  {
    return;
  }

  motor = &dj_motor[motor_id - 1U];
  limit = dj_motor_get_current_limit(motor->type);
  if (current > limit)
  {
    current = limit;
  }
  else if (current < -limit)
  {
    current = (int16_t)-limit;
  }

  motor->control.current_set = current;
}
