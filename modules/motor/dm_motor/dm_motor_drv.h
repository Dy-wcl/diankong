#ifndef DM_MOTOR_DRV_H
#define DM_MOTOR_DRV_H

#include "bsp_can.h"
#include "dm_motor_def.h"

#define MIT_MODE (0x000U)
#define POS_MODE (0x100U)
#define SPD_MODE (0x200U)
#define PSI_MODE (0x300U)

typedef enum
{
  Motor1,
  Motor2,
  Motor3,
  Motor4,
  Motor5,
  Motor6,
  num
} motor_num;

typedef enum
{
  mit_mode = 1,
  pos_mode = 2,
  spd_mode = 3,
  psi_mode = 4
} mode_e;

err_t dm_motor_ctrl_send(STM32CAN_t *can, motor_t *motor);
err_t dm_motor_enable(STM32CAN_t *can, motor_t *motor);
err_t dm_motor_disable(STM32CAN_t *can, motor_t *motor);
void dm_motor_clear_para(motor_t *motor);
err_t dm_motor_clear_err(STM32CAN_t *can, motor_t *motor);
void dm_motor_fbdata(motor_t *motor, const uint8_t data[8]);

err_t enable_motor_mode(STM32CAN_t *can, uint16_t motor_id, uint16_t mode_id);
err_t disable_motor_mode(STM32CAN_t *can, uint16_t motor_id, uint16_t mode_id);
err_t save_pos_zero(STM32CAN_t *can, uint16_t motor_id, uint16_t mode_id);
err_t clear_err(STM32CAN_t *can, uint16_t motor_id, uint16_t mode_id);
err_t mit_ctrl(STM32CAN_t *can, motor_t *motor, uint16_t motor_id,
               float pos, float vel, float kp, float kd, float tor);
err_t pos_ctrl(STM32CAN_t *can, uint16_t motor_id, float pos, float vel);
err_t spd_ctrl(STM32CAN_t *can, uint16_t motor_id, float vel);
err_t psi_ctrl(STM32CAN_t *can, uint16_t motor_id,
               float pos, float vel, float cur);
err_t read_motor_data(STM32CAN_t *can, uint16_t id, uint8_t rid);
err_t read_motor_ctrl_fbdata(STM32CAN_t *can, uint16_t id);
err_t write_motor_data(STM32CAN_t *can, uint16_t id, uint8_t rid,
                       uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
err_t save_motor_data(STM32CAN_t *can, uint16_t id);

#endif // DM_MOTOR_DRV_H
