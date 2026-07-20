#include "chassis_task.h"

#include "chassis_control.h"

#include "FreeRTOS.h"
#include "task.h"

#include "dr16.h"
#include "main.h"

extern DR16_t *dr16;

volatile err_t chassis_status = PENDING;

/**
 * @brief 底盘任务
 * @details 初始化完成后统一在任务循环中完成快照读取、控制计算与电流帧发送。
 */
void chassis_task(void *argument)
{
  (void)argument;

  const err_t init_result = chassis_control_init();
  const bool system_ready = (init_result == OK);
  chassis_status = init_result;

  for (;;)
  {
    if (system_ready)
    {
      cmd_rc_t command = {0};
      bool remote_online = false;

      if (dr16 != NULL)
      {
        const err_t snapshot_result =
            DR16_GetSnapshot(dr16, &command, &remote_online);
        if (snapshot_result != OK)
        {
          remote_online = false;
        }
      }

      /* 统一在底盘任务中完成控制计算与电流帧发送。 */
      chassis_status = Chassis_Mode(&command, remote_online, HAL_GetTick());
    }
    else
    {
      (void)chassis_control_force_stop();
    }

    vTaskDelay(pdMS_TO_TICKS(2U));
  }
}
