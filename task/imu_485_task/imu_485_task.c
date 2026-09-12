#include "imu_485_task.h"

#include "FreeRTOS.h"
#include "comp_utils.h"
#include "imu_rs485.h"
#include "task.h"
#include "usart.h"

IMU485_t *imu_485_device = NULL;

void imu_485_task(void *argument)
{
  RM_UNUSED(argument);

  static IMU485_t instance;
  err_t status = imu_485_init(&instance, &huart5);
  imu_485_device = &instance;
  instance.thread_alert = xTaskGetCurrentTaskHandle();

  if (status == OK) {
    status = imu_485_start(imu_485_device);
  }
  ASSERT(status == OK);
  if (status != OK) {
    vTaskDelete(NULL);
    return;
  }

  for (;;) {
    imu_485_update(imu_485_device, IMU485_OFFLINE_TIMEOUT_MS);
  }
}
