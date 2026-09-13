#include "imu_can_task.h"

#include "FreeRTOS.h"
#include "bsp_can.h"
#include "can.h"
#include "comp_utils.h"
#include "imu_can.h"
#include "task.h"

extern STM32CAN_t can1_instance;
IMUCAN_t* imu_can_device = NULL;

void imu_can_task(void* argument) {
  RM_UNUSED(argument);
  static IMUCAN_t instance;
  err_t status = imu_can_init(&instance, &can1_instance, IMU_CAN_DEFAULT_ID,
                              IMU_CAN_DEFAULT_MST_ID);
  imu_can_device = &instance;
  instance.thread_alert = xTaskGetCurrentTaskHandle();
  if (status == OK) status = imu_can_start(imu_can_device);
  ASSERT(status == OK);
  if (status != OK) {
    vTaskDelete(NULL);
    return;
  }
  for (;;) imu_can_update(imu_can_device, IMUCAN_OFFLINE_TIMEOUT_MS);
}
