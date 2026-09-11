#include "vt13_task.h"
#include "../../modules/vt13/vt13.h"
#include "FreeRTOS.h"
#include "comp_utils.h"
#include "task.h"
#include "usart.h"

vt13_t *vt13 = NULL;

void vt13_task(void *argument) {
  RM_UNUSED(argument);
  static vt13_t instance;
  err_t status = vt13_init(&instance, &huart3);
  vt13 = &instance;
  if (status == OK)
    status = vt13_start(vt13);
  ASSERT(status == OK);
  if (status != OK) {
    vTaskDelete(NULL);
    return;
  }
  for (;;)
    vt13_update(vt13, 20u);
}
