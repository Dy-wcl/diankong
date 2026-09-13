#include "joint_task.h"

#include "bsp_can.h"
#include "cmsis_os2.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_def.h"
#include "dm_motor_drv.h"
#include "joint_control.h"

extern motor_t motor[num];
extern float pos[num];
void joint_task(void* argument) {
  RM_UNUSED(argument);

  // 先初始化 DM 电机对象，补齐关节控制依赖的 ID、模式与力矩范围
  dm_motor_init();

  // save_pos_zero(&hcan1, motor[Motor1].id, PSI_MODE);
  // save_pos_zero(&hcan1, motor[Motor2].id, PSI_MODE);
  // save_pos_zero(&hcan1, motor[Motor3].id, PSI_MODE);
  // save_pos_zero(&hcan1, motor[Motor4].id, PSI_MODE);
  // save_pos_zero(&hcan1, motor[Motor5].id, PSI_MODE);
  // save_pos_zero(&hcan1, motor[Motor6].id, PSI_MODE);
  // 初始化关节位置为0
  // save_pos_zero(&hcan1, motor[Motor4].id, PSI_MODE);

  while (1) {
    // 保持稳定的 1ms 调度节拍，避免额外调度抖动放大 CAN 发送卡顿
    Joint_Mode();
    osDelay(1);
  }
}
