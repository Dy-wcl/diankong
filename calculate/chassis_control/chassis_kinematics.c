/**
 * @file chassis_kinematics.c
 * @brief 麦克纳姆轮底盘逆运动学实现。
 *
 * 纯函数：无全局状态，仅将机体速度指令分解为四轮目标转速。
 * 公式与参考工程混控一致，便于与历史标定对照。
 */

#include "chassis_kinematics.h"

#include <stddef.h>

/**
 * @brief 将底盘速度指令分解为四轮目标转速。
 *
 * 内部先将 forward/lateral/yaw 乘以 scale 得到 vx/vy/wz 量级，
 * 再按电机布局混控：
 *   前方
 *    1(左前)  2(右前)
 *    3(左后)  4(右后)
 *

  // 麦克纳姆轮逆运动学：将底盘速度 (vx, vy, wz) 分解为 4 个电机目标转速。
  //                      +vy(前)
  //                         ^
  //          II 象限       |       I 象限
  //       Motor1(左前)     |    Motor2(右前)
  //  -vx(左) <--------------+--------------> +vx(右)
  //       Motor3(左后)     |    Motor4(右后)
  //         III 象限       |      IV 象限
  //                         v
  //                      -vy(后)

 * wheel_rpm 为 NULL 时直接返回，不修改任何内存。
 */
void chassis_kinematics_mecanum(float forward, float lateral, float yaw,
                                float scale, float wheel_rpm[CHASSIS_WHEEL_COUNT]) {
  if (wheel_rpm == NULL) {
    return;
  }

  /* vx：横移分量；vy：前进分量；wz：旋转分量（均已含 scale） */
  const float vx = lateral * scale;
  const float vy = forward * scale;
  const float wz = yaw * scale;
//方向自行修改
  /* Motor1 左前: -vx + vy - wz */
  wheel_rpm[0] = -vx + vy - wz;
  /* Motor2 右前: +vx + vy - wz */
  wheel_rpm[1] = vx + vy - wz;
  /* Motor3 左后: +vx - vy - wz */
  wheel_rpm[2] = vx - vy - wz;
  /* Motor4 右后: -vx - vy - wz */
  wheel_rpm[3] = -vx - vy - wz;
}
