/**
 * @file chassis_kinematics.h
 * @brief 麦克纳姆轮底盘逆运动学纯函数接口。
 */
#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include <stdint.h>

/** 底盘轮组数量（左前/右前/左后/右后）。 */
#define CHASSIS_WHEEL_COUNT (4U)

/**
 * @brief 将底盘速度指令分解为四轮目标转速。
 *
 * 电机布局:
 *   前方
 *    1(左前)  2(右前)
 *    3(左后)  4(右后)
 *
 * 参考工程混控公式:
 *   M1 = -lateral + forward - yaw
 *   M2 =  lateral + forward - yaw
 *   M3 =  lateral - forward - yaw
 *   M4 = -lateral - forward - yaw
 *
 * Args:
 *   forward: 前后速度指令（正=前进方向约定与任务层一致）。
 *   lateral: 左右速度指令。
 *   yaw: 旋转指令。
 *   scale: 指令到目标 RPM 的比例系数。
 *   wheel_rpm: 输出四轮目标转速，不可为 NULL。
 */
void chassis_kinematics_mecanum(float forward, float lateral, float yaw,
                                float scale, float wheel_rpm[CHASSIS_WHEEL_COUNT]);

#endif /* CHASSIS_KINEMATICS_H */
