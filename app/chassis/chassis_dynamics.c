/**
 * @file chassis_dynamics.c
 * @brief 底盘动力学模块 - 实现麦克纳姆轮底盘的运动学逆解和前馈补偿
 * @details 该模块负责将底盘目标速度转换为各轮速度，并根据姿态角进行重力补偿
 */

#include "chassis_dynamics.h"

#include <math.h>

/* ==================== 物理参数定义 ==================== */
#define G 20.0f    // 车重，用于计算重力补偿力矩
#define R 0.076f   // 麦克纳姆轮半径 (m)，轮子的有效半径
#define K 1000.0f  // 力矩转换系数，将力转换为电机输出的比例系数

/* ==================== 静态变量 - 姿态信息 ==================== */
static float pitch_;  // 当前俯仰角 (rad)，底盘相对水平面的前后倾斜角度
static float roll_;   // 当前横滚角 (rad)，底盘相对水平面的左右倾斜角度
static float mp_;     // 机械俯仰角偏移 (rad)，底盘机械结构的固有俯仰角偏置

/**
 * @brief 设置底盘姿态角度
 * @details 更新底盘当前的姿态信息，用于后续的重力补偿计算
 *
 * @param y 偏航角 yaw (rad) - 未使用，保留接口
 * @param p 俯仰角 pitch (rad) - 底盘前后倾斜角度
 * @param r 横滚角 roll (rad) - 底盘左右倾斜角度
 * @param my 偏航角速度 (rad/s) - 未使用，保留接口
 * @param mp 机械俯仰角偏移 (rad) - 底盘机械结构的固有俯仰角
 *
 * @note 该函数通常由姿态估计模块（如IMU）定期调用以更新姿态数据
 */
void chassis_dynamics_set_attitude(float y, float p, float r, float my,
                                   float mp) {
  (void)y;     // 偏航角在本模块中未使用
  (void)my;    // 偏航角速度在本模块中未使用
  pitch_ = p;  // 保存俯仰角
  roll_ = r;   // 保存横滚角
  mp_ = mp;    // 保存机械俯仰角偏移
}

/**
 * @brief 麦克纳姆轮运动学逆解
 * @details 将底盘坐标系下的速度分量转换为四个麦克纳姆轮的目标角速度
 *
 * 麦克纳姆轮布局（俯视图）：
 *    前
 *  4     3
 *    \ /
 *    / \
 *  2     1
 *    后
 *
 * 转换矩阵基于45°麦克纳姆轮的运动学模型：
 * - 系数 s = √2/2 ≈ 0.707（45度角的正弦/余弦值）
 * - 每个轮子的速度由 vx、vy 和 wz 三个分量线性组合得到
 *
 * @param vx 底盘X方向线速度，正方向为前进（+X）
 * @param vy 底盘Y方向线速度，正方向为左移（+Y）
 * @param wz 底盘绕Z轴角速度，正方向为顺时针
 * @param o[4] 输出数组，顺序固定为 [FL, FR, RL, RR]：
 *             o[0]: 左前轮 FL —— 4 号电机
 *             o[1]: 右前轮 FR —— 3 号电机
 *             o[2]: 左后轮 RL —— 2 号电机
 *             o[3]: 右后轮 RR —— 1 号电机
 *
 * 坐标系约定：
 *             X：前，Y：左，Z：上
 *             wz > 0：从上往下看顺时针旋转
 */
void chassis_dynamics_inverse(float vx, float vy, float wz, float o[4]) {
  const float s = .70710678118f;  // √2/2 = sin(45°) = cos(45°)

  /*
   * 底盘坐标系：
   *   +X：前
   *   +Y：左
   *   +Z：上
  *   +wz：顺时针
   *
   * 输出顺序固定为 [FL, FR, RL, RR] = [4, 3, 2, 1]。
   * 注意：这里是“运动学正方向”，不等同于电机机械正转方向。
   * 电机正反转由 chassis_control.c 中的 reversed 参数统一处理。
   */
  o[0] = -s * vx - s * vy + wz;  // FL：4号
  o[1] = s * vx - s * vy + wz;   // FR：3号
  o[2] = -s * vx + s * vy + wz;  // RL：2号
  o[3] = s * vx + s * vy + wz;   // RR：1号
}

/**
 * @brief 基于姿态的前馈补偿
 * @details 根据底盘当前的俯仰角和横滚角，计算重力在各轮上产生的力矩补偿
 *
 * 补偿原理：
 * - 当底盘倾斜时，重力会在倾斜方向产生额外的力矩
 * - 通过计算重力分量在各轮上的影响，提前补偿以维持稳定控制
 * - 使用 sin(角度) 计算重力在倾斜方向的分量
 *
 * 力矩计算公式：
 * - q = √2 * R * K：单位重力加速度对应的轮子力矩
 * - 每个轮子的补偿力矩取决于横滚角和俯仰角的组合影响
 *
 * @param o[4] 输出数组，顺序固定为 [FL, FR, RL, RR]：
 *             o[0]: 左前轮（4号）
 *             o[1]: 右前轮（3号）
 *             o[2]: 左后轮（2号）
 *             o[3]: 右后轮（1号）
 *
 * @note 该补偿值应叠加到PID控制输出上，以提高响应速度和控制精度
 */
void chassis_dynamics_feedforward(float o[4]) {
  float p = pitch_ - mp_;            // 实际俯仰角 = 测量俯仰角 - 机械偏移角
  float r = roll_;                   // 横滚角
  float q = 1.41421356237f * R * K;  // √2 * R * K，归一化系数

  // 计算各轮的重力补偿力矩
  // 符号由轮子位置和重力方向决定
  o[0] = (-G * sinf(r) - G * sinf(p)) * q;  // 左前轮：横滚负向、俯仰负向
  o[1] = (G * sinf(r) - G * sinf(p)) * q;   // 右前轮：横滚正向、俯仰负向
  o[2] = (-G * sinf(r) + G * sinf(p)) * q;  // 左后轮：横滚负向、俯仰正向
  o[3] = (G * sinf(r) + G * sinf(p)) * q;   // 右后轮：横滚正向、俯仰正向
}
