/**
 * @file dm_motor_drv.h
 * @brief 达妙电机 CAN 协议发送与反馈解码接口。
 *
 * 本层负责把电机控制目标编码为达妙协议帧，并通过 bsp_can 提供的 STM32CAN_t
 * 设备发送；同时提供实时反馈解码、特殊命令帧以及寄存器读写命令。
 */
#ifndef __DM_MOTOR_DRV_H__
#define __DM_MOTOR_DRV_H__
#include "bsp_can.h"
#include "can.h"
#include "convert.h"
#include "dm_motor_def.h"
#include "main.h"

/** @name 控制模式对应的 CAN ID 偏移量
 * 发送控制帧时，实际标准帧 ID 由“电机 ID + 模式偏移量”组成。
 * @{
 */
#define MIT_MODE 0x000 ///< MIT 力矩/阻抗控制模式 ID 偏移量
#define POS_MODE 0x100 ///< 位置-速度控制模式 ID 偏移量
#define SPD_MODE 0x200 ///< 纯速度控制模式 ID 偏移量
#define PSI_MODE 0x300 ///< 位置-速度-电流控制模式 ID 偏移量
/** @} */

/** @name MIT 控制增益的协议映射范围
 * Kp 和 Kd 会按下列范围量化到控制帧中的 12 bit 无符号整数。
 * @{
 */
#define KP_MIN 0.0f   ///< Kp 最小允许值
#define KP_MAX 500.0f ///< Kp 最大允许值
#define KD_MIN 0.0f   ///< Kd 最小允许值
#define KD_MAX 5.0f   ///< Kd 最大允许值
/** @} */

/**
 * @brief 模块内预定义的电机索引。
 *
 * Motor1 至 Motor6 对应全局 motor 数组的六个元素；num 同时作为数组长度使用。
 */
typedef enum {
  Motor1, ///< 第 1 个电机，数组索引 0
  Motor2, ///< 第 2 个电机，数组索引 1
  Motor3, ///< 第 3 个电机，数组索引 2
  Motor4, ///< 第 4 个电机，数组索引 3
  Motor5, ///< 第 5 个电机，数组索引 4
  Motor6, ///< 第 6 个电机，数组索引 5

  num ///< 预定义电机数量，必须保持为枚举最后一项
} motor_num;

/**
 * @brief 上层选择的电机控制模式。
 *
 * 枚举值保存在 motor_ctrl_t::mode 中，由 dm_motor_ctrl_send() 选择对应的编码函数。
 */
typedef enum {
  mit_mode = 1, ///< MIT 力矩/阻抗控制：位置、速度、Kp、Kd、扭矩
  pos_mode = 2, ///< 位置-速度控制：位置、速度
  spd_mode = 3, ///< 纯速度控制：速度
  psi_mode = 4  ///< 位置-速度-电流控制：位置、速度、电流
} mode_e;

/**
 * @brief 按 motor->ctrl.mode 发送一次控制命令。
 * @param hcan 已初始化并可发送的 BSP CAN 设备。
 * @param motor 电机对象，提供命令 ID、映射范围和控制目标。
 * @note 底层发送结果不会通过本接口返回；需要诊断发送失败时应检查 bsp_can 状态。
 */
void dm_motor_ctrl_send(STM32CAN_t *hcan, motor_t *motor);

/**
 * @brief 按当前控制模式向电机发送使能特殊帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor 待使能电机对象。
 */
void dm_motor_enable(STM32CAN_t *hcan, motor_t *motor);

/**
 * @brief 按当前控制模式向电机发送失能特殊帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor 待失能电机对象。
 */
void dm_motor_disable(STM32CAN_t *hcan, motor_t *motor);

/**
 * @brief 清零电机对象中的控制目标和增益。
 * @param motor 待清理的电机对象。
 * @note 本函数只修改 RAM 中的控制结构，不会向电机发送命令。
 */
void dm_motor_clear_para(motor_t *motor);

/**
 * @brief 按当前控制模式向电机发送清除错误特殊帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor 待清除错误的电机对象。
 */
void dm_motor_clear_err(STM32CAN_t *hcan, motor_t *motor);

/**
 * @brief 解码一帧电机实时反馈并更新 motor->para。
 * @param motor 接收解析结果的电机对象。
 * @param rx_data 长度至少为 8 字节的达妙实时反馈数据区。
 */
void dm_motor_fbdata(motor_t *motor, const uint8_t *rx_data);

/**
 * @brief 向指定控制模式的 CAN ID 发送电机使能特殊帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor_id 电机基础标准帧 ID。
 * @param mode_id 控制模式 ID 偏移量，取 MIT_MODE、POS_MODE、SPD_MODE 或 PSI_MODE。
 */
void enable_motor_mode(STM32CAN_t *hcan, uint16_t motor_id, uint16_t mode_id);

/**
 * @brief 向指定控制模式的 CAN ID 发送电机失能特殊帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor_id 电机基础标准帧 ID。
 * @param mode_id 控制模式 ID 偏移量。
 */
void disable_motor_mode(STM32CAN_t *hcan, uint16_t motor_id,
                        uint16_t mode_id);

/**
 * @brief 编码并发送 MIT 力矩/阻抗控制帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor 电机对象，用于取得 PMAX、VMAX、TMAX 映射范围。
 * @param motor_id 电机基础标准帧 ID。
 * @param pos 目标位置，单位 rad。
 * @param vel 目标速度，单位 rad/s。
 * @param kp 位置比例增益。
 * @param kd 速度微分增益。
 * @param tor 前馈扭矩，单位 N·m。
 */
void mit_ctrl(STM32CAN_t *hcan, motor_t *motor, uint16_t motor_id, float pos,
              float vel, float kp, float kd, float tor);

/**
 * @brief 编码并发送位置-速度控制帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor_id 电机基础标准帧 ID。
 * @param pos 目标位置，单位 rad，以 32 bit IEEE 754 原始字节发送。
 * @param vel 目标速度，单位 rad/s，以 32 bit IEEE 754 原始字节发送。
 */
void pos_ctrl(STM32CAN_t *hcan, uint16_t motor_id, float pos, float vel);

/**
 * @brief 编码并发送纯速度控制帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor_id 电机基础标准帧 ID。
 * @param vel 目标速度，单位 rad/s，以 32 bit IEEE 754 原始字节发送。
 */
void spd_ctrl(STM32CAN_t *hcan, uint16_t motor_id, float vel);

/**
 * @brief 编码并发送位置-速度-电流控制帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor_id 电机基础标准帧 ID。
 * @param pos 目标位置，单位 rad。
 * @param vel 目标速度，单位 rad/s，协议发送值为 vel × 100。
 * @param cur 目标电流，单位 A，协议发送值为 cur × 10000。
 */
void psi_ctrl(STM32CAN_t *hcan, uint16_t motor_id, float pos, float vel,
              float cur);

/**
 * @brief 把当前位置保存为指定控制模式的零点。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor_id 电机基础标准帧 ID。
 * @param mode_id 控制模式 ID 偏移量。
 */
void save_pos_zero(STM32CAN_t *hcan, uint16_t motor_id, uint16_t mode_id);

/**
 * @brief 向指定控制模式发送清除错误特殊帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor_id 电机基础标准帧 ID。
 * @param mode_id 控制模式 ID 偏移量。
 */
void clear_err(STM32CAN_t *hcan, uint16_t motor_id, uint16_t mode_id);

/**
 * @brief 通过参数命令 ID 0x7FF 请求读取一个电机寄存器。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param id 目标电机的 11 bit 标准帧 ID。
 * @param rid 待读取的寄存器编号。
 */
void read_motor_data(STM32CAN_t *hcan, uint16_t id, uint8_t rid);

/**
 * @brief 请求电机回传当前控制状态。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param id 目标电机的 11 bit 标准帧 ID。
 */
void read_motor_ctrl_fbdata(STM32CAN_t *hcan, uint16_t id);

/**
 * @brief 通过参数命令 ID 0x7FF 写入一个 32 bit 寄存器值。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param id 目标电机的 11 bit 标准帧 ID。
 * @param rid 待写入的寄存器编号。
 * @param d0 32 bit 值的最低有效字节。
 * @param d1 32 bit 值的次低字节。
 * @param d2 32 bit 值的次高字节。
 * @param d3 32 bit 值的最高有效字节。
 */
void write_motor_data(STM32CAN_t *hcan, uint16_t id, uint8_t rid, uint8_t d0,
                      uint8_t d1, uint8_t d2, uint8_t d3);

/**
 * @brief 请求电机把当前参数保存到非易失存储器。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param id 目标电机的 11 bit 标准帧 ID。
 * @param rid 为保持接口一致而保留的参数；当前协议帧固定发送保存选择值 0x01。
 */
void save_motor_data(STM32CAN_t *hcan, uint16_t id, uint8_t rid);

#endif /* __DM_MOTOR_DRV_H__ */
