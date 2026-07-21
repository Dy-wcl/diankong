/**
 * @file dm_motor_ctrl.h
 * @brief 达妙电机对象管理、反馈路由与参数读取接口。
 *
 * 本层管理六个电机对象，订阅 bsp_can 接收回调，并在中断回调中完成实时反馈解析和
 * 参数帧快照。任务上下文通过 read_all_motor_data() 推进参数轮询状态机。
 */
#ifndef __DM_MOTOR_CTRL_H__
#define __DM_MOTOR_CTRL_H__
#include "bsp_can.h"
#include "convert.h"
#include "dm_motor_drv.h"
#include "main.h"
#include <stdint.h>

/**
 * @brief 应用层保留的当前电机编号变量。
 * @note 本模块只声明该符号，不在此处定义或修改它。
 */
extern int8_t motor_id;

// 以下发送计数声明为历史调试接口，当前保持注释状态，不参与编译。
// extern uint32_t motor1_data_sent;
// extern uint32_t motor2_data_sent;
// extern uint32_t motor3_data_sent;
// extern uint32_t motor4_data_sent;

/**
 * @brief 六个预定义达妙电机对象。
 * @note 下标使用 motor_num 枚举中的 Motor1 至 Motor6。
 */
extern motor_t motor[num];

/**
 * @brief 32 bit 浮点数、无符号整数和原始字节的共享存储表示。
 *
 * 用于按达妙寄存器协议的原始字节序在 float 与 uint32_t 之间重解释数据，不进行
 * 数值类型转换。当前实现面向 STM32 小端平台。
 */
typedef union {
  float f_val;    ///< 32 bit IEEE 754 单精度浮点表示
  uint32_t u_val; ///< 32 bit 无符号整数表示
  uint8_t b_val[4]; ///< 四字节原始表示，索引 0 为最低地址字节
} float_type_u;

/**
 * @brief 初始化六个电机对象及其参数快照。
 *
 * 电机命令 ID 初始化为 0x10～0x15，反馈 ID 初始化为 0x20～0x25；默认进入位置-
 * 速度模式，并设置 MIT 协议使用的位置、速度和扭矩映射范围。
 */
void dm_motor_init(void);

/**
 * @brief 消费参数反馈快照，并按 read_flag 请求下一项寄存器。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor 待读取参数的电机对象。
 *
 * @note 应在任务上下文周期调用，不应在 CAN 中断回调中调用。函数每次最多处理一个
 *       已完成快照并发出一个新的寄存器读取命令。
 */
void read_all_motor_data(STM32CAN_t *hcan, motor_t *motor);

/**
 * @brief 解析一帧寄存器反馈并更新电机参数镜像。
 * @param motor 接收参数结果的电机对象。
 * @param data 长度至少为 8 字节、命令字为 0x33 的寄存器反馈数据区。
 *
 * read_flag 决定本次返回值写入哪个 esc_inf_t 成员；解析完成后自动推进到下一步，
 * 最后一项完成时把 read_flag 清零。
 */
void receive_motor_data(motor_t *motor, const uint8_t *data);

/**
 * @brief 把本模块的 CAN 接收回调订阅到指定 BSP CAN 设备。
 * @param can 已初始化的 BSP CAN 设备。
 * @return OK 表示订阅成功；can 为 NULL 时返回 PTR_NULL；其他错误码由
 *         STM32CAN_SubscribeRx() 透传。
 *
 * 订阅后仅接收 11 bit 标准数据帧，并把反馈 ID 0x20～0x25 路由到对应电机对象。
 */
err_t dm_motor_attach_can(STM32CAN_t *can);

// CAN 接收回调的函数类型由 bsp_can.h 统一声明，本头文件只公开订阅入口。

/**
 * @brief 解析达妙实时反馈帧。
 * @param motor 接收解析结果的电机对象。
 * @param rx_data 长度至少为 8 字节的实时反馈数据区。
 * @note 当前五文件实现未提供此兼容声明的函数体；实时反馈请调用 dm_motor_fbdata()。
 */
void dm_motor_parse_fb_data(motor_t *motor, uint8_t *rx_data);

/**
 * @brief 按 MIT 协议把 motor->ctrl 编码到 8 字节发送缓冲区。
 * @param motor 提供控制目标及量化映射范围的电机对象。
 * @param tx_data 长度至少为 8 字节的输出缓冲区。
 * @note 当前五文件实现未提供此兼容声明的函数体；发送控制命令请调用
 *       dm_motor_ctrl_send() 或 mit_ctrl()。
 */
void dm_motor_pack_ctrl_data(motor_t *motor, uint8_t *tx_data);

/**
 * @brief 把浮点位置映射为 MIT 协议的 16 bit 无符号整数。
 * @param pos_float 待映射的位置值。
 * @param p_max 位置上限。
 * @param p_min 位置下限。
 * @return 量化后的整数值。
 */
int read_dm_motor_pos(float pos_float, float p_max, float p_min);

/**
 * @brief 把浮点速度映射为 MIT 协议的 12 bit 无符号整数。
 * @param vel_float 待映射的速度值。
 * @param v_max 速度上限。
 * @param v_min 速度下限。
 * @return 量化后的整数值。
 */
int read_dm_motor_vel(float vel_float, float v_max, float v_min);

/**
 * @brief 把浮点扭矩映射为 MIT 协议的 12 bit 无符号整数。
 * @param tor_float 待映射的扭矩值。
 * @param t_max 扭矩上限。
 * @param t_min 扭矩下限。
 * @return 量化后的整数值。
 */
int read_dm_motor_tor(float tor_float, float t_max, float t_min);

/**
 * @brief 把 MIT 协议 16 bit 位置值还原为浮点数。
 * @param pos_int 协议位置整数。
 * @param p_max 位置上限。
 * @param p_min 位置下限。
 * @return 还原后的浮点位置。
 */
float set_dm_motor_pos(int pos_int, float p_max, float p_min);

/**
 * @brief 把 MIT 协议 12 bit 速度值还原为浮点数。
 * @param vel_int 协议速度整数。
 * @param v_max 速度上限。
 * @param v_min 速度下限。
 * @return 还原后的浮点速度。
 */
float set_dm_motor_vel(int vel_int, float v_max, float v_min);

/**
 * @brief 把 MIT 协议 12 bit 扭矩值还原为浮点数。
 * @param tor_int 协议扭矩整数。
 * @param t_max 扭矩上限。
 * @param t_min 扭矩下限。
 * @return 还原后的浮点扭矩。
 */
float set_dm_motor_tor(int tor_int, float t_max, float t_min);

#endif /* __DM_MOTOR_CTRL_H__ */
