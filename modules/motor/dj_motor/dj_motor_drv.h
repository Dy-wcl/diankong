/**
 * @file dj_motor_drv.h
 * @brief DJI 电机 CAN 协议编解码、路由与发送适配接口。
 *
 * 本层为无全局状态的纯协议工具集，供 dj_motor_ctrl 调用：
 * - 型号 + 设备 ID → 反馈 ID / 控制组 / 槽位；
 * - 命令限幅与 4 路 int16 大端打包；
 * - 8 字节反馈帧解码与物理量换算；
 * - 通过 STM32CAN_Send() 发送标准数据帧。
 *
 * @note 本层不管理 group_mask / pending_mask，也不订阅 RX。
 *       拼包状态机与对象生命周期全部由 dj_motor_ctrl 负责。
 */
#ifndef DJ_MOTOR_DRV_H
#define DJ_MOTOR_DRV_H

#include "dj_motor_def.h"

/**
 * @brief 由型号与设备 ID 推导反馈 ID、控制组与帧内槽位。
 * @param type 电机型号。
 * @param device_id 设备 ID；M3508/M2006 合法范围为 1..8，GM6020 为 1..7。
 * @param route 输出路由；不可为 NULL。
 * @return OK 表示推导成功；device_id 或型号非法返回 ARG_ERR；route 为 NULL
 *         返回 PTR_NULL。
 *
 * 推导规则见 dj_motor_def.h 文件头中的地址表。GM6020 使用 0x204+ID 作为反馈 ID，
 * 控制组为 0x1FF 或 0x2FF（本仓库不使用 0x1FE/0x2FE）。
 */
err_t dj_motor_drv_derive_route(dj_motor_type_e type, uint8_t device_id,
                                dj_motor_route_t *route);

/**
 * @brief 查询型号对应的指令绝对值限幅。
 * @param type 电机型号。
 * @return 限幅绝对值；未知型号返回 0。
 *
 * M3508 为 ±16384，M2006 为 ±10000，GM6020 为 ±30000。
 */
int16_t dj_motor_drv_command_limit(dj_motor_type_e type);

/**
 * @brief 将命令钳位到型号允许的对称区间。
 * @param type 电机型号。
 * @param command 待限幅的逻辑或物理命令。
 * @return 限幅后的 int16 值；未知型号返回 0。
 */
int16_t dj_motor_drv_clamp_command(dj_motor_type_e type, int16_t command);

/**
 * @brief 将 4 路 int16 命令打包为 8 字节大端载荷。
 * @param commands 槽 0..3 的命令数组；不可为 NULL。
 * @param payload 输出 8 字节缓冲；不可为 NULL。
 *
 * 槽 N 写入 payload[2N]（高字节）与 payload[2N+1]（低字节）。
 * 空槽应传入 0。指针非法时静默返回。
 */
void dj_motor_drv_pack_commands(const int16_t commands[4], uint8_t payload[8]);

/**
 * @brief 解析 8 字节反馈数据区到原始整数结构。
 * @param data 长度至少为 8 的反馈数据区。
 * @param feedback 输出结构；不可为 NULL。
 * @return OK 或 PTR_NULL。
 *
 * 帧布局：encoder 大端 u16、rpm 大端 i16、current 大端 i16、temp u8。
 * 本函数不做方向反转与浮点换算。
 */
err_t dj_motor_drv_decode_feedback(const uint8_t data[8],
                                   dj_motor_raw_feedback_t *feedback);

/**
 * @brief 将编码器原始值换算为弧度角。
 * @param encoder 编码器值 0..8191。
 * @return 位置角，范围 [0, 2π)。
 */
float dj_motor_drv_encoder_to_rad(uint16_t encoder);

/**
 * @brief 将转子 RPM 换算为输出轴角速度。
 * @param type 用于选择减速比的电机型号。
 * @param rpm 转子侧转速（逻辑方向下可已取反）。
 * @return 输出轴角速度，单位 rad/s；减速比为 0 时返回 0。
 */
float dj_motor_drv_rpm_to_rad_s(dj_motor_type_e type, int16_t rpm);

/**
 * @brief 打包并发送指定控制组的 8 字节标准数据帧。
 * @param can 已初始化且可发送的 BSP CAN 设备。
 * @param group 控制组 ID（0x200 / 0x1FF / 0x2FF）。
 * @param commands 四槽命令；未使用槽填 0。
 * @return STM32CAN_Send() 的返回值；参数非法时返回 PTR_NULL 或 ARG_ERR。
 *
 * 邮箱忙时返回 BUSY，不排队、不重试。调用方应在下一控制周期用最新命令重发。
 */
err_t dj_motor_drv_send_group(STM32CAN_t *can, dj_motor_group_e group,
                              const int16_t commands[4]);

/**
 * @brief 将控制组枚举映射为 bus->groups[] 下标。
 * @param group 控制组 ID。
 * @return 0 对应 0x200，1 对应 0x1FF，2 对应 0x2FF；非法组返回 0xFF。
 */
uint8_t dj_motor_drv_group_index(dj_motor_group_e group);

/**
 * @brief 判断控制组 ID 是否为本模块支持的合法值。
 * @param group 待检查的控制组。
 * @return 合法返回 true，否则 false。
 */
bool dj_motor_drv_group_is_valid(dj_motor_group_e group);

#endif /* DJ_MOTOR_DRV_H */
