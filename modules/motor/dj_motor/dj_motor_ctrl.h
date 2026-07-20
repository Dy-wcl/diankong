/**
 * @file dj_motor_ctrl.h
 * @brief DJI 电机对象管理、写齐再发控制与反馈查询接口。
 *
 * 本文件是应用层唯一需要包含的 DJ 公共头。典型调用顺序：
 *
 * 1. STM32CAN_Init + 过滤器配置；
 * 2. dj_motor_bus_init(bus, can)（在 Start 前，占用 1 个 RX 订阅槽）；
 * 3. 多次 dj_motor_init(motor, bus, type, id, reversed)；
 * 4. STM32CAN_Start；
 * 5. 周期：dj_motor_set_command →（写齐后自动发送）；
 * 6. 停机：dj_motor_zero_and_flush(bus, group)。
 *
 * ## 发送语义摘要
 *
 * - set_command：限幅、可选反向，写入组缓存并置 pending 位；
 *   当 pending_mask == group_mask 时自动打包发送；
 * - force_flush_group：不要求写齐，按当前缓存立即发送；
 * - zero_and_flush：清零已注册槽命令并立即发送全零帧（安全停机推荐路径）。
 *
 * 发送失败时 pending 保留，返回 BSP 错误码；调用方可在下周期重写命令或
 * force_flush。ISR 只更新原始反馈整数，不做浮点与发送。
 */
#ifndef DJ_MOTOR_CTRL_H
#define DJ_MOTOR_CTRL_H

#include "dj_motor_def.h"
#include "dj_motor_drv.h"

/**
 * @brief 初始化总线对象并订阅 CAN 接收回调。
 * @param bus 调用方静态分配的总线对象。
 * @param can 已 Init 但通常尚未 Start 的 BSP CAN 设备。
 * @return OK 表示成功；bus 或 can 为 NULL 时返回 PTR_NULL；订阅失败时透传
 *         STM32CAN_SubscribeRx 错误码（如 STATE_ERR / FULL）。
 *
 * 成功后 bus->initialized 为 true，三组控制组 ID 已写入。订阅失败会清零 bus，
 * 避免半初始化。须在 STM32CAN_Start() 之前调用。
 */
err_t dj_motor_bus_init(dj_motor_bus_t *bus, STM32CAN_t *can);

/**
 * @brief 初始化电机实例并注册到指定总线。
 * @param motor 调用方静态分配的电机对象。
 * @param bus 已成功 bus_init 的总线。
 * @param type 电机型号。
 * @param device_id 设备 ID（见地址表）。
 * @param reversed true 时命令与反馈速度/电流/角度取反。
 * @return OK；空指针 PTR_NULL；总线未初始化/已 Start/重复 init 返回 STATE_ERR；
 *         容量满返回 FULL；路由非法或同 bus 冲突返回 ARG_ERR。
 *
 * 失败时不修改总线注册表与控制组成员，保证无半注册。初始化后禁止注销或改 ID。
 */
err_t dj_motor_init(dj_motor_t *motor, dj_motor_bus_t *bus,
                    dj_motor_type_e type, uint8_t device_id, bool reversed);

/**
 * @brief 写入逻辑命令；组内成员全部写过本轮命令时自动发送。
 * @param motor 已初始化的电机实例。
 * @param command 逻辑方向协议原始命令（先限幅再可选反向写入物理缓存）。
 * @return OK 表示已写入（可能同时完成发送）；发送失败时返回 BSP 错误码且
 *         pending 保留；未初始化 STATE_ERR；未挂到有效组 NOT_FOUND。
 *
 * @note 半写不发：若组内有 4 个成员而本周期只写了 3 个，不会触发 CAN 发送。
 *       安全路径请使用 dj_motor_zero_and_flush()，不要依赖写齐。
 */
err_t dj_motor_set_command(dj_motor_t *motor, int16_t command);

/**
 * @brief 用当前组缓存立即发送控制帧（不要求 pending 写齐）。
 * @param bus 已初始化的总线。
 * @param group 目标控制组。
 * @return OK 或 BSP 错误；组内无注册电机返回 NOT_FOUND。
 *
 * 发送成功后清零 pending_mask。未写过命令的已注册槽保持缓存中的旧值；
 * 从未写入过的槽在注册时已置 0。
 */
err_t dj_motor_force_flush_group(dj_motor_bus_t *bus, dj_motor_group_e group);

/**
 * @brief 将组内已注册电机命令清零并立即发送全零帧。
 * @param bus 已初始化的总线。
 * @param group 目标控制组。
 * @return OK 或 BSP 错误；空组返回 NOT_FOUND。
 *
 * @note 底盘停机、遥控离线、电机超时、ACTUATION 关闭等路径应调用本接口，
 *       作为“必须发出的安全帧”的一等 API，而不是只 set_command(0)。
 */
err_t dj_motor_zero_and_flush(dj_motor_bus_t *bus, dj_motor_group_e group);

/**
 * @brief 读取电机反馈的一致快照（含任务侧物理量换算）。
 * @param motor 已初始化的电机。
 * @param feedback 输出快照；不可为 NULL。
 * @return OK / PTR_NULL / STATE_ERR。
 *
 * 短临界区内只拷贝原始整数与 tick；编码器→弧度、RPM→rad/s 以及 reverse 处理
 * 在临界区外完成，避免在关中断期间做浮点运算。
 */
err_t dj_motor_get_feedback(const dj_motor_t *motor,
                            dj_motor_feedback_t *feedback);

/**
 * @brief 判断电机是否在线。
 * @param motor 电机实例。
 * @param now_tick 当前系统 tick（通常 HAL_GetTick()）。
 * @param timeout_ticks 允许的最大无反馈间隔；底盘使用 20ms。
 * @return 已初始化且至少收过一帧反馈，且 (now - last) 无符号差不超过超时
 *         时返回 true。
 *
 * 使用无符号减法，天然兼容 tick 回绕。
 */
bool dj_motor_is_online(const dj_motor_t *motor, uint32_t now_tick,
                        uint32_t timeout_ticks);

#endif /* DJ_MOTOR_CTRL_H */
