/**
 * @file control_common.h
 * @brief 控制 Facade 共用状态枚举。
 */
#ifndef CONTROL_COMMON_H
#define CONTROL_COMMON_H

/**
 * @brief 固定拓扑控制模块的生命周期状态。
 */
typedef enum {
  CONTROL_STATE_UNINITIALIZED = 0, /**< 尚未完成依赖绑定 */
  CONTROL_STATE_SAFE_DISABLED,     /**< 仅允许零输出 */
  CONTROL_STATE_READY,             /**< 输入与反馈健康，等待主动输出宏/使能 */
  CONTROL_STATE_ACTIVE,            /**< 闭环主动输出 */
  CONTROL_STATE_FAULT              /**< 故障，保持零输出 */
} control_state_e;

#endif /* CONTROL_COMMON_H */
