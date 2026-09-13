#pragma once

/**
 * @file comp_def.h
 * @brief 组件通用定义：信号位、常量等
 */

#ifdef __cplusplus
extern "C" {
#endif

// ==================== FreeRTOS 任务通知信号位定义 ====================
// ISR 收到完整帧后通知任务解析的信号位，每个模块使用不同的 bit 位
// 注意：不同的 SIGNAL 不能有相同的 bit 位，否则会导致任务通知混乱

#define SIGNAL_I6X_RAW_READY \
  (1u << 5)  //! I6X 模块：ISR 收到完整帧后通知任务解析的信号位
#define SIGNAL_VT13_RAW_REDY \
  (1u << 6)  //! VT13 模块：ISR 收到完整帧后通知任务解析的信号位
#define SIGNAL_DR16_RAW_REDY \
  (1u << 7)  //! DR16 模块：ISR 收到完整帧后通知任务解析的信号位
#define SIGNAL_LX824_RX_READY \
  (1u << 8)  //! LX824 模块：ISR 收到字节后通知任务处理的信号位
#define SIGNAL_IMU485_RAW_READY \
  (1u << 9)  //! IMU485 模块：ISR 收到完整帧后通知任务解析的信号位
#define VOFA_SIGNAL_RAW_READY \
  (1u << 10)  //! VOFA 模块：ISR 收到字节后通知任务处理的信号位
#define SIGNAL_IMUCAN_RAW_READY \
  (1u << 11)  //! IMU CAN 模块：ISR 收到数据帧后通知任务解析

#ifdef __cplusplus
}
#endif
