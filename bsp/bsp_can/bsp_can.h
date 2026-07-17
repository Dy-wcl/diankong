#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"
#include "comp_cmd.h"
#include "comp_utils.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BSP_CAN_DATA_SIZE (8U)
#define STM32CAN_RX_SUBSCRIBER_CAPACITY (4U)

//! BSP CAN 逻辑设备编号。
typedef enum
{
#ifdef CAN1
  BSP_CAN1,
#endif
#ifdef CAN2
  BSP_CAN2,
#endif

  BSP_CAN_NUMBER,
  BSP_CAN_ID_ERROR
} BSP_CAN_t;

//! CAN 接收帧快照。
typedef struct
{
  uint32_t id_;                          //! 标准帧或扩展帧 ID
  uint32_t ide_;                         //! HAL IDE 标志
  uint32_t rtr_;                         //! HAL RTR 标志
  uint32_t fifo_;                        //! 接收 FIFO
  uint8_t size_;                         //! 有效数据长度
  uint8_t data_[BSP_CAN_DATA_SIZE];      //! 数据快照，DLC 之后补零
} BSP_CAN_Frame_t;

typedef struct STM32CAN STM32CAN_t;

//! CAN 接收订阅回调，在 HAL 中断上下文执行。
typedef void (*STM32CAN_RxCallback_t)(STM32CAN_t *self,
                                      const BSP_CAN_Frame_t *frame,
                                      void *context);

//! 一个固定容量的接收订阅槽。
typedef struct
{
  STM32CAN_RxCallback_t callback_;
  void *context_;
} STM32CAN_RxSubscriber_t;

//! CAN BSP 控制块。
struct STM32CAN
{
  BSP_CAN_t id_;
  CAN_HandleTypeDef *can_handle_;
  STM32CAN_RxSubscriber_t subscribers_[STM32CAN_RX_SUBSCRIBER_CAPACITY];
  uint8_t subscriber_count_;
  bool started_;
  err_t last_error_;
};

#ifdef __cplusplus
extern "C" {
#endif

//! 将 HAL CAN Instance 映射到 BSP 逻辑 ID。
BSP_CAN_t BSP_CAN_get_id(CAN_TypeDef *addr);

//! 绑定 HAL 句柄并注册 CAN 控制块，不启动外设。
err_t STM32CAN_Init(STM32CAN_t *self, CAN_HandleTypeDef *can_handle);

//! 在启动前注册 RX 订阅者；相同 callback/context 重复注册视为成功。
err_t STM32CAN_SubscribeRx(STM32CAN_t *self,
                           STM32CAN_RxCallback_t callback,
                           void *context);

//! 启动 CAN 并开启 FIFO0/FIFO1 消息挂起通知。
err_t STM32CAN_Start(STM32CAN_t *self);

//! 将 HAL 过滤器配置应用到当前 CAN。
err_t STM32CAN_ConfigFilter(STM32CAN_t *self,
                            const CAN_FilterTypeDef *filter);

//! 发送一帧 1 到 8 字节的标准数据帧。
err_t STM32CAN_Send(STM32CAN_t *self,
                    uint32_t std_id,
                    const uint8_t *data,
                    size_t size);

//! 通过已注册 HAL 句柄查找对象并复用 STM32CAN_Send。
err_t STM32CAN_SendByHandle(CAN_HandleTypeDef *can_handle,
                            uint32_t std_id,
                            const uint8_t *data,
                            size_t size);

//! 按注册顺序向全部订阅者广播一帧数据。
void STM32CAN_HandleRxFrame(STM32CAN_t *self,
                            const BSP_CAN_Frame_t *frame);

//! 获取最近一次 BSP 操作结果。
err_t STM32CAN_GetLastError(const STM32CAN_t *self);

#ifdef __cplusplus
}
#endif

#endif // BSP_CAN_H
