#include "bsp_uart.h"

#include <string.h>

// ==================== DMA 对象表 ====================
//! RX = 硬件循环 DMA + 软件读指针；TX 分为变长单次帧和定长连续硬件 DBM。
//! HAL 回调只负责切分数据和推进发送状态，协议解析和复杂业务应放在任务上下文。

//! 单缓冲 RX 控制块映射表。
//! 第一维按 BSP_UART_t 逻辑设备索引，HAL RX 事件通过外设 Instance 反查到对应对象。
static STM32UART_t* stm32_uart_map[BSP_UART_NUMBER] = {0};

//! 变长帧 TX 控制块映射表。
//! HAL TX 完成回调通过该表找到正在发送的 TX 对象，并触发续发。
static STM32UARTFrameTx_t* stm32_uart_frame_tx_map[BSP_UART_NUMBER] = {0};

//! 硬件 DBM 对象表，与变长帧 TX 表互斥。
static STM32UARTDoubleBufTx_t* stm32_uart_double_buf_tx_map[BSP_UART_NUMBER] = {
    0};

//! TX 状态同时被任务和 DMA ISR 使用；保存 PRIMASK，兼容嵌套临界区。
static uint32_t STM32_UART_TX_Lock(void) {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void STM32_UART_TX_Unlock(uint32_t primask) { __set_PRIMASK(primask); }

//! 两块缓冲必须等长且不重叠，长度受 DMA 16 位 NDTR 限制。
static bool STM32_UART_TX_ValidBuffers(BSP_UART_RawData_t buf0,
                                       BSP_UART_RawData_t buf1) {
  const uintptr_t addr0 = (uintptr_t)buf0.addr_;
  const uintptr_t addr1 = (uintptr_t)buf1.addr_;
  const uintptr_t distance =
      (addr0 < addr1) ? (addr1 - addr0) : (addr0 - addr1);
  return (buf0.addr_ != NULL) && (buf1.addr_ != NULL) && (buf0.size_ > 0U) &&
         (buf0.size_ <= UINT16_MAX) && (buf0.size_ == buf1.size_) &&
         (distance >= buf0.size_);
}

//! 将 HAL 外设 Instance 转换为 BSP UART 逻辑 ID。
//! 该函数集中维护 UART 外设和 BSP 枚举的映射关系，新增串口时优先在这里补映射。
BSP_UART_t BSP_UART_get_id(USART_TypeDef* addr) {
  if (addr == NULL) {
    return BSP_UART_ID_ERROR;
  }

#ifdef USART1
  if (addr == USART1) {
    return BSP_USART1;
  }
#endif
#ifdef USART2
  if (addr == USART2) {
    return BSP_USART2;
  }
#endif
#ifdef USART3
  if (addr == USART3) {
    return BSP_USART3;
  }
#endif
#ifdef USART6
  if (addr == USART6) {
    return BSP_USART6;
  }
#endif
#ifdef UART4
  if (addr == UART4) {
    return BSP_UART4;
  }
#endif
#ifdef UART5
  if (addr == UART5) {
    return BSP_UART5;
  }
#endif

  return BSP_UART_ID_ERROR;
}

//! 检查 BSP UART 逻辑 ID 是否可用。
//! BSP_UART_ID_ERROR 和越界值都视为非法，避免访问对象表越界。
static bool BSP_UART_is_valid_id(BSP_UART_t id) {
  return (id != BSP_UART_ID_ERROR) && (id < BSP_UART_NUMBER);
}

// ==================== 单缓冲 DMA RX ====================

//! 初始化单缓冲 DMA RX 控制块。
//! 该函数只做参数绑定和对象注册；DMA 接收由 STM32UART_SetRxDMA() 显式启动。
err_t STM32UART_Init(STM32UART_t* self, UART_HandleTypeDef* uart_handle,
                     BSP_UART_RawData_t dma_buff_rx,
                     STM32UART_RxCallback_t callback) {
  if (self == NULL) {
    return PTR_NULL;
  }

  self->id_ = (uart_handle != NULL) ? BSP_UART_get_id(uart_handle->Instance)
                                    : BSP_UART_ID_ERROR;
  self->last_rx_pos_ = 0U;
  self->dma_buff_rx_ = dma_buff_rx;
  self->uart_handle_ = uart_handle;
  self->rx_callback_ = callback;
  self->last_error_ = PENDING;

  ASSERT(self->uart_handle_ != NULL);
  if (self->uart_handle_ == NULL) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(BSP_UART_is_valid_id(self->id_));
  if (!BSP_UART_is_valid_id(self->id_)) {
    self->last_error_ = NOT_FOUND;
    return self->last_error_;
  }

  ASSERT(self->dma_buff_rx_.addr_ != NULL);
  if (self->dma_buff_rx_.addr_ == NULL) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(self->dma_buff_rx_.size_ > 0U);
  if (self->dma_buff_rx_.size_ == 0U) {
    self->last_error_ = SIZE_ERR;
    return self->last_error_;
  }

  ASSERT((stm32_uart_map[self->id_] == NULL) ||
         (stm32_uart_map[self->id_] == self));
  if ((stm32_uart_map[self->id_] != NULL) &&
      (stm32_uart_map[self->id_] != self)) {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  stm32_uart_map[self->id_] = self;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 启动单缓冲 DMA RX。
//! RX DMA 使用 DMA_CIRCULAR，HAL_UARTEx_ReceiveToIdle_DMA() 用 IDLE 事件驱动数据切分。
//! 调用前必须保证 dma_buff_rx_ 指向的缓冲区在整个接收期间有效。
err_t STM32UART_SetRxDMA(STM32UART_t* self) {
  if (self == NULL) {
    return PTR_NULL;
  }

  ASSERT(self->uart_handle_ != NULL);
  if (self->uart_handle_ == NULL) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  if ((self->uart_handle_->Init.Mode & UART_MODE_RX) != UART_MODE_RX) {
    self->last_error_ = NOT_SUPPORT;
    return self->last_error_;
  }

  ASSERT(self->uart_handle_->hdmarx != NULL);
  if (self->uart_handle_->hdmarx == NULL) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(self->dma_buff_rx_.addr_ != NULL);
  if (self->dma_buff_rx_.addr_ == NULL) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(self->dma_buff_rx_.size_ > 0U);
  if (self->dma_buff_rx_.size_ == 0U) {
    self->last_error_ = SIZE_ERR;
    return self->last_error_;
  }

  self->uart_handle_->hdmarx->Init.Mode = DMA_CIRCULAR;
  const HAL_StatusTypeDef dma_status = HAL_DMA_Init(self->uart_handle_->hdmarx);
  VERIFY(dma_status == HAL_OK);
  if (dma_status != HAL_OK) {
    self->last_error_ = INIT_ERR;
    return self->last_error_;
  }

  const HAL_StatusTypeDef rx_status = HAL_UARTEx_ReceiveToIdle_DMA(
      self->uart_handle_, (uint8_t*)self->dma_buff_rx_.addr_,
      (uint16_t)self->dma_buff_rx_.size_);
  VERIFY(rx_status == HAL_OK);
  if (rx_status != HAL_OK) {
    self->last_error_ = INIT_ERR;
    return self->last_error_;
  }

  self->last_error_ = OK;
  return self->last_error_;
}

//! 更新单缓冲 RX 用户回调。
//! 不触碰 DMA 状态，只替换后续 RX 数据片段的处理入口。
void STM32UART_SetRxCallback(STM32UART_t* self,
                             STM32UART_RxCallback_t callback) {
  if (self == NULL) {
    return;
  }

  self->rx_callback_ = callback;
}

//! 获取单缓冲 RX 最近一次错误码。
err_t STM32UART_GetLastError(const STM32UART_t* self) {
  if (self == NULL) {
    return PTR_NULL;
  }

  return self->last_error_;
}

//! 将一段 RX 数据交给用户回调。
//! ISR 路径已经完成环形缓冲切片；这里仅做防御检查并调用 rx_callback_。
//! 用户回调可能运行在 HAL 回调上下文中，不应执行长时间阻塞操作。
void STM32UART_HandleRxData(STM32UART_t* self, uint8_t* data, size_t size) {
  if (self == NULL) {
    return;
  }

  ASSERT(self->rx_callback_ != NULL);
  if (self->rx_callback_ == NULL) {
    self->last_error_ = PTR_NULL;
    return;
  }

  ASSERT(data != NULL);
  if (data == NULL) {
    self->last_error_ = PTR_NULL;
    return;
  }

  ASSERT(size > 0U);
  if (size == 0U) {
    self->last_error_ = SIZE_ERR;
    return;
  }

  self->last_error_ = OK;
  self->rx_callback_(data, size);
}

//! 单缓冲 RX 事件处理入口。
//! 根据 DMA 剩余计数计算当前写入位置，并把 last_rx_pos_ 到 curr_pos 的增量切片回调。
//! 环形缓冲回绕时会拆成尾部和头部两段分别回调。
static void STM32_UART_RX_ISR_Handler(UART_HandleTypeDef* uart_handle) {
  ASSERT(uart_handle != NULL);
  if (uart_handle == NULL) {
    return;
  }

  const BSP_UART_t id = BSP_UART_get_id(uart_handle->Instance);
  ASSERT(BSP_UART_is_valid_id(id));
  if (!BSP_UART_is_valid_id(id)) {
    return;
  }

  STM32UART_t* uart = stm32_uart_map[id];
  ASSERT(uart != NULL);
  if (uart == NULL) {
    return;
  }

  ASSERT(uart_handle->hdmarx != NULL);
  if (uart_handle->hdmarx == NULL) {
    uart->last_error_ = PTR_NULL;
    return;
  }

  uint8_t* rx_buf = (uint8_t*)uart->dma_buff_rx_.addr_;
  const size_t dma_size = uart->dma_buff_rx_.size_;
  ASSERT(rx_buf != NULL);
  if (rx_buf == NULL) {
    uart->last_error_ = PTR_NULL;
    return;
  }

  ASSERT(dma_size > 0U);
  if (dma_size == 0U) {
    uart->last_error_ = SIZE_ERR;
    return;
  }

  const size_t dma_remaining =
      (size_t)__HAL_DMA_GET_COUNTER(uart_handle->hdmarx);
  ASSERT(dma_remaining <= dma_size);
  if (dma_remaining > dma_size) {
    uart->last_error_ = OUT_OF_RANGE;
    return;
  }

  const size_t curr_pos = dma_size - dma_remaining;
  const size_t last_pos = uart->last_rx_pos_;
  ASSERT(last_pos <= dma_size);
  if (last_pos > dma_size) {
    uart->last_error_ = OUT_OF_RANGE;
    uart->last_rx_pos_ = curr_pos;
    return;
  }

  if (curr_pos != last_pos) {
    if (curr_pos > last_pos) {
      const size_t data_size = curr_pos - last_pos;
      STM32UART_HandleRxData(uart, rx_buf + last_pos, data_size);
    } else {
      const size_t first_part_size = dma_size - last_pos;
      STM32UART_HandleRxData(uart, rx_buf + last_pos, first_part_size);

      if (curr_pos > 0U) {
        const size_t second_part_size = curr_pos;
        STM32UART_HandleRxData(uart, rx_buf, second_part_size);
      }
    }

    uart->last_rx_pos_ = curr_pos;
  }
}

// ==================== 变长帧 DMA TX ====================

//! 初始化变长帧 DMA TX 控制块。
//! 两块软件缓冲轮流作为 DMA 源，允许 DMA 忙时提前填充下一块 pending 数据。
err_t STM32UARTFrameTx_Init(STM32UARTFrameTx_t* self,
                            UART_HandleTypeDef* uart_handle,
                            BSP_UART_RawData_t dma_buff_0,
                            BSP_UART_RawData_t dma_buff_1,
                            STM32UART_TxCompleteCallback_t callback) {
  if (self == NULL) {
    return PTR_NULL;
  }

  const BSP_UART_t id = (uart_handle != NULL)
                            ? BSP_UART_get_id(uart_handle->Instance)
                            : BSP_UART_ID_ERROR;
  if (BSP_UART_is_valid_id(id) && (stm32_uart_frame_tx_map[id] == self) &&
      (self->tx_busy_ || (self->pending_size_ != 0U))) {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  self->id_ = (uart_handle != NULL) ? BSP_UART_get_id(uart_handle->Instance)
                                    : BSP_UART_ID_ERROR;
  self->last_tx_pos_ = 0U;
  self->dma_buff_0_ = dma_buff_0;
  self->dma_buff_1_ = dma_buff_1;
  self->uart_handle_ = uart_handle;
  self->tx_callback_ = callback;
  self->last_error_ = PENDING;
  self->active_buf_ = 0U;
  self->pending_size_ = 0U;
  self->tx_busy_ = false;
  self->dma_ready_ = false;

  ASSERT(self->uart_handle_ != NULL);
  if (self->uart_handle_ == NULL) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(BSP_UART_is_valid_id(self->id_));
  if (!BSP_UART_is_valid_id(self->id_)) {
    self->last_error_ = NOT_FOUND;
    return self->last_error_;
  }

  ASSERT(self->dma_buff_0_.addr_ != NULL);
  if (self->dma_buff_0_.addr_ == NULL) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(self->dma_buff_1_.addr_ != NULL);
  if (self->dma_buff_1_.addr_ == NULL) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(STM32_UART_TX_ValidBuffers(dma_buff_0, dma_buff_1));
  if (!STM32_UART_TX_ValidBuffers(dma_buff_0, dma_buff_1)) {
    self->last_error_ = SIZE_ERR;
    return self->last_error_;
  }

  if (stm32_uart_double_buf_tx_map[self->id_] != NULL) {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  ASSERT((stm32_uart_frame_tx_map[self->id_] == NULL) ||
         (stm32_uart_frame_tx_map[self->id_] == self));
  if ((stm32_uart_frame_tx_map[self->id_] != NULL) &&
      (stm32_uart_frame_tx_map[self->id_] != self)) {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  stm32_uart_frame_tx_map[self->id_] = self;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 配置变长帧 TX 的 DMA 通道。
//! TX DMA 使用 DMA_NORMAL，每次 Flush 只发送当前 active buffer 的 pending 数据。
err_t STM32UARTFrameTx_SetTxDMA(STM32UARTFrameTx_t* self) {
  if (self == NULL) {
    return PTR_NULL;
  }

  ASSERT(self->uart_handle_ != NULL);
  if (self->uart_handle_ == NULL) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  if ((self->uart_handle_->Init.Mode & UART_MODE_TX) != UART_MODE_TX) {
    self->last_error_ = NOT_SUPPORT;
    return self->last_error_;
  }

  ASSERT(self->uart_handle_->hdmatx != NULL);
  if (self->uart_handle_->hdmatx == NULL) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  if (self->tx_busy_ || (self->pending_size_ != 0U) ||
      (self->uart_handle_->gState != HAL_UART_STATE_READY) ||
      (self->uart_handle_->hdmatx->State != HAL_DMA_STATE_READY)) {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  self->dma_ready_ = false;
  self->uart_handle_->hdmatx->Init.Mode = DMA_NORMAL;
  const HAL_StatusTypeDef dma_status = HAL_DMA_Init(self->uart_handle_->hdmatx);
  VERIFY(dma_status == HAL_OK);
  if (dma_status != HAL_OK) {
    self->last_error_ = INIT_ERR;
    return self->last_error_;
  }

  self->active_buf_ = 0U;
  self->pending_size_ = 0U;
  self->tx_busy_ = false;

  self->dma_ready_ = true;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 写入一帧 TX 数据。
//! DMA 空闲时写入 active buffer 并立即启动发送；DMA 忙时写入另一块缓冲等待完成回调续发。
//! 只保留一块 pending 缓冲；已满时返回 BUSY，不覆盖已接受的帧。
err_t STM32UARTFrameTx_Write(STM32UARTFrameTx_t* self, const uint8_t* data,
                             size_t size) {
  if (self == NULL) {
    return PTR_NULL;
  }

  ASSERT(data != NULL);
  if (data == NULL) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(size > 0U);
  if (size == 0U) {
    self->last_error_ = SIZE_ERR;
    return self->last_error_;
  }

  ASSERT(size <= self->dma_buff_0_.size_);
  if (size > self->dma_buff_0_.size_) {
    self->last_error_ = OUT_OF_RANGE;
    return self->last_error_;
  }

  const uint32_t primask = STM32_UART_TX_Lock();
  if (!self->dma_ready_ || (self->pending_size_ != 0U)) {
    const err_t err = self->dma_ready_ ? BUSY : STATE_ERR;
    if (self->dma_ready_ && !self->tx_busy_) {
      // 上次 ISR 续发失败时先重试旧帧，本次新帧仍返回 BUSY，避免重复入队。
      (void)STM32UARTFrameTx_Flush(self);
    }
    self->last_error_ = err;
    STM32_UART_TX_Unlock(primask);
    return err;
  }

  const uint8_t fill_buf =
      self->tx_busy_ ? (uint8_t)(1U - self->active_buf_) : self->active_buf_;
  void* buf_addr =
      (fill_buf == 0U) ? self->dma_buff_0_.addr_ : self->dma_buff_1_.addr_;

  memcpy(buf_addr, data, size);
  self->pending_size_ = size;

  const err_t err = self->tx_busy_ ? OK : STM32UARTFrameTx_Flush(self);
  if (err != OK) {
    // 返回失败表示本次写入未被接受，调用方可以安全重试。
    self->pending_size_ = 0U;
  }
  self->last_error_ = err;
  STM32_UART_TX_Unlock(primask);
  return err;
}

//! 提交 pending 数据到 HAL DMA 发送。
//! Flush 成功后 pending_size_ 清零，发送完成后由 HAL_UART_TxCpltCallback() 切换 active buffer。
err_t STM32UARTFrameTx_Flush(STM32UARTFrameTx_t* self) {
  if (self == NULL) {
    return PTR_NULL;
  }

  const uint32_t primask = STM32_UART_TX_Lock();
  err_t err = OK;
  if (!self->dma_ready_) {
    err = STATE_ERR;
  } else if (self->tx_busy_) {
    err = BUSY;
  } else if (self->pending_size_ == 0U) {
    err = EMPTY;
  } else {
    void* buf_addr = (self->active_buf_ == 0U) ? self->dma_buff_0_.addr_
                                               : self->dma_buff_1_.addr_;
    self->tx_busy_ = true;
    const HAL_StatusTypeDef tx_status = HAL_UART_Transmit_DMA(
        self->uart_handle_, (uint8_t*)buf_addr, (uint16_t)self->pending_size_);
    if (tx_status == HAL_OK) {
      self->pending_size_ = 0U;
    } else {
      self->tx_busy_ = false;
      err = (tx_status == HAL_BUSY) ? BUSY : FAILED;
    }
  }

  self->last_error_ = err;
  STM32_UART_TX_Unlock(primask);
  return err;
}

//! 更新变长帧 TX 完成回调。
//! 不影响正在进行的 DMA 发送，只改变后续完成事件通知对象。
void STM32UARTFrameTx_SetTxCompleteCallback(
    STM32UARTFrameTx_t* self, STM32UART_TxCompleteCallback_t callback) {
  if (self == NULL) {
    return;
  }

  self->tx_callback_ = callback;
}

//! 获取变长帧 TX 最近一次错误码。
err_t STM32UARTFrameTx_GetLastError(const STM32UARTFrameTx_t* self) {
  if (self == NULL) {
    return PTR_NULL;
  }

  return self->last_error_;
}

//! 处理 TX DMA 完成事件。
//! 释放 busy 状态、切换 active buffer；如果另一块缓冲有 pending 数据则立即续发。
void STM32UARTFrameTx_HandleTxComplete(STM32UARTFrameTx_t* self) {
  if ((self == NULL) || !self->tx_busy_) {
    return;
  }

  self->tx_busy_ = false;
  self->active_buf_ = (uint8_t)(1U - self->active_buf_);

  if (self->pending_size_ > 0U) {
    (void)STM32UARTFrameTx_Flush(self);
  }

  if (self->tx_callback_ != NULL) {
    self->tx_callback_();
  }
}

// ==================== 硬件双缓冲 DMA TX ====================

//! 从 DMA Parent 反查硬件 DBM 对象；只接管本对象的 TX Stream。
static STM32UARTDoubleBufTx_t* STM32_UART_DBM_GetObject(
    DMA_HandleTypeDef* hdma) {
  if ((hdma == NULL) || (hdma->Parent == NULL)) {
    return NULL;
  }

  UART_HandleTypeDef* huart = (UART_HandleTypeDef*)hdma->Parent;
  const BSP_UART_t id = BSP_UART_get_id(huart->Instance);
  if (!BSP_UART_is_valid_id(id) || (huart->hdmatx != hdma)) {
    return NULL;
  }
  return stm32_uart_double_buf_tx_map[id];
}

//! M0/M1 完成共用入口。HAL 已清 TC 标志，CT 此时指向下一页。
static void STM32_UART_DBM_Complete(DMA_HandleTypeDef* hdma) {
  STM32UARTDoubleBufTx_t* self = STM32_UART_DBM_GetObject(hdma);
  if ((self != NULL) && (hdma->ErrorCode == HAL_DMA_ERROR_NONE)) {
    STM32UARTDoubleBufTx_HandleTxComplete(self);
  }
}

//! 错误时先关闭请求和中断，不在 ISR 中调用依赖 HAL tick 的阻塞 Abort。
//! 任务重新调用 SetTxDMA 恢复；只改变 TX 状态，RX 继续运行。
static void STM32_UART_DBM_Error(DMA_HandleTypeDef* hdma) {
  STM32UARTDoubleBufTx_t* self = STM32_UART_DBM_GetObject(hdma);
  if (self == NULL) {
    return;
  }

  CLEAR_BIT(self->uart_handle_->Instance->CR3, USART_CR3_DMAT);
  __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
  __HAL_DMA_DISABLE_IT(hdma, DMA_IT_FE);
  __HAL_DMA_DISABLE(hdma);
  // HAL 的 DMA 错误分支只在 TE 时复位状态；FE/DME 进入本回调时 State 仍为 BUSY 且持锁。
  // 流已被上面停止，这里补齐复位，任务侧 SetTxDMA + Flush 的恢复路径才能走通。
  hdma->State = HAL_DMA_STATE_READY;
  __HAL_UNLOCK(hdma);
  self->uart_handle_->ErrorCode |= HAL_UART_ERROR_DMA;
  self->uart_handle_->TxXferCount = 0U;
  self->uart_handle_->gState = HAL_UART_STATE_READY;
  self->tx_busy_ = false;
  self->dma_ready_ = false;
  self->last_error_ = FAILED;
}

//! 只绑定对象和两块硬件页；启动前由调用方填充两页，或使用 Write 同步填充。
err_t STM32UARTDoubleBufTx_Init(STM32UARTDoubleBufTx_t* self,
                                UART_HandleTypeDef* uart_handle,
                                BSP_UART_RawData_t dma_buff_0,
                                BSP_UART_RawData_t dma_buff_1,
                                STM32UART_TxRefillCallback_t callback) {
  if (self == NULL) {
    return PTR_NULL;
  }
  if ((uart_handle == NULL) || (dma_buff_0.addr_ == NULL) ||
      (dma_buff_1.addr_ == NULL)) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  const BSP_UART_t id = BSP_UART_get_id(uart_handle->Instance);
  if (!BSP_UART_is_valid_id(id)) {
    self->last_error_ = NOT_FOUND;
    return self->last_error_;
  }
  if (!STM32_UART_TX_ValidBuffers(dma_buff_0, dma_buff_1)) {
    self->last_error_ = SIZE_ERR;
    return self->last_error_;
  }
  if ((stm32_uart_frame_tx_map[id] != NULL) ||
      ((stm32_uart_double_buf_tx_map[id] != NULL) &&
       ((stm32_uart_double_buf_tx_map[id] != self) || self->tx_busy_))) {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  self->id_ = id;
  self->dma_buff_0_ = dma_buff_0;
  self->dma_buff_1_ = dma_buff_1;
  self->uart_handle_ = uart_handle;
  self->tx_callback_ = callback;
  self->active_buf_ = 0U;
  self->tx_busy_ = false;
  self->dma_ready_ = false;
  stm32_uart_double_buf_tx_map[id] = self;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 配置双缓冲 TX 的 DMA 通道。
//! 硬件 DBM 强制循环，M0/M1 共用同一个页长；配置后保持停止，Flush 再启动。
err_t STM32UARTDoubleBufTx_SetTxDMA(STM32UARTDoubleBufTx_t* self) {
  if (self == NULL) {
    return PTR_NULL;
  }
  UART_HandleTypeDef* huart = self->uart_handle_;
  if ((huart == NULL) || (huart->hdmatx == NULL) ||
      (huart->hdmatx->Instance == NULL)) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }
  if (!BSP_UART_is_valid_id(self->id_) ||
      (stm32_uart_double_buf_tx_map[self->id_] != self)) {
    self->last_error_ = STATE_ERR;
    return self->last_error_;
  }
  if (!STM32_UART_TX_ValidBuffers(self->dma_buff_0_, self->dma_buff_1_)) {
    self->last_error_ = SIZE_ERR;
    return self->last_error_;
  }
  if (((huart->Init.Mode & UART_MODE_TX) != UART_MODE_TX) ||
      ((huart->Init.WordLength == UART_WORDLENGTH_9B) &&
       (huart->Init.Parity == UART_PARITY_NONE))) {
    self->last_error_ = NOT_SUPPORT;
    return self->last_error_;
  }

  DMA_HandleTypeDef* hdma = huart->hdmatx;
  if (self->tx_busy_ || (huart->gState != HAL_UART_STATE_READY) ||
      ((hdma->Instance->CR & DMA_SxCR_EN) != 0U)) {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  self->dma_ready_ = false;
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
  hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma->Init.MemInc = DMA_MINC_ENABLE;
  hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma->Init.Mode = DMA_CIRCULAR;
  hdma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  const HAL_StatusTypeDef dma_status = HAL_DMA_Init(hdma);
  VERIFY(dma_status == HAL_OK);
  if (dma_status != HAL_OK) {
    self->last_error_ = INIT_ERR;
    return self->last_error_;
  }

  hdma->Parent = huart;
  hdma->XferCpltCallback = STM32_UART_DBM_Complete;
  hdma->XferM1CpltCallback = STM32_UART_DBM_Complete;
  hdma->XferHalfCpltCallback = NULL;
  hdma->XferM1HalfCpltCallback = NULL;
  hdma->XferErrorCallback = STM32_UART_DBM_Error;
  hdma->XferAbortCallback = NULL;
  __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
  __HAL_DMA_DISABLE_IT(hdma, DMA_IT_FE);
  CLEAR_BIT(hdma->Instance->CR, DMA_SxCR_CT | DMA_SxCR_PFCTRL);
  SET_BIT(hdma->Instance->CR, DMA_SxCR_DBM);
  hdma->Instance->PAR = (uint32_t)(uintptr_t)&huart->Instance->DR;
  hdma->Instance->M0AR = (uint32_t)(uintptr_t)self->dma_buff_0_.addr_;
  hdma->Instance->M1AR = (uint32_t)(uintptr_t)self->dma_buff_1_.addr_;
  hdma->Instance->NDTR = (uint32_t)self->dma_buff_0_.size_;
  self->active_buf_ = 0U;
  self->dma_ready_ = true;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 空闲时用同一个完整页初始化 M0/M1；运行时只允许完成回调填充空闲页。
err_t STM32UARTDoubleBufTx_Write(STM32UARTDoubleBufTx_t* self,
                                 const uint8_t* data, size_t size) {
  if (self == NULL) {
    return PTR_NULL;
  }
  if (data == NULL) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }
  if ((size == 0U) || (size != self->dma_buff_0_.size_)) {
    self->last_error_ = SIZE_ERR;
    return self->last_error_;
  }

  const uint32_t primask = STM32_UART_TX_Lock();
  err_t err;
  if (!self->dma_ready_ || self->tx_busy_) {
    err = self->dma_ready_ ? BUSY : STATE_ERR;
  } else {
    memcpy(self->dma_buff_0_.addr_, data, size);
    memcpy(self->dma_buff_1_.addr_, data, size);
    err = STM32UARTDoubleBufTx_Flush(self);
  }
  self->last_error_ = err;
  STM32_UART_TX_Unlock(primask);
  return err;
}

//! 使用 HAL DMAEx 启动真正 DBM；不使用会清除 DBM 的 HAL_UART_Transmit_DMA。
err_t STM32UARTDoubleBufTx_Flush(STM32UARTDoubleBufTx_t* self) {
  if (self == NULL) {
    return PTR_NULL;
  }

  const uint32_t primask = STM32_UART_TX_Lock();
  err_t err = OK;
  UART_HandleTypeDef* huart = self->uart_handle_;
  if (!self->dma_ready_) {
    err = STATE_ERR;
  } else if (self->tx_busy_ || (huart->gState != HAL_UART_STATE_READY) ||
             (huart->hdmatx->State != HAL_DMA_STATE_READY) ||
             ((huart->hdmatx->Instance->CR & DMA_SxCR_EN) != 0U)) {
    err = BUSY;
  } else {
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
    CLEAR_BIT(huart->hdmatx->Instance->CR, DMA_SxCR_CT);
    __DMB();
    self->active_buf_ = 0U;
    self->tx_busy_ = true;
    const HAL_StatusTypeDef status = HAL_DMAEx_MultiBufferStart_IT(
        huart->hdmatx, (uint32_t)(uintptr_t)self->dma_buff_0_.addr_,
        (uint32_t)(uintptr_t)&huart->Instance->DR,
        (uint32_t)(uintptr_t)self->dma_buff_1_.addr_,
        (uint32_t)self->dma_buff_0_.size_);
    if (status == HAL_OK) {
      huart->pTxBuffPtr = (uint8_t*)self->dma_buff_0_.addr_;
      huart->TxXferSize = (uint16_t)self->dma_buff_0_.size_;
      huart->TxXferCount = huart->TxXferSize;
      huart->gState = HAL_UART_STATE_BUSY_TX;
      __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);
      SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);
    } else {
      err = (status == HAL_BUSY) ? BUSY : FAILED;
    }
  }
  self->last_error_ = err;
  STM32_UART_TX_Unlock(primask);
  return err;
}

//! 显式停止连续流。停止不是页边界操作，不保证最后一页完整。
err_t STM32UARTDoubleBufTx_Stop(STM32UARTDoubleBufTx_t* self) {
  if (self == NULL) {
    return PTR_NULL;
  }
  if ((__get_IPSR() != 0U) || (__get_PRIMASK() != 0U) ||
      (__get_BASEPRI() != 0U)) {
    self->last_error_ = NOT_SUPPORT;
    return self->last_error_;
  }
  if (!self->dma_ready_) {
    self->last_error_ = STATE_ERR;
    return self->last_error_;
  }

  UART_HandleTypeDef* huart = self->uart_handle_;
  const uint32_t primask = STM32_UART_TX_Lock();
  if (!self->tx_busy_) {
    self->last_error_ = OK;
    STM32_UART_TX_Unlock(primask);
    return OK;
  }
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
  __HAL_DMA_DISABLE_IT(huart->hdmatx,
                       DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
  __HAL_DMA_DISABLE_IT(huart->hdmatx, DMA_IT_FE);
  self->dma_ready_ = false;
  STM32_UART_TX_Unlock(primask);

  const HAL_StatusTypeDef status = HAL_DMA_Abort(huart->hdmatx);
  if (status != HAL_OK) {
    self->tx_busy_ = false;
    huart->gState = HAL_UART_STATE_READY;
    self->last_error_ = (status == HAL_TIMEOUT) ? TIMEOUT : FAILED;
    return self->last_error_;
  }
  huart->TxXferCount = 0U;
  huart->gState = HAL_UART_STATE_READY;
  self->tx_busy_ = false;
  self->dma_ready_ = true;
  self->last_error_ = OK;
  return self->last_error_;
}

void STM32UARTDoubleBufTx_SetTxCompleteCallback(
    STM32UARTDoubleBufTx_t* self, STM32UART_TxRefillCallback_t callback) {
  if (self != NULL) {
    const uint32_t primask = STM32_UART_TX_Lock();
    self->tx_callback_ = callback;
    STM32_UART_TX_Unlock(primask);
  }
}

err_t STM32UARTDoubleBufTx_GetLastError(const STM32UARTDoubleBufTx_t* self) {
  return (self == NULL) ? PTR_NULL : self->last_error_;
}

//! 硬件已经切页，不重启 DMA，也不手工翻转 CT；用户回调只能写刚完成的页。
void STM32UARTDoubleBufTx_HandleTxComplete(STM32UARTDoubleBufTx_t* self) {
  if ((self == NULL) || !self->tx_busy_) {
    return;
  }
  const uint32_t ct = self->uart_handle_->hdmatx->Instance->CR & DMA_SxCR_CT;
  __DMB();
  self->active_buf_ = (ct == 0U) ? 0U : 1U;
  // CT=0 表示DMA切换到了M0，说明M1刚完成
  // CT=1 表示DMA切换到了M1，说明M0刚完成
  BSP_UART_RawData_t completed =
      (ct == 0U) ? self->dma_buff_1_ : self->dma_buff_0_;
  if (self->tx_callback_ != NULL) {
    self->tx_callback_((uint8_t*)completed.addr_, completed.size_);
    __DMB();
  }
}

// ==================== HAL 回调分发 ====================

//! HAL Receive-To-Idle 事件回调。
//! HAL 只提供 UART 句柄和本轮 size；这里反查 BSP 对象后按 DMA 计数器切片。
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
  RM_UNUSED(size);

  if (huart == NULL) {
    return;
  }

  const BSP_UART_t id = BSP_UART_get_id(huart->Instance);
  if (!BSP_UART_is_valid_id(id)) {
    return;
  }

  if (stm32_uart_map[id] != NULL) {
    STM32_UART_RX_ISR_Handler(huart);
  }
}

//! HAL UART TC 完成回调，只分发变长帧 TX。
//! 反查变长帧 TX 控制块并推进发送状态；未注册对象时静默返回。
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == NULL) {
    return;
  }

  const BSP_UART_t id = BSP_UART_get_id(huart->Instance);
  if (!BSP_UART_is_valid_id(id)) {
    return;
  }

  STM32UARTFrameTx_t* tx = stm32_uart_frame_tx_map[id];
  if (tx != NULL) {
    STM32UARTFrameTx_HandleTxComplete(tx);
  }
}

//! HAL UART 错误回调，回收被错误路径终止的变长帧 TX 发送状态。
//! RX 侧错误同样会进入本回调；只有 TX DMA 已物理停止且 UART TC 中断未挂起时，
//! 才能确认本次传输是被错误路径终止的，此时复位 tx_busy_，pending 数据由 Write/Flush 重试。
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  if (huart == NULL) {
    return;
  }

  const BSP_UART_t id = BSP_UART_get_id(huart->Instance);
  if (!BSP_UART_is_valid_id(id)) {
    return;
  }

  STM32UARTFrameTx_t* tx = stm32_uart_frame_tx_map[id];
  if ((tx == NULL) || !tx->tx_busy_ || (huart->hdmatx == NULL)) {
    return;
  }

  // DMA TC 已搬完但 UART TC 未到的窗口里 EN 同样为 0，此时 TCIE 已挂起、
  // 正常完成路径仍会触发 HandleTxComplete，不能在这里回收状态。
  if (((huart->hdmatx->Instance->CR & DMA_SxCR_EN) == 0U) &&
      ((huart->Instance->CR1 & USART_CR1_TCIE) == 0U)) {
    tx->tx_busy_ = false;
  }
}
