#include "bsp_can.h"

#include <string.h>

// ==================== CAN 对象表与内部工具 ====================

static STM32CAN_t *stm32_can_map[BSP_CAN_NUMBER] = {0};

BSP_CAN_t BSP_CAN_get_id(CAN_TypeDef *addr)
{
  if (addr == NULL)
  {
    return BSP_CAN_ID_ERROR;
  }

#ifdef CAN1
  if (addr == CAN1)
  {
    return BSP_CAN1;
  }
#endif
#ifdef CAN2
  if (addr == CAN2)
  {
    return BSP_CAN2;
  }
#endif

  return BSP_CAN_ID_ERROR;
}

static bool BSP_CAN_is_valid_id(BSP_CAN_t id)
{
  return (id != BSP_CAN_ID_ERROR) && (id < BSP_CAN_NUMBER);
}

static STM32CAN_t *BSP_CAN_get_object(CAN_HandleTypeDef *can_handle)
{
  if ((can_handle == NULL) || (can_handle->Instance == NULL))
  {
    return NULL;
  }

  const BSP_CAN_t id = BSP_CAN_get_id(can_handle->Instance);
  if (!BSP_CAN_is_valid_id(id))
  {
    return NULL;
  }

  STM32CAN_t *self = stm32_can_map[id];
  if ((self == NULL) || (self->can_handle_ != can_handle))
  {
    return NULL;
  }

  return self;
}

static err_t BSP_CAN_start_handle(CAN_HandleTypeDef *can_handle)
{
  ASSERT(can_handle != NULL);
  if (can_handle == NULL)
  {
    return PTR_NULL;
  }

  if (can_handle->State != HAL_CAN_STATE_LISTENING)
  {
    const HAL_StatusTypeDef start_status = HAL_CAN_Start(can_handle);
    VERIFY(start_status == HAL_OK);
    if (start_status != HAL_OK)
    {
      return INIT_ERR;
    }
  }

  const HAL_StatusTypeDef notification_status = HAL_CAN_ActivateNotification(
      can_handle,
      CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
  VERIFY(notification_status == HAL_OK);
  return (notification_status == HAL_OK) ? OK : INIT_ERR;
}

static err_t BSP_CAN_send_with_retry(CAN_HandleTypeDef *can_handle,
                                     CAN_TxHeaderTypeDef *tx_header,
                                     const uint8_t *data)
{
  uint32_t used_mailbox = 0U;
  bool mailbox_seen = false;

  for (uint8_t attempt = 0U; attempt < 3U; ++attempt)
  {
    if (HAL_CAN_GetTxMailboxesFreeLevel(can_handle) == 0U)
    {
      continue;
    }

    mailbox_seen = true;
    if (HAL_CAN_AddTxMessage(can_handle,
                             tx_header,
                             (uint8_t *)data,
                             &used_mailbox) == HAL_OK)
    {
      return OK;
    }
  }

  return mailbox_seen ? FAILED : BUSY;
}

static err_t BSP_CAN_send_std_data(CAN_HandleTypeDef *can_handle,
                                   uint32_t std_id,
                                   const uint8_t *data,
                                   size_t size)
{
  ASSERT(can_handle != NULL);
  if (can_handle == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(data != NULL);
  if (data == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(size > 0U);
  if (size == 0U)
  {
    return SIZE_ERR;
  }

  ASSERT(size <= BSP_CAN_DATA_SIZE);
  if (size > BSP_CAN_DATA_SIZE)
  {
    return OUT_OF_RANGE;
  }

  ASSERT(std_id <= 0x7FFU);
  if (std_id > 0x7FFU)
  {
    return OUT_OF_RANGE;
  }

  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = std_id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = (uint32_t)size;
  tx_header.TransmitGlobalTime = DISABLE;

  return BSP_CAN_send_with_retry(can_handle, &tx_header, data);
}

// ==================== CAN 对象接口 ====================

err_t STM32CAN_Init(STM32CAN_t *self, CAN_HandleTypeDef *can_handle)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  self->last_error_ = PENDING;
  ASSERT(can_handle != NULL);
  if ((can_handle == NULL) || (can_handle->Instance == NULL))
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  const BSP_CAN_t id = BSP_CAN_get_id(can_handle->Instance);
  ASSERT(BSP_CAN_is_valid_id(id));
  if (!BSP_CAN_is_valid_id(id))
  {
    self->last_error_ = NOT_FOUND;
    return self->last_error_;
  }

  if ((stm32_can_map[id] != NULL) && (stm32_can_map[id] != self))
  {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  self->id_ = id;
  self->can_handle_ = can_handle;
  memset(self->subscribers_, 0, sizeof(self->subscribers_));
  self->subscriber_count_ = 0U;
  self->started_ = false;
  stm32_can_map[id] = self;
  self->last_error_ = OK;
  return self->last_error_;
}

err_t STM32CAN_SubscribeRx(STM32CAN_t *self,
                           STM32CAN_RxCallback_t callback,
                           void *context)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(callback != NULL);
  if (callback == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  if (self->started_)
  {
    self->last_error_ = STATE_ERR;
    return self->last_error_;
  }

  for (uint8_t i = 0U; i < self->subscriber_count_; ++i)
  {
    const STM32CAN_RxSubscriber_t *subscriber = &self->subscribers_[i];
    if ((subscriber->callback_ == callback) &&
        (subscriber->context_ == context))
    {
      self->last_error_ = OK;
      return self->last_error_;
    }
  }

  if (self->subscriber_count_ >= STM32CAN_RX_SUBSCRIBER_CAPACITY)
  {
    self->last_error_ = FULL;
    return self->last_error_;
  }

  STM32CAN_RxSubscriber_t *subscriber =
      &self->subscribers_[self->subscriber_count_++];
  subscriber->callback_ = callback;
  subscriber->context_ = context;
  self->last_error_ = OK;
  return self->last_error_;
}

err_t STM32CAN_Start(STM32CAN_t *self)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  self->last_error_ = BSP_CAN_start_handle(self->can_handle_);
  if (self->last_error_ == OK)
  {
    self->started_ = true;
  }
  return self->last_error_;
}

err_t STM32CAN_ConfigFilter(STM32CAN_t *self,
                            const CAN_FilterTypeDef *filter)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(self->can_handle_ != NULL);
  ASSERT(filter != NULL);
  if ((self->can_handle_ == NULL) || (filter == NULL))
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  const HAL_StatusTypeDef status =
      HAL_CAN_ConfigFilter(self->can_handle_, (CAN_FilterTypeDef *)filter);
  VERIFY(status == HAL_OK);
  self->last_error_ = (status == HAL_OK) ? OK : FAILED;
  return self->last_error_;
}

err_t STM32CAN_Send(STM32CAN_t *self,
                    uint32_t std_id,
                    const uint8_t *data,
                    size_t size)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  self->last_error_ =
      BSP_CAN_send_std_data(self->can_handle_, std_id, data, size);
  return self->last_error_;
}

err_t STM32CAN_SendByHandle(CAN_HandleTypeDef *can_handle,
                            uint32_t std_id,
                            const uint8_t *data,
                            size_t size)
{
  if (can_handle == NULL)
  {
    return PTR_NULL;
  }

  STM32CAN_t *self = BSP_CAN_get_object(can_handle);
  if (self == NULL)
  {
    return NOT_FOUND;
  }

  return STM32CAN_Send(self, std_id, data, size);
}

void STM32CAN_HandleRxFrame(STM32CAN_t *self,
                            const BSP_CAN_Frame_t *frame)
{
  if (self == NULL)
  {
    return;
  }

  ASSERT(frame != NULL);
  if (frame == NULL)
  {
    self->last_error_ = PTR_NULL;
    return;
  }

  if (frame->size_ > BSP_CAN_DATA_SIZE)
  {
    self->last_error_ = OUT_OF_RANGE;
    return;
  }

  self->last_error_ = OK;
  for (uint8_t i = 0U; i < self->subscriber_count_; ++i)
  {
    STM32CAN_RxSubscriber_t *subscriber = &self->subscribers_[i];
    subscriber->callback_(self, frame, subscriber->context_);
  }
}

err_t STM32CAN_GetLastError(const STM32CAN_t *self)
{
  return (self != NULL) ? self->last_error_ : PTR_NULL;
}

// ==================== HAL RX 分发 ====================

static void BSP_CAN_RX_ISR_Handler(CAN_HandleTypeDef *can_handle,
                                   uint32_t fifo)
{
  STM32CAN_t *self = BSP_CAN_get_object(can_handle);
  if (self == NULL)
  {
    return;
  }

  CAN_RxHeaderTypeDef rx_header = {0};
  BSP_CAN_Frame_t frame = {0};
  if (HAL_CAN_GetRxMessage(can_handle,
                           fifo,
                           &rx_header,
                           frame.data_) != HAL_OK)
  {
    self->last_error_ = FAILED;
    return;
  }

  if (rx_header.DLC > BSP_CAN_DATA_SIZE)
  {
    self->last_error_ = OUT_OF_RANGE;
    return;
  }

  frame.id_ = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId
                                            : rx_header.ExtId;
  frame.ide_ = rx_header.IDE;
  frame.rtr_ = rx_header.RTR;
  frame.fifo_ = fifo;
  frame.size_ = (uint8_t)rx_header.DLC;
  if (frame.size_ < BSP_CAN_DATA_SIZE)
  {
    memset(&frame.data_[frame.size_],
           0,
           BSP_CAN_DATA_SIZE - frame.size_);
  }

  STM32CAN_HandleRxFrame(self, &frame);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can_handle)
{
  BSP_CAN_RX_ISR_Handler(can_handle, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *can_handle)
{
  BSP_CAN_RX_ISR_Handler(can_handle, CAN_RX_FIFO1);
}
