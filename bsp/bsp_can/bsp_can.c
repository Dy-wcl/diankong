#include "bsp_can.h"

#include <string.h>

// ==================== 对象表与内部工具 ====================

/**
 * 按逻辑 ID 索引的 CAN 控制块指针表；Init 时注册，HAL 回调时反查。
 * 下标范围为 [0, BSP_CAN_NUMBER)。
 */
static STM32CAN_t *stm32_can_map[BSP_CAN_NUMBER] = {0};

/**
 * @brief 硬件外设基址 → 逻辑 CAN ID
 *
 * 根据编译期启用的 CAN 宏逐项比对。
 * @param addr 如 CAN1 / CAN2 等寄存器基址
 * @return 对应 BSP_CANx；NULL 或未匹配返回 BSP_CAN_ID_ERROR
 */
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

/**
 * @brief 判断逻辑 CAN ID 是否合法
 * @param id BSP_CAN_get_id 或 Init 得到的 ID
 * @return true：可用于索引对象表；false：ERROR 或越界
 */
static bool BSP_CAN_is_valid_id(BSP_CAN_t id)
{
  return (id != BSP_CAN_ID_ERROR) && (id < BSP_CAN_NUMBER);
}

/**
 * @brief 由 HAL 句柄反查已注册的 CAN 控制块
 *
 * 供 RX ISR / SendByHandle 使用。
 * 除 ID 合法外，还要求 map 中对象的 can_handle_ 与入参一致，
 * 避免句柄复用或错误绑定导致误分发。
 * @param can_handle HAL CAN 句柄
 * @return 已 Init 的 STM32CAN_t*；未注册或参数非法返回 NULL
 */
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

/**
 * @brief 启动 HAL CAN 并激活 RX FIFO pending 中断
 *
 * - 若尚未 LISTENING：调用 HAL_CAN_Start
 * - 无论是否已启动，均尝试激活 FIFO0 + FIFO1 的 MSG_PENDING 通知
 * @param can_handle HAL CAN 句柄
 * @return OK 成功；PTR_NULL / INIT_ERR
 */
static err_t BSP_CAN_start_handle(CAN_HandleTypeDef *can_handle)
{
  if (can_handle == NULL)
  {
    return PTR_NULL;
  }

  if (can_handle->State != HAL_CAN_STATE_LISTENING)
  {
    if (HAL_CAN_Start(can_handle) != HAL_OK)
    {
      return INIT_ERR;
    }
  }

  return (HAL_CAN_ActivateNotification(
              can_handle,
              CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) ==
          HAL_OK)
             ? OK
             : INIT_ERR;
}

/**
 * @brief 发送一帧标准数据帧（无排队）
 *
 * 校验句柄/数据/长度/标准 ID 范围后，检查邮箱空闲；
 * 无空邮箱立即 BUSY；否则组装 TxHeader 并 HAL_CAN_AddTxMessage。
 * @param can_handle HAL CAN 句柄
 * @param std_id     标准帧 ID，须 ≤ 0x7FF
 * @param data       数据指针
 * @param size       1..BSP_CAN_DATA_SIZE
 * @return OK / BUSY / PTR_NULL / SIZE_ERR / OUT_OF_RANGE / FAILED
 */
static err_t BSP_CAN_send_std_data(CAN_HandleTypeDef *can_handle,
                                   uint32_t std_id,
                                   const uint8_t *data,
                                   size_t size)
{
  if ((can_handle == NULL) || (data == NULL))
  {
    return PTR_NULL;
  }
  if (size == 0U)
  {
    return SIZE_ERR;
  }
  if ((size > BSP_CAN_DATA_SIZE) || (std_id > 0x7FFU))
  {
    return OUT_OF_RANGE;
  }
  if (HAL_CAN_GetTxMailboxesFreeLevel(can_handle) == 0U)
  {
    return BUSY;
  }

  CAN_TxHeaderTypeDef tx_header = {0};
  uint32_t used_mailbox = 0U;
  tx_header.StdId = std_id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = (uint32_t)size;
  tx_header.TransmitGlobalTime = DISABLE;

  return (HAL_CAN_AddTxMessage(can_handle,
                               &tx_header,
                               (uint8_t *)data,
                               &used_mailbox) == HAL_OK)
             ? OK
             : FAILED;
}

// ==================== 对象初始化与订阅 ====================

/**
 * @brief 初始化 CAN 控制块并注册到对象表
 *
 * 校验句柄、Instance、逻辑 ID；同一 ID 若已被其他对象占用则返回 BUSY。
 * 清空订阅表、subscriber_count_、started_；不启动外设。
 * 须随后配置过滤器（可选）、SubscribeRx、Start。
 */
err_t STM32CAN_Init(STM32CAN_t *self, CAN_HandleTypeDef *can_handle)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  self->last_error_ = PENDING;
  if ((can_handle == NULL) || (can_handle->Instance == NULL))
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  const BSP_CAN_t id = BSP_CAN_get_id(can_handle->Instance);
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

/**
 * @brief 注册 RX 订阅者（仅 Start 前允许）
 *
 * 1. 相同 callback+context 已存在 → 幂等 OK
 * 2. 已 Start → STATE_ERR
 * 3. 槽位已满 → FULL
 * 4. 否则写入下一空槽并递增 count
 */
err_t STM32CAN_SubscribeRx(STM32CAN_t *self,
                           STM32CAN_RxCallback_t callback,
                           void *context)
{
  if ((self == NULL) || (callback == NULL))
  {
    return PTR_NULL;
  }
  for (uint8_t index = 0U; index < self->subscriber_count_; ++index)
  {
    const STM32CAN_RxSubscriber_t *subscriber = &self->subscribers_[index];
    if ((subscriber->callback_ == callback) &&
        (subscriber->context_ == context))
    {
      self->last_error_ = OK;
      return self->last_error_;
    }
  }
  if (self->started_)
  {
    self->last_error_ = STATE_ERR;
    return self->last_error_;
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

// ==================== 启动、过滤器与发送 ====================

/**
 * @brief 启动 CAN 并开启 FIFO0/FIFO1 pending 中断
 *
 * 成功后标记 started_，锁定订阅表不可再增。
 */
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

/**
 * @brief 配置 HAL 过滤器（透传）
 *
 * filter 指针强制转换仅为匹配 HAL 非 const 形参；调用期间不得修改。
 */
err_t STM32CAN_ConfigFilter(STM32CAN_t *self,
                            const CAN_FilterTypeDef *filter)
{
  if ((self == NULL) || (filter == NULL))
  {
    return PTR_NULL;
  }
  if (self->can_handle_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  self->last_error_ =
      (HAL_CAN_ConfigFilter(self->can_handle_,
                            (CAN_FilterTypeDef *)filter) == HAL_OK)
          ? OK
          : FAILED;
  return self->last_error_;
}

/**
 * @brief 对象式标准帧发送
 *
 * 结果写入 last_error_ 后返回，便于 GetLastError 查询。
 */
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

/**
 * @brief HAL 句柄适配发送：反查对象后转发 STM32CAN_Send
 *
 * 未 Init / 句柄不匹配时返回 NOT_FOUND（不写任何 last_error_）。
 */
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
  return (self != NULL) ? STM32CAN_Send(self, std_id, data, size) : NOT_FOUND;
}

// ==================== RX 广播与错误查询 ====================

/**
 * @brief 向全部订阅者按注册顺序广播帧快照
 *
 * 校验 frame / size 后置 last_error_=OK，再依次调用各 callback。
 * 某一订阅者异常不影响后续调用（回调本身不应抛错/长时间阻塞）。
 */
void STM32CAN_HandleRxFrame(STM32CAN_t *self,
                            const BSP_CAN_Frame_t *frame)
{
  if (self == NULL)
  {
    return;
  }
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
  for (uint8_t index = 0U; index < self->subscriber_count_; ++index)
  {
    STM32CAN_RxSubscriber_t *subscriber = &self->subscribers_[index];
    subscriber->callback_(self, frame, subscriber->context_);
  }
}

/**
 * @brief 获取最近错误码
 * @return self->last_error_；self 为 NULL 时返回 PTR_NULL
 */
err_t STM32CAN_GetLastError(const STM32CAN_t *self)
{
  return (self != NULL) ? self->last_error_ : PTR_NULL;
}

// ==================== HAL 回调分发 ====================

/**
 * @brief RX ISR 核心：从指定 FIFO 取帧、构快照并广播
 *
 * 1. 反查已注册对象（未注册则忽略，避免误处理）
 * 2. HAL_CAN_GetRxMessage 读出 header + data
 * 3. DLC 超限则置 OUT_OF_RANGE 且不广播
 * 4. 填充 BSP_CAN_Frame_t：ID 按 IDE 选 StdId/ExtId；尾部数据清零
 * 5. 调用 STM32CAN_HandleRxFrame 通知全部订阅者
 *
 * 本路径运行于中断上下文，订阅回调须保持短且非阻塞。
 * @param can_handle 触发 pending 的 HAL 句柄
 * @param fifo       CAN_RX_FIFO0 或 CAN_RX_FIFO1
 */
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
    /* 仅保留 DLC 有效字节，尾部清零，避免上层读到脏数据。 */
    memset(&frame.data_[frame.size_],
           0,
           BSP_CAN_DATA_SIZE - frame.size_);
  }
  STM32CAN_HandleRxFrame(self, &frame);
}

/**
 * @brief HAL CAN FIFO0 消息 pending 回调
 *
 * 弱符号覆盖：转入统一 RX 分发路径，fifo = CAN_RX_FIFO0。
 * @param can_handle 触发中断的 CAN 句柄
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can_handle)
{
  BSP_CAN_RX_ISR_Handler(can_handle, CAN_RX_FIFO0);
}

/**
 * @brief HAL CAN FIFO1 消息 pending 回调
 *
 * 弱符号覆盖：转入统一 RX 分发路径，fifo = CAN_RX_FIFO1。
 * FIFO0/FIFO1 共用构帧与广播逻辑，仅来源字段不同。
 * @param can_handle 触发中断的 CAN 句柄
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *can_handle)
{
  BSP_CAN_RX_ISR_Handler(can_handle, CAN_RX_FIFO1);
}
