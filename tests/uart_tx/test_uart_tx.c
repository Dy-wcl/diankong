#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "bsp_uart.h"

USART_TypeDef test_usart1, test_usart2;
uint32_t test_primask, test_ipsr, test_basepri;
static HAL_StatusTypeDef start_result = HAL_OK;
static unsigned dbm_starts, frame_starts, completions;
static uint8_t *refill_page;
static size_t refill_size;
static uint8_t frame_log[6][8];
static uint16_t frame_lengths[6];

HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *h)
{
  h->Instance->CR = h->Init.Mode;
  h->State = HAL_DMA_STATE_READY;
  h->ErrorCode = HAL_DMA_ERROR_NONE;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *h)
{
  assert(test_primask == 0U && test_ipsr == 0U && test_basepri == 0U);
  CLEAR_BIT(h->Instance->CR, DMA_SxCR_EN);
  h->State = HAL_DMA_STATE_READY;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *h,
    uint32_t src, uint32_t dst, uint32_t second, uint32_t size)
{
  assert(h->XferCpltCallback && h->XferM1CpltCallback && h->XferErrorCallback);
  if (start_result != HAL_OK) return start_result;
  assert(h->State == HAL_DMA_STATE_READY);
  h->Instance->M0AR = src;
  h->Instance->M1AR = second;
  h->Instance->PAR = dst;
  h->Instance->NDTR = size;
  SET_BIT(h->Instance->CR, DMA_SxCR_DBM | DMA_SxCR_EN | DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
  SET_BIT(h->Instance->FCR, DMA_IT_FE);
  h->State = HAL_DMA_STATE_BUSY;
  ++dbm_starts;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, const uint8_t *data, uint16_t size)
{
  assert(h->hdmatx->Init.Mode == DMA_NORMAL);
  if (start_result != HAL_OK) return start_result;
  assert(h->gState == HAL_UART_STATE_READY && frame_starts < 6U && size <= 8U);
  memcpy(frame_log[frame_starts], data, size);
  frame_lengths[frame_starts++] = size;
  h->gState = HAL_UART_STATE_BUSY_TX;
  h->hdmatx->State = HAL_DMA_STATE_BUSY;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *data, uint16_t size)
{
  (void)h; (void)data; (void)size;
  return HAL_OK;
}

static void Refill(uint8_t *data, size_t size)
{
  refill_page = data;
  refill_size = size;
  memset(data, 0xA5, size);
  ++completions;
}

static void FinishFrame(UART_HandleTypeDef *h)
{
  h->gState = HAL_UART_STATE_READY;
  h->hdmatx->State = HAL_DMA_STATE_READY;
  test_ipsr = 1U;
  HAL_UART_TxCpltCallback(h);
  test_ipsr = 0U;
}

int main(void)
{
  uint8_t banks[2][8] = {{0}}, frames[2][8] = {{0}};
  const uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  DMA_Stream_TypeDef stream = {0}, frame_stream = {0};
  DMA_HandleTypeDef dma = {.Instance = &stream}, frame_dma = {.Instance = &frame_stream};
  UART_HandleTypeDef uart = {.Instance = USART1, .Init = {.Mode = UART_MODE_TX | UART_MODE_RX}, .hdmatx = &dma};
  UART_HandleTypeDef frame_uart = {.Instance = USART2, .Init = {.Mode = UART_MODE_TX}, .hdmatx = &frame_dma};
  STM32UARTDoubleBufTx_t tx = {0}, invalid = {0};
  STM32UARTFrameTx_t frame_tx = {0}, conflict = {0};
  BSP_UART_RawData_t b0 = {banks[0], 8}, b1 = {banks[1], 8};

  assert(STM32UARTDoubleBufTx_SetTxDMA(NULL) == PTR_NULL);
  assert(STM32UARTDoubleBufTx_Init(&invalid, &uart, b0, b0, NULL) == SIZE_ERR);
  assert(STM32UARTDoubleBufTx_Init(&invalid, &uart, b0, (BSP_UART_RawData_t){banks[1], 7}, NULL) == SIZE_ERR);
  assert(STM32UARTDoubleBufTx_Init(&invalid, &uart, (BSP_UART_RawData_t){banks[0], 65536},
      (BSP_UART_RawData_t){banks[1], 65536}, NULL) == SIZE_ERR);
  assert(STM32UARTDoubleBufTx_Init(&tx, &uart, b0, b1, Refill) == OK);
  assert(STM32UARTDoubleBufTx_Flush(&tx) == STATE_ERR);
  assert(STM32UARTFrameTx_Init(&conflict, &uart, b0, b1, NULL) == BUSY);
  uart.Init.WordLength = UART_WORDLENGTH_9B;
  assert(STM32UARTDoubleBufTx_SetTxDMA(&tx) == NOT_SUPPORT);
  uart.Init.WordLength = 8U;
  assert(STM32UARTDoubleBufTx_SetTxDMA(&tx) == OK);
  assert((stream.CR & (DMA_SxCR_DBM | DMA_SxCR_CIRC)) == (DMA_SxCR_DBM | DMA_SxCR_CIRC));
  assert(!(stream.CR & DMA_SxCR_EN) && !(uart.Instance->CR3 & USART_CR3_DMAT));
  assert(stream.M0AR == (uint32_t)(uintptr_t)banks[0] && stream.M1AR == (uint32_t)(uintptr_t)banks[1]);
  assert(stream.NDTR == 8 && !dma.XferHalfCpltCallback && !dma.XferM1HalfCpltCallback);
  assert(STM32UARTDoubleBufTx_Write(&tx, payload, 7) == SIZE_ERR);
  start_result = HAL_ERROR;
  assert(STM32UARTDoubleBufTx_Write(&tx, payload, 8) == FAILED);
  assert(!tx.tx_busy_ && uart.gState == HAL_UART_STATE_READY && !(uart.Instance->CR3 & USART_CR3_DMAT));
  start_result = HAL_OK;
  test_primask = 1U;
  assert(STM32UARTDoubleBufTx_Flush(&tx) == OK && test_primask == 1U);
  test_primask = 0U;
  assert(dbm_starts == 1 && tx.tx_busy_ && (stream.CR & DMA_SxCR_EN));
  assert(memcmp(banks[0], payload, 8) == 0 && memcmp(banks[1], payload, 8) == 0);
  assert(STM32UARTDoubleBufTx_Write(&tx, payload, 8) == BUSY);
  assert(STM32UARTDoubleBufTx_SetTxDMA(&tx) == BUSY);

  // Model automatic CT switch *before* HAL dispatches each completion.
  SET_BIT(stream.CR, DMA_SxCR_CT);
  dma.XferCpltCallback(&dma);
  assert(tx.active_buf_ == 1 && refill_page == banks[0] && refill_size == 8);
  assert(memcmp(banks[1], payload, 8) == 0);
  CLEAR_BIT(stream.CR, DMA_SxCR_CT);
  dma.XferM1CpltCallback(&dma);
  assert(tx.active_buf_ == 0 && refill_page == banks[1] && completions == 2);
  assert(dbm_starts == 1 && tx.tx_busy_ && stream.NDTR == 8);
  HAL_UART_TxCpltCallback(&uart);
  assert(completions == 2 && dbm_starts == 1); // UART TC cannot dispatch DBM twice.
  test_ipsr = 1U;
  assert(STM32UARTDoubleBufTx_Stop(&tx) == NOT_SUPPORT);
  test_ipsr = 0U;
  SET_BIT(uart.Instance->CR3, USART_CR3_DMAR);
  assert(STM32UARTDoubleBufTx_Stop(&tx) == OK);
  assert(!tx.tx_busy_ && !(stream.CR & DMA_SxCR_EN) && !(stream.FCR & DMA_IT_FE));
  assert((uart.Instance->CR3 & USART_CR3_DMAR) && uart.gState == HAL_UART_STATE_READY);
  assert(STM32UARTDoubleBufTx_Flush(&tx) == OK && dbm_starts == 2);
  dma.ErrorCode = 1U;
  dma.XferErrorCallback(&dma);
  assert(!tx.tx_busy_ && !tx.dma_ready_ && tx.last_error_ == FAILED);
  assert(!(stream.FCR & DMA_IT_FE) && (uart.Instance->CR3 & USART_CR3_DMAR));
  assert(STM32UARTDoubleBufTx_SetTxDMA(&tx) == OK);
  assert(STM32UARTDoubleBufTx_Write(&tx, payload, 8) == OK);
  assert(STM32UARTDoubleBufTx_Stop(&tx) == OK);

  assert(STM32UARTFrameTx_Init(&frame_tx, &frame_uart,
      (BSP_UART_RawData_t){frames[0], 8}, (BSP_UART_RawData_t){frames[1], 8}, NULL) == OK);
  assert(STM32UARTFrameTx_Write(&frame_tx, payload, 3) == STATE_ERR);
  assert(STM32UARTFrameTx_SetTxDMA(&frame_tx) == OK);
  assert(STM32UARTFrameTx_Write(&frame_tx, payload, 3) == OK);
  assert(STM32UARTFrameTx_Write(&frame_tx, payload + 3, 5) == OK);
  assert(STM32UARTFrameTx_Write(&frame_tx, payload, 1) == BUSY);
  assert(STM32UARTFrameTx_SetTxDMA(&frame_tx) == BUSY);
  FinishFrame(&frame_uart);
  assert(frame_starts == 2 && frame_lengths[0] == 3 && frame_lengths[1] == 5);
  assert(memcmp(frame_log[0], payload, 3) == 0 && memcmp(frame_log[1], payload + 3, 5) == 0);
  FinishFrame(&frame_uart);
  assert(!frame_tx.tx_busy_ && frame_tx.pending_size_ == 0 && frame_starts == 2);
  assert(STM32UARTFrameTx_Flush(&frame_tx) == EMPTY);
  start_result = HAL_BUSY;
  assert(STM32UARTFrameTx_Write(&frame_tx, payload, 1) == BUSY);
  assert(frame_tx.pending_size_ == 0 && !frame_tx.tx_busy_);
  start_result = HAL_OK;
  assert(STM32UARTFrameTx_Write(&frame_tx, payload, 1) == OK);
  FinishFrame(&frame_uart);
  assert(frame_starts == 3 && frame_lengths[2] == 1 && test_primask == 0U);
  assert(STM32UARTFrameTx_Write(&frame_tx, payload, 2) == OK);
  assert(STM32UARTFrameTx_Write(&frame_tx, payload + 2, 4) == OK);
  start_result = HAL_ERROR;
  FinishFrame(&frame_uart);
  assert(!frame_tx.tx_busy_ && frame_tx.pending_size_ == 4);
  start_result = HAL_OK;
  // Retry the accepted queued frame, reject the new frame without silently queuing it.
  assert(STM32UARTFrameTx_Write(&frame_tx, payload, 1) == BUSY);
  assert(frame_starts == 5 && frame_lengths[4] == 4 && frame_tx.pending_size_ == 0);
  assert(memcmp(frame_log[4], payload + 2, 4) == 0);
  FinishFrame(&frame_uart);
  assert(!frame_tx.tx_busy_ && test_primask == 0U);
  puts("UART TX tests passed: DBM lifecycle/refill/errors and finite frame ordering/backpressure.");
  return 0;
}
