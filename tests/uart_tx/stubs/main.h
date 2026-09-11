#pragma once

#include <stddef.h>
#include <stdint.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t CR1, CR3, SR, DR; } USART_TypeDef;
typedef struct { uint32_t CR, FCR, PAR, M0AR, M1AR, NDTR; } DMA_Stream_TypeDef;
typedef struct
{
  uint32_t Direction, PeriphInc, MemInc, PeriphDataAlignment;
  uint32_t MemDataAlignment, Mode, FIFOMode;
} DMA_InitTypeDef;
typedef struct DMA_HandleTypeDef
{
  DMA_Stream_TypeDef *Instance;
  DMA_InitTypeDef Init;
  void *Parent;
  uint32_t State, ErrorCode;
  void (*XferCpltCallback)(struct DMA_HandleTypeDef *);
  void (*XferM1CpltCallback)(struct DMA_HandleTypeDef *);
  void (*XferHalfCpltCallback)(struct DMA_HandleTypeDef *);
  void (*XferM1HalfCpltCallback)(struct DMA_HandleTypeDef *);
  void (*XferErrorCallback)(struct DMA_HandleTypeDef *);
  void (*XferAbortCallback)(struct DMA_HandleTypeDef *);
} DMA_HandleTypeDef;
typedef struct
{
  USART_TypeDef *Instance;
  struct { uint32_t Mode, WordLength, Parity; } Init;
  DMA_HandleTypeDef *hdmatx, *hdmarx;
  const uint8_t *pTxBuffPtr;
  uint16_t TxXferSize, TxXferCount;
  uint32_t gState, ErrorCode;
} UART_HandleTypeDef;

extern USART_TypeDef test_usart1, test_usart2;
#define USART1 (&test_usart1)
#define USART2 (&test_usart2)
#define UART_MODE_TX 1U
#define UART_MODE_RX 2U
#define UART_WORDLENGTH_9B 9U
#define UART_PARITY_NONE 0U
#define UART_FLAG_TC (1U << 6)
#define USART_CR3_DMAT (1U << 7)
#define USART_CR3_DMAR (1U << 6)
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_TX 1U
#define HAL_UART_ERROR_DMA 1U
#define HAL_DMA_STATE_READY 0U
#define HAL_DMA_STATE_BUSY 1U
#define HAL_DMA_ERROR_NONE 0U
#define DMA_SxCR_EN (1U << 0)
#define DMA_IT_DME (1U << 1)
#define DMA_IT_TE (1U << 2)
#define DMA_IT_HT (1U << 3)
#define DMA_IT_TC (1U << 4)
#define DMA_SxCR_PFCTRL (1U << 5)
#define DMA_SxCR_CIRC (1U << 8)
#define DMA_SxCR_DBM (1U << 18)
#define DMA_SxCR_CT (1U << 19)
#define DMA_IT_FE (1U << 7)
#define DMA_NORMAL 0U
#define DMA_CIRCULAR DMA_SxCR_CIRC
#define DMA_MEMORY_TO_PERIPH 1U
#define DMA_PINC_DISABLE 0U
#define DMA_MINC_ENABLE 1U
#define DMA_PDATAALIGN_BYTE 0U
#define DMA_MDATAALIGN_BYTE 0U
#define DMA_FIFOMODE_DISABLE 0U
#define CLEAR_BIT(reg, bits) ((reg) &= ~(bits))
#define SET_BIT(reg, bits) ((reg) |= (bits))
#define __HAL_DMA_DISABLE(h) CLEAR_BIT((h)->Instance->CR, DMA_SxCR_EN)
// Deliberately mirror HAL: FE cannot be combined with the CR interrupt mask.
#define __HAL_DMA_DISABLE_IT(h, bits) (((bits) != DMA_IT_FE) ? \
  ((h)->Instance->CR &= ~(bits)) : ((h)->Instance->FCR &= ~(bits)))
#define __HAL_DMA_GET_COUNTER(h) ((h)->Instance->NDTR)
#define __HAL_UART_CLEAR_FLAG(h, bits) CLEAR_BIT((h)->Instance->SR, bits)
extern uint32_t test_primask, test_ipsr, test_basepri;
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask = 1U; }
static inline void __set_PRIMASK(uint32_t v) { test_primask = v; }
static inline uint32_t __get_IPSR(void) { return test_ipsr; }
static inline uint32_t __get_BASEPRI(void) { return test_basepri; }
static inline void __DMB(void) {}

HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *h);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *h);
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *h,
    uint32_t src, uint32_t dst, uint32_t second, uint32_t size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, const uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *data, uint16_t size);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h);
