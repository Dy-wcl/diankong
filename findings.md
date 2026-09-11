# Findings

- Project uses STM32F4 HAL and CMake; UART BSP is in bsp/bsp_uart.
- Existing user changes are confined to three .settings files.
- Only Vofa and LX824 use STM32UARTDoubleBufTx; both submit variable-length, finite frames.
- STM32F405 DBM uses one NDTR reload value for M0/M1 and automatically continues on the other bank before the completion ISR runs. Stopping in that ISR cannot guarantee no extra bytes.
- HAL_DMAEx_MultiBufferStart_IT requires M0/M1 complete and error callbacks; HAL_UART_Transmit_DMA clears DBM via its normal DMA setup and cannot implement DBM streaming.
- Current Write overwrites pending frames; modules externally wrap Write in FreeRTOS critical sections.
- ARM GCC, host GCC, CMake and Ninja are available. Existing Debug build directory is present.
