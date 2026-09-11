# UART TX host regression

Run from the repository root (GCC, including MinGW):

```powershell
New-Item -ItemType Directory -Path build/uart_tx -Force | Out-Null
gcc -std=c11 -Wall -Wextra -Werror -Itests/uart_tx/stubs -Ibsp/bsp_uart -Icomponent bsp/bsp_uart/bsp_uart.c tests/uart_tx/test_uart_tx.c -o build/uart_tx/test_uart_tx.exe
./build/uart_tx/test_uart_tx.exe
```

The test compiles the production BSP against a small HAL/CMSIS stub. It checks DBM register configuration, fixed-size validation, M0/M1 refill ownership, no restart or UART callback duplication, explicit stop/restart, TX-only error recovery, nested interrupt-mask restoration, finite frame lengths/order, queue backpressure, and startup failure/retry.

This is a state-machine regression, not a DMA timing simulator. Board testing must check continuous M0/M1 output and the refill deadline under maximum ISR load. VOFA should retain its actual float count and frame tail; LX824 requests should appear exactly once with their original length/checksum.
