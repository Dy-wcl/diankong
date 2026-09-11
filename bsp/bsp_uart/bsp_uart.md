# bsp_uart

RX 使用 `ReceiveToIdle + DMA_CIRCULAR`，TX 根据发送语义选择两条独立路径：

| 接口 | DMA 模式 | 用途 |
| --- | --- | --- |
| `STM32UARTDoubleBufTx_*` | 硬件 DBM，M0/M1 自动循环切换 | 定长连续数据流 |
| `STM32UARTFrameTx_*` | `DMA_NORMAL`，一帧发送、一帧等待 | 变长协议帧，每帧只发送一次 |

同一 UART 只能绑定一个 TX 对象，可以同时绑定 RX 对象。实现见 [bsp_uart.c](./bsp_uart.c)，接口见 [bsp_uart.h](./bsp_uart.h)。

## 单缓冲循环 RX

`STM32UART_Init()` 绑定 RX 缓冲和接收回调，`STM32UART_SetRxDMA()` 启动循环接收。`HAL_UARTEx_RxEventCallback()` 根据 DMA 计数器切分新增字节，回绕时最多分为两段。对象和缓冲必须长期有效，中断中将新字节交给 FIFO，在任务中解析协议。

## 硬件双缓冲连续 TX

STM32F405 DBM 的两页共用一个 `NDTR` 重载长度。传输完成时硬件立即切换 `CT`，不等待 CPU 中断。因此本接口始终按完整页连续发送，不支持“pending 为空就精确停止”或两页不同帧长。

```c
static uint8_t tx_pages[2][128];
static STM32UARTDoubleBufTx_t tx;

static void RefillTxPage(uint8_t *data, size_t size)
{
  // data 是刚发送完的页。填满 size 字节，供硬件下一轮使用。
  // 示例保持内容不变，因此会循环发送原有内容。
  RM_UNUSED(data);
  RM_UNUSED(size);
}

err_t StartStream(void)
{
  err_t err = STM32UARTDoubleBufTx_Init(&tx, &huart1,
      (BSP_UART_RawData_t){tx_pages[0], sizeof(tx_pages[0])},
      (BSP_UART_RawData_t){tx_pages[1], sizeof(tx_pages[1])}, RefillTxPage);
  if (err != OK)
  {
    return err;
  }
  err = STM32UARTDoubleBufTx_SetTxDMA(&tx);
  if (err != OK)
  {
    return err;
  }
  // 此处两页均为有效的零数据；实际应用应先填满两页。
  return STM32UARTDoubleBufTx_Flush(&tx);
}
```

- `SetTxDMA()` 配置 `DMA_CIRCULAR + DBM`、`PAR`、`M0AR`、`M1AR` 和固定 `NDTR`，尚不发送。
- `Flush()` 调用 `HAL_DMAEx_MultiBufferStart_IT()` 并开启 UART `DMAT`；两页必须都已准备好。运行中再次调用返回 `BUSY`。
- `Write()` 仅供空闲时启动：长度必须等于页容量，复制同一份数据到两页后调用 `Flush()`。运行中返回 `BUSY`，不会改写 DMA 使用中的页。
- M0/M1 完成事件均从 DMA 回调进入 `HandleTxComplete()`，读取硬件 `CT` 并把刚完成页交给填充回调。不会重启 DMA，也不会通过 `HAL_UART_TxCpltCallback()` 重复分发。
- 回调签名变为 `void (*)(uint8_t *data, size_t size)`。回调运行在 DMA ISR 中，只能写传入页；不更新时该页内容会在下一轮重复发送。
- **最大 DMA 中断延迟 + 回调填充耗时必须小于一页发送时间**。8N1 时一页时间约为 `size × 10 / baud_rate` 秒；高优先级中断和临界区耗时也要计入。超过期限可能写到硬件已重新使用的页。应先在任务中准备数据，在回调中快速复制，不能阻塞等待。
- 两页须等长、非重叠，容量为 1～65535 字节，位于 DMA 可访问的 SRAM（不能使用 F405 CCM）。不支持 9 位无校验的数据格式。
- `Stop()` 只能在任务上下文且未进入临界区时调用，只关闭 TX DMA，RX 不受影响。停止可能截断当前页，UART 中已有的字节仍会发出；需要完整单次帧时使用 `FrameTx`。
- DMA 错误关闭 TX 请求与中断，将 `last_error_` 设为 `FAILED`；任务可重新调用 `SetTxDMA()`，准备两页后再次启动。配置、启动和停止等生命周期操作由一个任务串行管理。

## 变长帧 TX 与模块迁移

`Vofa` 和 `LX824` 已迁移到 `STM32UARTFrameTx_*`，保留原来的 firewater 帧及舵机请求/应答协议：不补齐发送、不自动重复指令。

```c
static uint8_t frame_pages[2][32];
static STM32UARTFrameTx_t frame_tx;

err_t InitFrameTx(void)
{
  err_t err = STM32UARTFrameTx_Init(&frame_tx, &huart6,
      (BSP_UART_RawData_t){frame_pages[0], sizeof(frame_pages[0])},
      (BSP_UART_RawData_t){frame_pages[1], sizeof(frame_pages[1])}, NULL);
  return (err == OK) ? STM32UARTFrameTx_SetTxDMA(&frame_tx) : err;
}
```

调用 `STM32UARTFrameTx_Write(&frame_tx, data, actual_size)` 发送实际长度。返回 `OK` 表示帧已接受；一帧在发送且一帧已排队时返回 `BUSY`，新帧未入队，调用方可以稍后重试。BSP 内部保存/恢复 `PRIMASK`，模块无需再包裹 FreeRTOS 临界区。

`HAL_UART_TxCpltCallback()` 在 UART TC 后释放当前页并启动排队帧。初次提交失败不会保留未接受的新帧；已接受的排队帧若续发失败，则保留并可用 `Flush()` 重试，后续 `Write()` 也会尝试推进旧帧并对新帧返回 `BUSY`。

## 验证

主机状态测试见 [tests/uart_tx](../../tests/uart_tx/README.md)。该测试覆盖接口与状态转换；真实 DMA 时序、回调期限和串口波形需要上板验证。
