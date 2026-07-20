# bsp_can

`bsp_can` 将 STM32 Classical CAN 封装为对象式接口。模块只负责 HAL 句柄绑定、标准帧收发、过滤器配置和固定容量 RX 广播，不包含任何电机协议或旧式 weak callback。

## 模块职责

- 使用 `STM32CAN_t` 管理一个 HAL CAN 外设。
- 将 `CAN1/CAN2` 映射为稳定的 `BSP_CAN_t` 逻辑编号。
- 发送 1 到 8 字节的标准数据帧。
- 把 HAL RX 数据转换为 `BSP_CAN_Frame_t` 快照。
- 按注册顺序向最多四个 RX 订阅者广播同一帧。

DJI、DM 等协议打包和帧 ID 路由属于对应业务模块，不应放入 CAN BSP。

## 数据结构

| 名称 | 作用 |
| --- | --- |
| `BSP_CAN_t` | BSP 逻辑 CAN 编号 |
| `BSP_CAN_Frame_t` | RX 帧快照，包含 ID、IDE、RTR、FIFO、DLC 和数据 |
| `STM32CAN_RxCallback_t` | 带 `context` 的订阅回调类型 |
| `STM32CAN_RxSubscriber_t` | 单个回调与上下文订阅槽 |
| `STM32CAN_t` | CAN 句柄、订阅表、启动状态和错误状态控制块 |

关键限制：

- `BSP_CAN_DATA_SIZE` 固定为 8 字节。
- `STM32CAN_RX_SUBSCRIBER_CAPACITY` 固定为 4。
- 不使用堆内存，不支持运行期退订。
- `context` 由调用方持有，其生命周期必须覆盖 CAN 运行期。

## 初始化与订阅

推荐顺序为：绑定对象、配置过滤器、注册订阅者、启动 CAN。

```c
static STM32CAN_t can1;

static void app_can_rx(STM32CAN_t *self,
                       const BSP_CAN_Frame_t *frame,
                       void *context)
{
  (void)self;
  (void)context;

  if ((frame->ide_ == CAN_ID_STD) &&
      (frame->rtr_ == CAN_RTR_DATA))
  {
    // 根据 frame->id_ 和 frame->size_ 解析数据。
  }
}

err_t app_can_init(const CAN_FilterTypeDef *filter)
{
  err_t error = STM32CAN_Init(&can1, &hcan1);
  if (error != OK)
  {
    return error;
  }

  error = STM32CAN_ConfigFilter(&can1, filter);
  if (error != OK)
  {
    return error;
  }

  error = STM32CAN_SubscribeRx(&can1, app_can_rx, NULL);
  if (error != OK)
  {
    return error;
  }

  return STM32CAN_Start(&can1);
}
```

订阅规则：

- 订阅只允许在 `STM32CAN_Start()` 成功前完成。
- 启动后注册返回 `STATE_ERR`。
- 相同 `callback + context` 重复注册保持幂等并返回 `OK`。
- 第五个不同订阅者返回 `FULL`。
- 回调按注册顺序执行。

## 发送数据

新代码应持有 `STM32CAN_t *` 并使用 `STM32CAN_Send()`：

```c
const uint8_t data[] = {0x11U, 0x22U, 0x33U};
err_t error = STM32CAN_Send(&can1, 0x123U, data, sizeof(data));
```

仍只持有 HAL 句柄的边界代码可使用 `STM32CAN_SendByHandle()`：

```c
err_t error = STM32CAN_SendByHandle(&hcan1,
                                    0x123U,
                                    data,
                                    sizeof(data));
```

`STM32CAN_SendByHandle()` 不是独立发送实现。它先查找已由 `STM32CAN_Init()` 注册的对象，再复用 `STM32CAN_Send()`；句柄未注册时返回 `NOT_FOUND`。

发送约束：

- 仅支持标准数据帧，ID 范围为 `0x000..0x7FF`。
- DLC 范围为 1 到 8 字节。
- 发送不排队、不缓存旧帧，也不在本周期内重试。
- 无空邮箱立即返回 `BUSY`；HAL 发送失败返回 `FAILED`。调用方应在下一控制周期使用最新状态重新组帧。

## 接收路径

FIFO0 和 FIFO1 共用同一条分发路径：

```text
HAL_CAN_RxFifoXMsgPendingCallback
  -> HAL_CAN_GetRxMessage
  -> 构造 BSP_CAN_Frame_t
  -> STM32CAN_HandleRxFrame
  -> 按顺序调用全部订阅者
```

RX 帧只复制 DLC 指定的有效数据，剩余字节清零。HAL 读取失败或 DLC 超过 8 时不会调用订阅者，并通过 `STM32CAN_GetLastError()` 暴露错误。

订阅回调运行在 HAL 中断上下文，必须保持非阻塞，不应执行日志输出、等待、动态分配或复杂协议流程。需要任务级处理时，由订阅者自行拷贝帧并通知任务。

## 接口速查

| 函数 | 作用 |
| --- | --- |
| `BSP_CAN_get_id` | 将 HAL Instance 转换为 BSP 逻辑编号 |
| `STM32CAN_Init` | 绑定并注册 CAN 对象 |
| `STM32CAN_SubscribeRx` | 在启动前添加 RX 订阅者 |
| `STM32CAN_Start` | 启动外设并开启 FIFO0/FIFO1 通知 |
| `STM32CAN_ConfigFilter` | 应用 HAL 过滤器配置 |
| `STM32CAN_Send` | 通过对象发送标准数据帧 |
| `STM32CAN_SendByHandle` | 通过已注册 HAL 句柄适配发送 |
| `STM32CAN_HandleRxFrame` | 向全部订阅者广播帧快照 |
| `STM32CAN_GetLastError` | 获取最近一次 BSP 错误 |

## 已移除接口

以下接口不再提供：

- `STM32CAN_SendDjiCurrent`
- `STM32CAN_SetRxCallback`
- `CAN_Init`
- `CAN_Filter_Mask_Config_16bit`
- `CAN_Filter_Mask_Config_32bit`
- `CAN_Send_Data_X8`
- `canx_receive`
- `dm_can_send_data`
- `dm_can1_rx_callback` / `dm_can2_rx_callback`
- `dj_motor_can1_rx_callback` / `dj_motor_can2_rx_callback`

DJI 命令发送统一由 `modules/motor/dj_motor` 的
`dj_motor_set_command()` 与 `dj_motor_send_group()` 完成。
