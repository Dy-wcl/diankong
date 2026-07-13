# Diankong（电控）代码使用手册 —— 完整函数参考

> **项目概述**：基于 STM32F405 + FreeRTOS 的 RoboMaster 机器人电控固件  
> **适用 MCU**：STM32F405RG  
> **RTOS**：FreeRTOS (CMSIS-RTOS v2)  
> **协议年份**：RoboMaster 2026

---

## 目录

1. [项目结构总览](#1-项目结构总览)
2. [硬件资源分配](#2-硬件资源分配)
3. [Component 层 —— comp_cmd / comp_utils](#3-component-层)
4. [BSP 层 — CAN 驱动 (bsp_can)](#4-bsp-can-驱动)
5. [BSP 层 — UART DMA 驱动 (bsp_uart)](#5-bsp-uart-dma-驱动)
6. [BSP 层 — PWM 驱动 (bsp_pwm)](#6-bsp-pwm-驱动)
7. [BSP 层 — I²C 驱动 (bsp_iic)](#7-bsp-iic-驱动)
8. [BSP 层 — USB CDC 驱动 (bsp_usb)](#8-bsp-usb-cdc-驱动)
9. [BSP 层 — DWT 精密定时 (bsp_dwt)](#9-bsp-dwt-精密定时)
10. [Modules 层 — DR16 遥控器 (dr16)](#10-dr16-大疆遥控器sbus)
11. [Modules 层 — I6X 遥控器 (i6x)](#11-i6x-富斯-ibus-遥控器)
12. [Modules 层 — LX824 舵机 (lx824)](#12-lx824-总线串口舵机)
13. [Modules 层 — VOFA+ 上位机 (vofa)](#13-vofa-上位机调试协议)
14. [Modules 层 — 裁判系统 (game)](#14-game-robomaster-裁判系统)
15. [Arithmetic 层 — 增量式 PID](#15-增量式-pid-控制器)
16. [Arithmetic 层 — 位置式 PID](#16-位置式-pid-控制器)
17. [Arithmetic 层 — CRC 校验](#17-裁判系统-crc-校验)
18. [Task 层 — FreeRTOS 任务](#18-task-freertos-任务)
19. [构建系统](#19-构建系统)
20. [快速上手指南](#20-快速上手指南)
21. [常见问题与调试](#21-常见问题与调试)

---

## 1. 项目结构总览

```
diankong/
├── Core/                    # STM32CubeMX 生成的 HAL 层
│   ├── Inc/                 #   头文件
│   └── Src/                 #   源文件
├── component/               # 通用组件（错误码、数学工具、命令结构体）
├── bsp/                     # 板级支持包
│   ├── bsp_can/             #   CAN 总线驱动
│   ├── bsp_iic/             #   I²C 驱动（当前已注释禁用）
│   ├── bsp_pwm/             #   PWM 驱动
│   ├── bsp_uart/            #   UART DMA 驱动
│   ├── bsp_usb/             #   USB CDC 驱动（当前已注释禁用）
│   └── dwt/                 #   DWT 精密定时
├── modules/                 # 协议与设备驱动
│   ├── DR16/                #   DR16 SBUS 遥控器
│   ├── I6X/                 #   I6X iBus 遥控器
│   ├── LX824/               #   LX-824 总线舵机
│   ├── Vofa/                #   VOFA+ 调试协议
│   └── game/                #   RoboMaster 裁判系统
├── arithmetic/              # 算法
│   ├── pid/pid_incremental/ #   增量式 PID
│   ├── pid/pid_location/    #   位置式 PID
│   └── referee/             #   CRC 校验
├── task/                    # FreeRTOS 任务实现
│   ├── dr16_task/
│   ├── i6x_task/
│   ├── lx824_task/
│   ├── vofa_task/
│   └── game_task/
├── CMakeLists.txt
├── CMakePresets.json
└── jie_max.ioc
```

---

## 2. 硬件资源分配

| 外设 | 用途 | GPIO | 通信参数 |
|------|------|------|----------|
| **USART2** | DR16 SBUS 遥控器 | PD6 (RX) | 100kbps, 8E2, 需电平转换 |
| **UART4** | I6X iBus 遥控器 | - | 115200, 8N1, 3.3V 直连 |
| **USART3** | 裁判系统 / 图传链路 | - | 裁判系统协议 |
| **USART1** | LX824 总线舵机 | - | 半双工 |
| **USART6** | VOFA+ 上位机 | - | firewater 协议 |
| **CAN1** | 电机控制总线 (DJI) | PA11(RX), PA12(TX) | 500kbps |
| **CAN2** | 电机控制总线 (达妙) | PB5(RX), PB6(TX) | 500kbps |
| **PA7** | LED 指示灯 | PA7 | GPIO 推挽输出 |

### 时钟配置

| 时钟域 | 频率 |
|--------|------|
| SYSCLK (HSE→PLL) | 144 MHz |
| APB1 (/4) | 36 MHz |
| APB2 (/2) | 72 MHz |

---

## 3. Component 层

### 3.1 统一错误码枚举 `err_t`

**文件**：`component/comp_cmd.h`

```c
typedef enum {
    PENDING     = 1,    // 等待中 / Pending
    OK          = 0,    // 操作成功
    FAILED      = -1,   // 操作失败
    INIT_ERR    = -2,   // 初始化错误
    ARG_ERR     = -3,   // 参数错误
    STATE_ERR   = -4,   // 状态错误
    SIZE_ERR    = -5,   // 长度错误
    CHECK_ERR   = -6,   // 校验错误
    NOT_SUPPORT = -7,   // 不支持
    NOT_FOUND   = -8,   // 未找到
    NO_RESPONSE = -9,   // 无响应
    NO_MEM      = -10,  // 内存不足
    NO_BUFF     = -11,  // 缓冲区不足
    TIMEOUT     = -12,  // 超时
    EMPTY       = -13,  // 为空
    FULL        = -14,  // 已满
    BUSY        = -15,  // 忙碌
    PTR_NULL    = -16,  // 空指针
    OUT_OF_RANGE = -17  // 超出范围
} err_t;
```

**使用说明**：项目内所有模块的函数统一使用 `err_t` 作为返回值类型。判断成功应使用 `== OK`，判断失败应使用 `!= OK`。

---

### 3.2 通用数据类型

**文件**：`component/comp_cmd.h`

#### `vector2_t` / `vector3_t`

```c
typedef struct { float x; float y; } vector2_t;
typedef struct { float x; float y; float z; } vector3_t;
```

#### `cmd_switch_pos_t`

```c
typedef enum {
    CMD_SW_ERR  = 0,  // 错误/未识别
    CMD_SW_UP   = 1,  // 上拨
    CMD_SW_DOWN = 2,  // 下拨
    CMD_SW_MID  = 3,  // 中位
} cmd_switch_pos_t;
```

#### `cmd_key_t`

```c
typedef enum {
    CMD_KEY_W, CMD_KEY_S, CMD_KEY_A, CMD_KEY_D,  // WASD
    CMD_KEY_SHIFT, CMD_KEY_CTRL,                  // 修饰键
    CMD_KEY_Q, CMD_KEY_E, CMD_KEY_R, CMD_KEY_F, CMD_KEY_G,  // 技能键
    CMD_KEY_Z, CMD_KEY_X, CMD_KEY_C, CMD_KEY_V, CMD_KEY_B,  // 功能键
    CMD_KEY_L_CLICK, CMD_KEY_R_CLICK,              // 鼠标左右键
    CMD_KEY_NUM,
} cmd_key_t;
```

#### `cmd_rc_t`

```c
typedef struct {
    struct {
        vector2_t l;  // 左摇杆 (x, y)，范围约 -1~1
        vector2_t r;  // 右摇杆 (x, y)，范围约 -1~1
    } ch;

    float ch_res;           // 保留通道值，归一化约 -1~1
    cmd_switch_pos_t sw_l;  // 左拨杆位置
    cmd_switch_pos_t sw_r;  // 右拨杆位置
    cmd_mouse_t mouse;      // 鼠标数据
    uint16_t key;           // 键盘位图（位索引见 cmd_key_t）
    uint16_t res;           // 保留值
} cmd_rc_t;
```

---

### 3.3 数学工具函数

**文件**：`component/comp_utils.h` / `comp_utils.c`

#### `MAX(a, b)` / `MIN(a, b)` — 类型安全极值

```c
#define MAX(a, b) ({ __typeof__(a) _a = (a); __typeof__(b) _b = (b); _a > _b ? _a : _b; })
#define MIN(a, b) ({ __typeof__(a) _a = (a); __typeof__(b) _b = (b); _a < _b ? _a : _b; })
```

#### `abs_clampf(x, limit)` — 对称限幅

| 参数 | 类型 | 说明 | 取值范围 |
|------|------|------|---------|
| `x` | `float` | 输入值 | 任意实数 |
| `limit` | `float` | 限幅绝对值 | ≥ 0 |

| 返回值 | 条件 |
|--------|------|
| `x` | `-limit ≤ x ≤ limit` |
| `limit` | `x > limit` |
| `-limit` | `x < -limit` |

**示例**：`abs_clampf(150.0f, 100.0f)` → `100.0f`

#### `clampf(origin, lo, hi)` — 区间限幅

| 参数 | 类型 | 说明 | 取值范围 |
|------|------|------|---------|
| `origin` | `float *` | 被限幅的值（指针，就地修改） | 非 NULL |
| `lo` | `float` | 下限 | 必须 < `hi` |
| `hi` | `float` | 上限 | 必须 > `lo` |

**断言**：`origin != NULL`，`hi > lo`

#### `signf(x)` — 符号函数

| 参数 | 类型 | 返回值 |
|------|------|--------|
| `x > 0` | `float` | `1.0f` |
| `x == 0` | `float` | `0.0f` |
| `x < 0` | `float` | 注意：当前实现返回 `0.0f`（bug），实际应返回 `-1.0f` |

> ⚠️ 当前 `signf(x)` 在 `x < 0` 时返回 `0.0f` 而非 `-1.0f`，这是已知 bug。如需正确的符号函数请自行实现 `return (x > 0) ? 1.0f : (x < 0) ? -1.0f : 0.0f;`

#### `inv_sqrtf(x)` — 平方根倒数

```c
float inv_sqrtf(float x);
```

| 参数 | 说明 | 取值范围 |
|------|------|---------|
| `x` | 输入 | > 0 |

**返回值**：`1.0f / sqrtf(x)`

**内部**：提供了 Fast Inverse Square Root 算法（0x5f3759df），当前被注释禁用，调用标准库 `1.0f / sqrtf(x)`。

#### `circle_error(sp, fb, range)` — 循环值误差

用于编码器等周期循环值（如 0~2π），自动选择最短路径误差。

| 参数 | 类型 | 说明 | 取值范围 |
|------|------|------|---------|
| `sp` | `float` | 目标值 | 任意 |
| `fb` | `float` | 反馈值 | 任意 |
| `range` | `float` | 循环范围 | `> 0` 时生效，`≤ 0` 时返回 `sp - fb` |

**返回值**：循环最短误差，范围 `(-range/2, range/2]`

**示例**：

```c
// 目标 350°，当前 10°，范围 360° → 误差应为 -20°（顺时针更近）
float err = circle_error(350.0f, 10.0f, 360.0f);  // → -20.0f

// 目标 10°，当前 350°，范围 360° → 误差应为 +20°
float err = circle_error(10.0f, 350.0f, 360.0f);   // → 20.0f
```

#### `circle_add(origin, delta, range)` — 循环值加法

| 参数 | 类型 | 说明 | 取值范围 |
|------|------|------|---------|
| `origin` | `float *` | 被加数（就地修改） | 建议在 `[0, range)` |
| `delta` | `float` | 增量（可为负） | 任意 |
| `range` | `float` | 循环范围 | `> 0` 时生效 |

**示例**：

```c
float angle = 350.0f;
circle_add(&angle, 20.0f, 360.0f);  // angle → 10.0f
circle_add(&angle, -15.0f, 360.0f); // angle → 355.0f
```

#### `circle_reverse(origin)` — 循环值取反

```c
void circle_reverse(float *origin);
```

在 `0~2π` 范围内取反：`*origin = -(*origin) + M_2PI`

#### `ASSERT(expr)` — Debug 断言

```c
ASSERT(expr);  // MCU_DEBUG_BUILD 时 expr 为 false 则进入 verify_failed() 死循环
ASSERT(expr);  // Release 时展开为 ((void)(0))，expr 不执行
```

- **Debug 构建**（`-DCMAKE_BUILD_TYPE=Debug`）：`MCU_DEBUG_BUILD` 被定义，断言生效
- **Release 构建**：断言被完全移除，`expr` 不会被执行

#### `VERIFY(expr)` — 始终执行的断言

```c
VERIFY(expr);  // Debug 时如果 expr 为 false 则进入 verify_failed() 死循环
VERIFY(expr);  // Release 时展开为 ((void)(expr))，expr 仍执行但结果被忽略
```

- **Debug 构建**：与 ASSERT 相同
- **Release 构建**：`expr` 仍被执行，但结果被丢弃

#### `CONTAINER_OF(ptr, type, member)` — 从成员反算容器

```c
#define CONTAINER_OF(ptr, type, member) \
    ({ const typeof(((type *)0)->member) *__mptr = (ptr); \
       (type *)((char *)__mptr - offsetof(type, member)); })
```

**示例**：

```c
struct Motor { uint32_t id; float speed; };
struct Motor m = { .id = 1, .speed = 100.0f };
float *sp = &m.speed;
struct Motor *pm = CONTAINER_OF(sp, struct Motor, speed);  // pm == &m
```

#### `ARRAY_LEN(array)` — 数组长度

```c
#define ARRAY_LEN(array) (sizeof((array)) / sizeof(*(array)))
```

> ⚠️ 仅适用于真正的数组，传入指针会得到错误结果。

---

### 3.4 辅助宏

#### `RM_UNUSED(X)` — 抑制未使用参数警告

```c
#define RM_UNUSED(X) ((void)X)
```

用于函数参数表中未使用的形参：

```c
void my_task(void *argument) {
    RM_UNUSED(argument);
    // ...
}
```

---

## 4. BSP CAN 驱动

**文件**：`bsp/bsp_can/bsp_can.h` / `bsp_can.c`

### 核心数据结构

#### `STM32CAN_t` — CAN 对象控制块

```c
typedef struct STM32CAN {
    BSP_CAN_t id_;                       // BSP 逻辑设备编号（BSP_CAN1 / BSP_CAN2）
    CAN_HandleTypeDef *can_handle_;      // HAL CAN 句柄（&hcan1 / &hcan2）
    STM32CAN_RxCallback_t rx_callback_;  // 对象式 RX 分发回调
    err_t last_error_;                   // 最近一次操作结果
} STM32CAN_t;
```


#### `BSP_CAN_Frame_t` — 接收帧结构

```c
typedef struct {
    uint32_t id_;                 // 标准帧或扩展帧 ID
    uint32_t ide_;                // IDE 标志（CAN_ID_STD / CAN_ID_EXT）
    uint32_t rtr_;                // RTR 标志（CAN_RTR_DATA / CAN_RTR_REMOTE）
    uint32_t fifo_;               // 接收 FIFO 编号（0 或 1）
    uint8_t size_;                // 数据长度（最大 8）
    uint8_t data_[CAN_DATA_SIZE]; // 数据快照（8 字节）
} BSP_CAN_Frame_t;
```

#### `BSP_CAN_t` — 逻辑设备编号

```c
typedef enum {
#ifdef CAN1
    BSP_CAN1,        // CAN1 逻辑设备
#endif
#ifdef CAN2
    BSP_CAN2,        // CAN2 逻辑设备
#endif
    BSP_CAN_NUMBER,  // 设备总数
    BSP_CAN_ID_ERROR // 无效 ID
} BSP_CAN_t;
```

#### `STM32CAN_RxCallback_t` — 接收回调类型

```c
typedef void (*STM32CAN_RxCallback_t)(STM32CAN_t *self, const BSP_CAN_Frame_t *frame);
```

---

### 对象式 API

---

#### `STM32CAN_Init` — 初始化 CAN 控制块（绑定 HAL 句柄）

```c
err_t STM32CAN_Init(STM32CAN_t *self,
                    CAN_HandleTypeDef *can_handle,
                    STM32CAN_RxCallback_t callback);
```

- 第一个参数 `*self` —— `STM32CAN_t` 结构体指针，该结构体包含 BSP 逻辑设备编号 (`id_`)、HAL CAN 句柄 (`can_handle_`)、接收回调函数指针 (`rx_callback_`) 和最近一次错误码 (`last_error_`)，Init 时绑定到指定的 HAL CAN 句柄并注册到全局对象表
- 第二个参数 `can_handle` —— HAL CAN 句柄，指定使用哪个 CAN 外设
- 第三个参数 `callback` —— 接收回调函数指针，收到 CAN 帧时自动调用，可 NULL（此时回落到 weak 回调）

| 返回值 | 含义 |
|--------|------|
| `OK` | 初始化成功 |
| `NOT_FOUND` | 无法将 `can_handle` 映射到 BSP 逻辑设备 |
| `BUSY` | 该 CAN 外设已被另一个 `STM32CAN_t` 占用 |

---

#### `STM32CAN_Start` — 启动 CAN（开启通信和 RX 中断）

```c
err_t STM32CAN_Start(STM32CAN_t *self);
```

- 第一个参数 `*self` —— `STM32CAN_t` 结构体指针，该结构体包含 HAL CAN 句柄 (`can_handle_`)，Start 时通过该句柄调用 `HAL_CAN_Start` 开启通信并激活 RX FIFO0/FIFO1 消息挂起中断

| 返回值 | 含义 |
|--------|------|
| `OK` | 启动成功 |
| `INIT_ERR` | `HAL_CAN_Start` 或 `ActivateNotification` 失败 |

**说明**：可重复调用（检测到 `HAL_CAN_STATE_LISTENING` 时跳过 Start）

---

#### `STM32CAN_ConfigFilter` — 配置接收滤波器

```c
err_t STM32CAN_ConfigFilter(STM32CAN_t *self,
                            const CAN_FilterTypeDef *filter);
```

- 第一个参数 `*self` —— `STM32CAN_t` 结构体指针，该结构体包含 HAL CAN 句柄 (`can_handle_`)，ConfigFilter 时通过该句柄调用 `HAL_CAN_ConfigFilter` 设置滤波器
- 第二个参数 `*filter` —— HAL 滤波器配置结构体，指定滤波器的 ID、掩码、FIFO 分配和缩放模式

**使用示例**：

```c
CAN_FilterTypeDef filter = {
    .FilterIdHigh = 0x0000,
    .FilterIdLow = 0x0000,
    .FilterMaskIdHigh = 0x0000,
    .FilterMaskIdLow = 0x0000,
    .FilterFIFOAssignment = CAN_RX_FIFO0,
    .FilterBank = 0,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_32BIT,
    .FilterActivation = ENABLE,
    .SlaveStartFilterBank = 14,
};
STM32CAN_ConfigFilter(&can1, &filter);
```

---

#### `STM32CAN_Send` — 发送标准数据帧

```c
err_t STM32CAN_Send(STM32CAN_t *self,
                    uint32_t std_id,
                    const uint8_t *data,
                    size_t size);
```

- 第一个参数 `*self` —— `STM32CAN_t` 结构体指针，该结构体包含 HAL CAN 句柄 (`can_handle_`)，Send 时通过该句柄将数据帧发送到 CAN 总线
- 第二个参数 `std_id` —— 标准帧 ID，范围 `0x000 ~ 0x7FF`（11 位 CAN ID）
- 第三个参数 `*data` —— 待发送的数据缓冲区首地址
- 第四个参数 `size` —— 数据长度（字节），范围 `1 ~ 8`

| 返回值 | 含义 |
|--------|------|
| `OK` | 发送成功 |
| `OUT_OF_RANGE` | `std_id > 0x7FF` 或 `size > 8` |
| `SIZE_ERR` | `size == 0` |
| `BUSY` | 连续 3 次尝试均无空闲邮箱 |
| `FAILED` | 有邮箱但 `HAL_CAN_AddTxMessage` 失败 |

**说明**：内部最多重试 3 次寻找空闲邮箱，使用标准帧 + 数据帧类型发送

---

#### `STM32CAN_SendDjiCurrent` — 发送 DJI 电机电流控制帧

```c
err_t STM32CAN_SendDjiCurrent(STM32CAN_t *self,
                              uint32_t ctrl_id,
                              const int16_t current[4]);
```

- 第一个参数 `*self` —— `STM32CAN_t` 结构体指针，该结构体包含 HAL CAN 句柄 (`can_handle_`)，SendDjiCurrent 时通过该句柄将 4 路电机电流打包为 8 字节 CAN 数据帧发出
- 第二个参数 `ctrl_id` —— 控制帧 CAN ID，范围 `0x000 ~ 0x7FF`，DJI 电机通常使用 `0x200`~`0x2FF`
- 第三个参数 `current` —— 4 路 int16 电流值数组，每路范围 `-16384 ~ 16384`（具体取决于电机型号）

**数据打包格式**（高字节在前）：
```
Byte[0] = current[0] >> 8    Byte[1] = current[0] & 0xFF
Byte[2] = current[1] >> 8    Byte[3] = current[1] & 0xFF
Byte[4] = current[2] >> 8    Byte[5] = current[2] & 0xFF
Byte[6] = current[3] >> 8    Byte[7] = current[3] & 0xFF
```

**示例**：

```c
// 通过 ID 0x200 控制 4 个 DJI 电机
int16_t currents[4] = {1000, -2000, 500, 0};
STM32CAN_SendDjiCurrent(&can_motor, 0x200, currents);
```

---

#### `STM32CAN_SetRxCallback` — 更新接收回调

```c
void STM32CAN_SetRxCallback(STM32CAN_t *self,
                            STM32CAN_RxCallback_t callback);
```

- 第一个参数 `*self` —— `STM32CAN_t` 结构体指针，该结构体包含接收回调函数指针 (`rx_callback_`)，调用此函数替换该指针
- 第二个参数 `callback` —— 新回调函数指针，收到 CAN 帧时自动调用，可传入 NULL 清除回调

**说明**：运行时动态变更接收处理函数，不重启 CAN。

---

#### `STM32CAN_GetLastError` — 读取最近错误码

```c
err_t STM32CAN_GetLastError(const STM32CAN_t *self);
```

- 第一个参数 `*self` —— `STM32CAN_t` 结构体指针，该结构体包含最近一次操作错误码 (`last_error_`)，GetLastError 时读取该字段

| 返回值 | 含义 |
|--------|------|
| `OK` / 其他 | 最近一次 API 调用设置的错误码 |

---

### 兼容旧接口

以下接口保留以兼容历史代码，新业务建议使用对象式 API。

---

#### `CAN_Init` — 启动 CAN（旧接口）

```c
void CAN_Init(CAN_HandleTypeDef *hcan);
```

| 参数 | 说明 |
|------|------|
| `hcan` | HAL CAN 句柄，如 `&hcan1` |

启动 CAN 并开启 FIFO 中断，与 `STM32CAN_Start` 功能相同。

---

#### `CAN_Filter_Mask_Config_16bit` — 16 位掩码滤波器

```c
void CAN_Filter_Mask_Config_16bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para,
                                  uint16_t ID1, uint16_t Mask1,
                                  uint16_t ID2, uint16_t Mask2);
```

| 参数 | 说明 | 可用值 |
|------|------|--------|
| `Object_Para` | 用 `CAN_FILTER()` 宏打包参数 | FilterBank、FIFO、ID 类型、帧类型 |
| `ID1` / `Mask1` | 第一组 16 位 ID + 掩码 | 0x0000 ~ 0xFFFF |
| `ID2` / `Mask2` | 第二组 16 位 ID + 掩码 | 0x0000 ~ 0xFFFF |

**`Object_Para` 打包宏**：

```c
CAN_FILTER(x)        // 滤波器编号 = x 左移 3 位
CAN_FIFO_0           // 使用 FIFO0
CAN_FIFO_1           // 使用 FIFO1
CAN_STDID            // 标准帧 ID
CAN_EXTID            // 扩展帧 ID
CAN_DATA_TYPE        // 数据帧
CAN_REMOTE_TYPE      // 远程帧

// 组合示例：滤波器 0, FIFO0, 标准帧, 数据帧
uint8_t para = CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE;
```

**示例**：

```c
// 滤波器 5, FIFO0, 标准帧, 数据帧
CAN_Filter_Mask_Config_16bit(&hcan1,
    CAN_FILTER(5) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE,
    0x200, 0x7FF,  // ID1=0x200, Mask1=0x7FF（精确匹配 0x200）
    0x1FF, 0x7FF); // ID2=0x1FF, Mask2=0x7FF（精确匹配 0x1FF）
```

---

#### `CAN_Filter_Mask_Config_32bit` — 32 位掩码滤波器

```c
void CAN_Filter_Mask_Config_32bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para,
                                  uint32_t ID, uint32_t Mask_ID);
```

| 参数 | 说明 |
|------|------|
| `ID` | 32 位扩展帧 ID |
| `Mask_ID` | 对应掩码 |

**说明**：32 位模式下，ID 格式取决于 `Object_Para` 中的 `CAN_STDID`/`CAN_EXTID`：
- 标准帧：`(id << 5) | CAN_ID_STD | rtr`
- 扩展帧：`(id << 3) | CAN_ID_EXT | rtr`

---

#### `dj_CAN_Send_Data` — DJI 电机电流控制（旧接口）

```c
uint8_t dj_CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID,
                         int16_t cm1_iq, int16_t cm2_iq,
                         int16_t cm3_iq, int16_t cm4_iq,
                         uint16_t Length);
```

| 参数 | 说明 |
|------|------|
| `hcan` | HAL CAN 句柄 |
| `ID` | 控制帧 ID（0x000~0x7FF） |
| `cm1_iq` ~ `cm4_iq` | 4 路电流值 |
| `Length` | 数据长度（通常为 8） |

| 返回值 | 含义 |
|--------|------|
| `1` | 发送成功 |
| `0` | 发送失败 |

---

#### `CAN_Send_Data_X8` — 发送 8 字节全零帧

```c
void CAN_Send_Data_X8(CAN_HandleTypeDef *hcan, uint16_t ID);
```

发送一帧 8 字节全零的标准帧，可用于电机急停。

---

#### `canx_receive` — 从 FIFO 读取一帧

```c
uint8_t canx_receive(hcan_t *hcan, uint16_t *rec_id, uint8_t *buf, uint32_t fifo);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `hcan` | `hcan_t *` | HAL CAN 句柄 |
| `rec_id` | `uint16_t *` | 输出：接收到的 CAN ID |
| `buf` | `uint8_t *` | 输出：数据内容（≥8 字节） |
| `fifo` | `uint32_t` | FIFO 编号：`CAN_RX_FIFO0` 或 `CAN_RX_FIFO1` |

| 返回值 | 含义 |
|--------|------|
| `> 0` | 数据长度 DLC |
| `0` | 读取失败或参数无效 |

---

#### Weak 回调（用户可覆盖）

```c
// 达妙电机 CAN1 接收回调
__weak void dm_can1_rx_callback(hcan_t *hcan, uint16_t *rec_id, uint8_t *rx_data);

// 达妙电机 CAN2 接收回调
__weak void dm_can2_rx_callback(hcan_t *hcan, uint16_t *rec_id, uint8_t *rx_data);

// DJI 电机 CAN1 接收回调
__weak void dj_motor_can1_rx_callback(hcan_t *hcan, uint16_t *rec_id, uint8_t *rx_data);

// DJI 电机 CAN2 接收回调
__weak void dj_motor_can2_rx_callback(hcan_t *hcan, uint16_t *rec_id, uint8_t *rx_data);
```

**说明**：当没有注册 `STM32CAN_t` 对象式回调时，中断分发会回落到这些 weak 函数。用户在其它编译单元提供同名强定义即可接管。

---

#### `dm_can_send_data` — 通用标准帧发送（旧接口）

```c
uint8_t dm_can_send_data(hcan_t *hcan, uint16_t id, uint8_t *data, uint32_t len);
```

| 参数 | 取值范围 |
|------|---------|
| `id` | `0x000 ~ 0x7FF` |
| `data` | 非 NULL |
| `len` | `1 ~ 8` |

| 返回值 | 含义 |
|--------|------|
| `1` | 成功 |
| `0` | 失败 |

---

## 5. BSP UART DMA 驱动

**文件**：`bsp/bsp_uart/bsp_uart.h` / `bsp_uart.c`

提供两个独立通道：单缓冲 RX（循环 DMA）和双缓冲 TX（乒乓 DMA）。

### 通用类型

#### `BSP_UART_RawData_t` — 原始缓冲描述

```c
typedef struct {
    void *addr_;   // 缓冲区起始地址
    size_t size_;  // 缓冲区容量（字节）
} BSP_UART_RawData_t;
```

#### `STM32UART_RxCallback_t` — RX 回调类型

```c
typedef void (*STM32UART_RxCallback_t)(uint8_t *data, size_t size);
```

- 在 ISR 上下文中调用（不可阻塞）
- `data` / `size` 指向本次接收的数据片段
- DMA 循环缓冲回绕时会分两次回调

#### `STM32UART_TxCompleteCallback_t` — TX 完成回调

```c
typedef void (*STM32UART_TxCompleteCallback_t)(void);
```

- 在 `HAL_UART_TxCpltCallback` 中调用（ISR 上下文）

#### `BSP_UART_t` — 逻辑设备编号

```c
typedef enum {
#ifdef USART1
    BSP_USART1,
#endif
#ifdef USART2
    BSP_USART2,
#endif
#ifdef USART3
    BSP_USART3,
#endif
#ifdef USART6
    BSP_USART6,
#endif
#ifdef UART4
    BSP_UART4,
#endif
#ifdef UART5
    BSP_UART5,
#endif
    BSP_UART_NUMBER,
    BSP_UART_ID_ERROR
} BSP_UART_t;
```

---

### 5.1 单缓冲 DMA RX

#### `STM32UART_t` — RX 控制块

```c
typedef struct {
    BSP_UART_t id_;
    size_t last_rx_pos_;                 // 软件读指针
    BSP_UART_RawData_t dma_buff_rx_;     // 循环 DMA 原始缓冲
    UART_HandleTypeDef *uart_handle_;
    STM32UART_RxCallback_t rx_callback_; // 新数据片段回调
    err_t last_error_;
} STM32UART_t;
```

#### `STM32UART_Init` — 初始化 RX 控制块（绑定 UART 硬件和 DMA 缓冲）

```c
err_t STM32UART_Init(STM32UART_t *self,
                     UART_HandleTypeDef *uart_handle,
                     BSP_UART_RawData_t dma_buff_rx,
                     STM32UART_RxCallback_t callback);
```

- 第一个参数 `*self` —— `STM32UART_t` 结构体指针，该结构体包含软件读指针 (`last_rx_pos_`)、循环 DMA 缓冲 (`dma_buff_rx_`)、HAL UART 句柄 (`uart_handle_`) 和接收回调 (`rx_callback_`)，Init 时绑定到指定的 UART 外设和 DMA 缓冲
- 第二个参数 `uart_handle` —— HAL UART 句柄，指定使用哪个串口的 RX 通道
- 第三个参数 `dma_buff_rx` —— `BSP_UART_RawData_t` 类型，指定循环 DMA 缓冲的地址和大小（必须是 `{buffer, sizeof(buffer)}` 复合字面量）
- 第四个参数 `callback` —— 接收回调函数指针，UART 接收到数据时自动调用

| 返回值 | 含义 |
|--------|------|
| `OK` | 成功 |
| `NOT_FOUND` | 无法映射到 BSP 逻辑串口 ID |
| `SIZE_ERR` | DMA 缓冲大小为 0 |
| `BUSY` | 该 UART 外设已被其他控制块占用 |

**说明**：DMA 缓冲在整个接收期间必须保持有效；同一 UART 外设只能注册一个 RX 控制块

---

#### `STM32UART_SetRxDMA` — 启动 DMA 接收

```c
err_t STM32UART_SetRxDMA(STM32UART_t *self);
```

- 第一个参数 `*self` —— `STM32UART_t` 结构体指针，该结构体包含 HAL UART 句柄 (`uart_handle_`) 和循环 DMA 缓冲 (`dma_buff_rx_`)，SetRxDMA 时将 RX DMA 设为 CIRCULAR 模式并调用 `HAL_UARTEx_ReceiveToIdle_DMA` 启动接收

| 返回值 | 含义 |
|--------|------|
| `OK` | 成功 |
| `NOT_SUPPORT` | UART 未启用 RX 模式 |
| `INIT_ERR` | DMA 或 UART 初始化失败 |

**说明**：空闲线中断会自动切分数据包，无需外部干预

---

#### `STM32UART_SetRxCallback` — 更新接收回调

```c
void STM32UART_SetRxCallback(STM32UART_t *self,
                             STM32UART_RxCallback_t callback);
```

- 第一个参数 `*self` —— `STM32UART_t` 结构体指针，该结构体包含接收回调函数指针 (`rx_callback_`)，调用此函数替换该指针
- 第二个参数 `callback` —— 新接收回调函数指针，收到数据时自动调用

**说明**：不触碰 DMA 状态，只改变后续数据回调入口

---

#### `STM32UART_HandleRxData` — 处理接收数据

```c
void STM32UART_HandleRxData(STM32UART_t *self, uint8_t *data, size_t size);
```

- 第一个参数 `*self` —— `STM32UART_t` 结构体指针，该结构体包含接收回调 (`rx_callback_`)，HandleRxData 时调用该回调将数据片段传递给用户
- 第二个参数 `*data` —— 接收到的数据片段首地址
- 第三个参数 `size` —— 数据片段长度（字节）

**说明**：由 ISR 处理函数调用，循环 DMA 缓冲回绕时会自动拆分为两段分别回调

---

#### `STM32UART_GetLastError` — 读取错误码

```c
err_t STM32UART_GetLastError(const STM32UART_t *self);
```

- 第一个参数 `*self` —— `STM32UART_t` 结构体指针，该结构体包含最近一次操作错误码 (`last_error_`)，读取该字段

---

### 5.2 双缓冲 DMA TX

#### `STM32UARTDoubleBufTx_t` — TX 控制块

```c
typedef struct {
    BSP_UART_t id_;
    size_t last_tx_pos_;
    BSP_UART_RawData_t dma_buff_0_;             // TX 缓冲 0
    BSP_UART_RawData_t dma_buff_1_;             // TX 缓冲 1
    UART_HandleTypeDef *uart_handle_;
    STM32UART_TxCompleteCallback_t tx_callback_;
    volatile uint8_t active_buf_;               // 当前 DMA 使用哪个缓冲 (0/1)
    volatile size_t pending_size_;              // 等待发送的字节数
    volatile bool tx_busy_;                     // DMA 是否正在发送
} STM32UARTDoubleBufTx_t;
```

#### `STM32UARTDoubleBufTx_Init` — 初始化 TX 控制块（绑定 UART 硬件和双缓冲）

```c
err_t STM32UARTDoubleBufTx_Init(STM32UARTDoubleBufTx_t *self,
                                UART_HandleTypeDef *uart_handle,
                                BSP_UART_RawData_t dma_buff_0,
                                BSP_UART_RawData_t dma_buff_1,
                                STM32UART_TxCompleteCallback_t callback);
```

- 第一个参数 `*self` —— `STM32UARTDoubleBufTx_t` 结构体指针，该结构体包含两块 TX DMA 缓冲 (`dma_buff_0_`/`dma_buff_1_`)、HAL UART 句柄 (`uart_handle_`)、发送完成回调 (`tx_callback_`) 以及双缓冲状态变量 (`active_buf_`/`pending_size_`/`tx_busy_`)，Init 时绑定到指定 UART 和两块发送缓冲
- 第二个参数 `uart_handle` —— HAL UART 句柄，指定使用哪个串口的 TX 通道
- 第三个参数 `dma_buff_0` —— 第一块 TX DMA 缓冲（必须与 `dma_buff_1` 大小相同）
- 第四个参数 `dma_buff_1` —— 第二块 TX DMA 缓冲（必须与 `dma_buff_0` 大小相同）
- 第五个参数 `callback` —— 发送完成回调函数指针，DMA 发送完成后在 ISR 中调用，可 NULL

| 返回值 | 含义 |
|--------|------|
| `OK` | 成功 |
| `SIZE_ERR` | 两块缓冲大小不一致或为 0 |

**说明**：单次写入数据长度不能超过单块缓冲容量

---

#### `STM32UARTDoubleBufTx_SetTxDMA` — 配置 TX DMA

```c
err_t STM32UARTDoubleBufTx_SetTxDMA(STM32UARTDoubleBufTx_t *self);
```

- 第一个参数 `*self` —— `STM32UARTDoubleBufTx_t` 结构体指针，该结构体包含 HAL UART 句柄 (`uart_handle_`)，SetTxDMA 时将 TX DMA 设为 NORMAL 模式并复位双缓冲发送状态

| 返回值 | 含义 |
|--------|------|
| `OK` | 成功 |
| `NOT_SUPPORT` | UART 未启用 TX 模式 |
| `INIT_ERR` | DMA 初始化失败 |

---

#### `STM32UARTDoubleBufTx_Write` — 写入待发送数据

```c
err_t STM32UARTDoubleBufTx_Write(STM32UARTDoubleBufTx_t *self,
                                 const uint8_t *data,
                                 size_t size);
```

- 第一个参数 `*self` —— `STM32UARTDoubleBufTx_t` 结构体指针，该结构体包含两块 TX 缓冲 (`dma_buff_0_`/`dma_buff_1_`) 和 DMA 忙标志 (`tx_busy_`)，Write 时根据 `tx_busy_` 决定写入 active buffer（空闲时）或 pending buffer（忙时）
- 第二个参数 `*data` —— 待发送数据首地址
- 第三个参数 `size` —— 数据长度，范围 `1 ~ dma_buff_0_.size_`

**行为逻辑**：
- DMA 空闲 → 写入 active buffer 并立即调用 `Flush` 启动发送
- DMA 忙 → 写入另一块缓冲，等待完成回调后自动续发
- 连续写入会覆盖尚未发送的 pending 数据，调用方需控制发送节奏

---

#### `STM32UARTDoubleBufTx_Flush` — 提交 DMA 发送

```c
err_t STM32UARTDoubleBufTx_Flush(STM32UARTDoubleBufTx_t *self);
```

- 第一个参数 `*self` —— `STM32UARTDoubleBufTx_t` 结构体指针，该结构体包含 `active_buf_`（当前活动缓冲编号）和 `tx_busy_`（DMA 忙标志），Flush 时将当前 active buffer 中的 pending 数据通过 `HAL_UART_Transmit_DMA` 发送

| 返回值 | 含义 |
|--------|------|
| `OK` | 已提交 DMA 发送 |
| `EMPTY` | 没有 pending 数据 |
| `BUSY` | DMA 正在发送中 |

---

#### `STM32UARTDoubleBufTx_HandleTxComplete` — 处理 TX DMA 完成事件

```c
void STM32UARTDoubleBufTx_HandleTxComplete(STM32UARTDoubleBufTx_t *self);
```

- 第一个参数 `*self` —— `STM32UARTDoubleBufTx_t` 结构体指针，该结构体包含双缓冲状态变量，HandleTxComplete 时切换 active buffer 并检查另一块缓冲是否有 pending 数据待续发

**内部操作**：
1. 清除 `tx_busy_` 标志
2. 切换 `active_buf_`（0 ↔ 1）
3. 如果另一块缓冲有 pending 数据则自动调用 `Flush` 续发
4. 调用用户 TX 完成回调

**说明**：在 `HAL_UART_TxCpltCallback` 中自动调用

---

#### `STM32UARTDoubleBufTx_SetTxCompleteCallback` — 更新 TX 完成回调

```c
void STM32UARTDoubleBufTx_SetTxCompleteCallback(
    STM32UARTDoubleBufTx_t *self,
    STM32UART_TxCompleteCallback_t callback);
```

- 第一个参数 `*self` —— `STM32UARTDoubleBufTx_t` 结构体指针，该结构体包含 TX 完成回调指针 (`tx_callback_`)，调用此函数替换该指针
- 第二个参数 `callback` —— 新 TX 完成回调函数指针，可 NULL 清除回调

---

#### `STM32UARTDoubleBufTx_GetLastError` — 读取错误码

```c
err_t STM32UARTDoubleBufTx_GetLastError(const STM32UARTDoubleBufTx_t *self);
```

- 第一个参数 `*self` —— `STM32UARTDoubleBufTx_t` 结构体指针，读取其 `last_error_` 字段

---

### 5.3 HAL 回调分发

```c
// RX 空闲线中断 → FIFO 切片 → 用户回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

// TX DMA 完成 → 切换 buffer → 续发 pending → 用户回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
```

**说明**：BSP 自动注册 HAL 回调并分发到正确的对象表。用户在 CubeMX 生成代码后不需要修改 `stm32f4xx_it.c` 中的中断处理。

---

## 6. BSP PWM 驱动

**文件**：`bsp/bsp_pwm/bsp_pwm.h` / `bsp_pwm.c`

### `STM32PWM_t` — PWM 控制块

```c
typedef struct {
    TIM_HandleTypeDef *tim_handle_;  // CubeMX 生成的 TIM 句柄
    uint32_t channel_;               // TIM_CHANNEL_x
    float duty_cycle_;               // 当前占空比 [0.0, 1.0]
    bool started_;                   // 是否已启动
    err_t last_error_;               // 最近错误码
} STM32PWM_t;
```

#### `STM32PWM_Init` — 初始化（绑定定时器和通道）

```c
err_t STM32PWM_Init(STM32PWM_t *self,
                    TIM_HandleTypeDef *tim_handle,
                    uint32_t channel);
```

- 第一个参数 `*self` —— `STM32PWM_t` 结构体指针，该结构体包含 HAL 定时器句柄 (`tim_handle_`)、PWM 通道 (`channel_`)、当前占空比 (`duty_cycle_`) 和启动标志 (`started_`)，Init 时绑定到指定定时器和通道
- 第二个参数 `tim_handle` —— HAL 定时器句柄，指定使用哪个定时器产生 PWM
- 第三个参数 `channel` —— PWM 输出通道，指定定时器的哪个通道输出 PWM 信号

| 返回值 | 含义 |
|--------|------|
| `OK` | 成功 |

---

#### `STM32PWM_Start` — 启动 PWM 输出

```c
err_t STM32PWM_Start(STM32PWM_t *self);
```

- 第一个参数 `*self` —— `STM32PWM_t` 结构体指针，该结构体包含定时器句柄和通道号，Start 时调用 `HAL_TIM_PWM_Start` 启动对应通道的 PWM 输出

---

#### `STM32PWM_SetDutyCycle` — 设置占空比

```c
err_t STM32PWM_SetDutyCycle(STM32PWM_t *self, float duty_cycle);
```

- 第一个参数 `*self` —— `STM32PWM_t` 结构体指针，该结构体包含定时器句柄和通道号，SetDutyCycle 时通过 `__HAL_TIM_SET_COMPARE` 写入新的占空比值到 CCR 寄存器
- 第二个参数 `duty_cycle` —— 占空比，范围 `[0.0, 1.0]`，0.5 表示 50%

**内部实现**：`pulse = (uint32_t)(duty_cycle * ARR)`，写入对应通道 CCR 寄存器

---

#### `STM32PWM_Stop` — 停止 PWM 输出

```c
err_t STM32PWM_Stop(STM32PWM_t *self);
```

- 第一个参数 `*self` —— `STM32PWM_t` 结构体指针，该结构体包含定时器句柄和通道号，Stop 时调用 `HAL_TIM_PWM_Stop` 停止对应通道的 PWM 输出

---

#### `STM32PWM_GetLastError` — 读取错误码

```c
err_t STM32PWM_GetLastError(const STM32PWM_t *self);
```

- 第一个参数 `*self` —— `STM32PWM_t` 结构体指针，读取其 `last_error_` 字段

---

### 完整使用示例

```c
static STM32PWM_t servo_pwm;

void init_servo(void) {
    STM32PWM_Init(&servo_pwm, &htim2, TIM_CHANNEL_1);
    STM32PWM_Start(&servo_pwm);
    STM32PWM_SetDutyCycle(&servo_pwm, 0.075f);  // 7.5% → 舵机中位
}
```

---

## 7. BSP I²C 驱动

**文件**：`bsp/bsp_iic/bsp_iic.h` / `bsp_iic.c`

> **⚠️ 当前状态**：`bsp/CMakeLists.txt` 中 I²C 驱动已注释，不参与编译。如需使用，取消注释。

### `BSP_IIC_Callback_t` — 事件回调类型

```c
typedef enum {
    BSP_IIC_MASTER_TX_CPLT_CB,   // Master 发送完成
    BSP_IIC_MASTER_RX_CPLT_CB,   // Master 接收完成
    BSP_IIC_SLAVE_TX_CPLT_CB,    // Slave 发送完成
    BSP_IIC_SLAVE_RX_CPLT_CB,    // Slave 接收完成
    BSP_IIC_LISTEN_CPLT_CB,      // Listen 完成
    BSP_IIC_MEM_TX_CPLT_CB,      // Memory 发送完成
    BSP_IIC_MEM_RX_CPLT_CB,      // Memory 接收完成
    BSP_IIC_ERROR_CB,            // 错误
    BSP_IIC_ABORT_CPLT_CB,       // Abort 完成
    BSP_IIC_CB_NUMBER
} BSP_IIC_Callback_t;
```

### `STM32IIC_RegisterCallback` — 注册事件回调

```c
err_t STM32IIC_RegisterCallback(STM32IIC_t *self,
                                BSP_IIC_Callback_t type,
                                STM32IIC_Callback_t callback);
```

| 参数 | 说明 |
|------|------|
| `type` | 事件类型（见枚举） |
| `callback` | 用户回调，非 NULL |

---

## 8. BSP USB CDC 驱动

**文件**：`bsp/bsp_usb/bsp_usb.h` / `bsp_usb.c`

> **⚠️ 当前状态**：`bsp/CMakeLists.txt` 中 USB 驱动已注释，不参与编译。

#### `STM32USB_Write` — CDC 发送数据

```c
err_t STM32USB_Write(STM32USB_t *self, const uint8_t *data, size_t size);
```

| 参数 | 取值范围 |
|------|---------|
| `size` | `1 ~ APP_TX_DATA_SIZE`（通常 2048） |

| 返回值 | 含义 |
|--------|------|
| `STATE_ERR` | USB 未枚举（`dev_state != USBD_STATE_CONFIGURED`） |
| `BUSY` | 上次发送未完成（`TxState != 0`） |
| `OUT_OF_RANGE` | 数据过长 |

**说明**：发送前自动复制到 `UserTxBufferFS`，避免调用方栈数据在 USB 完成前失效。

#### `STM32USB_IsConfigured` — 检查 USB 枚举状态

```c
bool STM32USB_IsConfigured(const STM32USB_t *self);
```

返回 `true` 表示 USB 已枚举且处于 CONFIGURED 状态。

---

## 9. BSP DWT 精密定时

**文件**：`bsp/dwt/bsp_dwt.h` / `bsp_dwt.c`

基于 Cortex-M DWT CYCCNT 周期计数器，提供纳秒级精度的时间测量。

### `DWT_Time_t` — 软件时间轴拆分

```c
typedef struct {
    uint32_t s;    // 秒
    uint16_t ms;   // 当前秒内的毫秒余数
    uint16_t us;   // 当前毫秒内的微秒余数
} DWT_Time_t;
```

---

#### `DWT_Init` — 初始化 DWT

```c
void DWT_Init(uint32_t CPU_Freq_mHz);
```

| 参数 | 说明 | 本工程使用 |
|------|------|-----------|
| `CPU_Freq_mHz` | CPU 主频（单位 MHz） | `144`（SYSCLK = 144MHz） |

**内部操作**：
1. 使能 DWT/ITM 调试跟踪外设（`CoreDebug->DEMCR |= TRCENA_Msk`）
2. 清零并启动 CYCCNT 计数器
3. 保存频率换算因子

---

#### `DWT_GetDeltaT` — 获取时间间隔

```c
float DWT_GetDeltaT(uint32_t *cnt_last);
```

| 参数 | 说明 |
|------|------|
| `cnt_last` | 上一次保存的 CYCCNT 值（函数内自动更新） |

| 返回值 | 说明 |
|--------|------|
| `float` | 本次调用与上次调用之间的秒数 |

**示例**：

```c
uint32_t tick = 0;

void control_loop(void) {
    float dt = DWT_GetDeltaT(&tick);  // 获取控制周期（秒）
    // dt 可用于 PID 计算等
}
```

---

#### `DWT_GetDeltaT64` — 高精度时间间隔

```c
double DWT_GetDeltaT64(uint32_t *cnt_last);
```

与 `DWT_GetDeltaT` 相同，但返回 `double` 精度。

---

#### `DWT_GetTimeline_s` — 秒级时间轴

```c
float DWT_GetTimeline_s(void);
```

返回从 `DWT_Init` 开始到现在的秒数。

#### `DWT_GetTimeline_ms` — 毫秒级时间轴

```c
float DWT_GetTimeline_ms(void);
```

#### `DWT_GetTimeline_us` — 微秒级时间轴

```c
uint64_t DWT_GetTimeline_us(void);
```

#### `DWT_Delay` — 忙等待延时

```c
void DWT_Delay(float Delay);
```

| 参数 | 说明 |
|------|------|
| `Delay` | 延时秒数（如 0.001 = 1ms） |

**说明**：不依赖 SysTick，可在临界区或中断关闭期间使用。但会占用 CPU 100%。

---

#### `DWT_SysTimeUpdate` — 刷新时间轴

```c
void DWT_SysTimeUpdate(void);
```

手动更新软件时间轴。如果长时间不调用 timeline 读取函数，CYCCNT 可能溢出导致时间轴滞后。三个 `DWT_GetTimeline_*` 函数内部会自动调用此函数。

---

### `TIME_ELAPSE` 宏 — 代码段计时

```c
#define TIME_ELAPSE(dt, code) \
    do { \
        float tstart = DWT_GetTimeline_s(); \
        code; \
        dt = DWT_GetTimeline_s() - tstart; \
        LOGINFO("[DWT] " #dt " = %f s\r\n", dt); \
    } while (0)
```

**示例**：

```c
float elapsed;
TIME_ELAPSE(elapsed, {
    HAL_Delay(10);
});
// elapsed ≈ 0.01f
```

---

## 10. DR16 大疆遥控器（SBUS）

**文件**：`modules/DR16/dr16.h` / `dr16.c`

### 协议说明

| 参数 | 值 |
|------|-----|
| 物理层 | USART2, 100kbps, 8E2 |
| 帧长 | 18 字节 |
| 发送周期 | 14ms |
| 摇杆原始值 | 364 ~ 1684（中值 1024） |
| 离线超时 | 20ms |

---

### `DR16_t` — DR16 对象

```c
typedef struct {
    TaskHandle_t thread_alert;      // FreeRTOS 任务句柄（ISR 通知用）
    BaseType_t switch_required;     // 预留
    cmd_rc_t dr16_cmd;              // 归一化后的遥控器命令
    bool online_;                   // 在线标志
    STM32UART_t uart_;              // UART DMA 接收通道（内部使用）
    err_t init_error_;              // 初始化错误码
    dr16_data_t dr16_data_;         // 最近一次原始帧快照
} DR16_t;
```

**`DR16_t` 结构体关键字段**：

| 字段 | 类型 | 说明 | 由谁设置 |
|------|------|------|---------|
| `thread_alert` | `TaskHandle_t` | ISR 通知用的 FreeRTOS 任务句柄，初始化后、Start **前**由用户赋值为 `xTaskGetCurrentTaskHandle()` | **用户必须设置** |
| `dr16_cmd` | `cmd_rc_t` | 归一化后的遥控器命令（摇杆 ±1，拨杆/键盘/鼠标映射） | `DR16_Update` |
| `online_` | `bool` | 在线标志，最近一次 Update 是否收到合法 SBUS 帧 | `DR16_Update` |
| `init_error_` | `err_t` | Init 阶段错误码，Start 前可查询 | `DR16_Init` |
| `uart_` | `STM32UART_t` | UART DMA 接收通道（内部使用），存储接收状态 | `DR16_Init` |
| `dr16_data_` | `dr16_data_t` | 最近一次原始 SBUS 帧快照 | `DR16_Update` |

---

### `DR16_Init` — 初始化（绑定 UART 硬件）

```c
err_t DR16_Init(DR16_t *self, UART_HandleTypeDef *uart_handle);
```

- 第一个参数 `*self` —— `DR16_t` 结构体指针，该结构体包含 UART DMA 接收通道 (`uart_`)、解析后的遥控器命令 (`dr16_cmd`)、在线标志 (`online_`) 和原始帧快照 (`dr16_data_`)，Init 时清零所有字段并绑定到指定 UART 外设
- 第二个参数 `uart_handle` —— HAL UART 句柄，指定使用哪个串口连接 DR16 接收机，通常 `&huart2`

**内部操作**：
1. `memset` 清零 `DR16_t` 对象
2. 调用 `STM32UART_Init` 将 `self->uart_` 绑定到 `uart_handle` 和内部 DMA 缓冲
3. 注册 `DR16_RxCallback` 到 BSP UART（ISR 中收到完整 18 字节帧后通过 `xTaskNotifyFromISR` 通知任务）

---

### `DR16_Start` — 启动 DMA 接收

```c
err_t DR16_Start(DR16_t *self);
```

- 第一个参数 `*self` —— `DR16_t` 结构体指针，该结构体包含 UART DMA 接收通道 (`uart_`)，Start 时调用 `STM32UART_SetRxDMA` 启动该通道的循环 DMA 接收

**说明**：启动后 UART DMA 持续接收，ISR 回调自动帧同步

---

### `DR16_Update` — 更新遥控器状态（任务周期调用）

```c
void DR16_Update(DR16_t *self, uint32_t timeout_ms);
```

- 第一个参数 `*self` —— `DR16_t` 结构体指针，该结构体包含 UART 接收通道 (`uart_`)、FreeRTOS 任务句柄 (`thread_alert`) 和解析后的遥控器数据，Update 时等待 ISR 通知并更新这些字段
- 第二个参数 `timeout_ms` —— 超时时间（ms），超过此时间未收到合法帧则置 `online_ = false`，建议 `DR16_OFFLINE_TIMEOUT_MS` (20)

**行为**：
1. 调用 `xTaskNotifyWait` 阻塞等待 ISR 通知（`SIGNAL_DR16_RAW_REDY`）
2. 超时 → `online_ = false`，清零 `dr16_cmd`
3. 收到通知 → 快照原始帧，调用 `DR16_ParseRc` 解析
4. 解析成功 → `online_ = true`，`dr16_cmd` 更新
5. 数据异常（通道值超出 `[364, 1684]` 或拨杆无效）→ 保持上次状态

**必须在 FreeRTOS 任务中调用**，`self->thread_alert` 需提前赋值为当前任务句柄。

---

### `dr16_data_t` — 原始帧结构

```c
typedef struct __attribute__((packed)) {
    // 摇杆（11 位各）
    uint16_t ch_r_x : 11;  // 右 X: 364(左) ~ 1024(中) ~ 1684(右)
    uint16_t ch_r_y : 11;  // 右 Y: 364(后) ~ 1024(中) ~ 1684(前)
    uint16_t ch_l_x : 11;  // 左 X: 364(左) ~ 1024(中) ~ 1684(右)
    uint16_t ch_l_y : 11;  // 左 Y: 364(后) ~ 1024(中) ~ 1684(前)

    uint8_t sw_r : 2;      // 右拨杆: 1=上, 3=中, 2=下
    uint8_t sw_l : 2;      // 左拨杆: 1=上, 3=中, 2=下

    int16_t x;             // 鼠标 X 增量
    int16_t y;             // 鼠标 Y 增量
    int16_t z;             // 鼠标滚轮增量

    uint8_t press_l;       // 鼠标左键 (0/1)
    uint8_t press_r;       // 鼠标右键 (0/1)

    uint16_t key;          // 键盘位图
    uint16_t res;          // 保留通道
} dr16_data_t;
```

**解析后的 `cmd_rc_t` 字段**：

| 字段 | 来源 | 范围 |
|------|------|------|
| `ch.r.x` | 右摇杆 X | ≈ -1.0 ~ 1.0 |
| `ch.r.y` | 右摇杆 Y | ≈ -1.0 ~ 1.0 |
| `ch.l.x` | 左摇杆 X | ≈ -1.0 ~ 1.0 |
| `ch.l.y` | 左摇杆 Y | ≈ -1.0 ~ 1.0 |
| `sw_l` | 左拨杆 | `CMD_SW_UP/MID/DOWN` |
| `sw_r` | 右拨杆 | `CMD_SW_UP/MID/DOWN` |
| `mouse.x/y/z` | 鼠标增量 | 原始值 |
| `mouse.click.l/r` | 鼠标按键 | `0/1` |
| `key` | 键盘位图 | 见 `cmd_key_t` |

---

### 完整任务示例

```c
#include "dr16.h"

DR16_t *dr16 = NULL;

void dr16_task(void *argument) {
    static DR16_t dr16_instance;

    // 第 1 步：初始化，绑定 USART2
    err_t status = DR16_Init(&dr16_instance, &huart2);
    dr16 = &dr16_instance;

    // 第 2 步：保存任务句柄
    dr16->thread_alert = xTaskGetCurrentTaskHandle();

    // 第 3 步：启动 DMA
    if (status == OK) status = DR16_Start(dr16);
    ASSERT(status == OK);
    if (status != OK) vTaskDelete(NULL);

    // 第 4 步：周期性更新
    for (;;) {
        DR16_Update(dr16, DR16_OFFLINE_TIMEOUT_MS);

        if (dr16->online_) {
            // 使用 dr16->dr16_cmd 控制
        }
    }
}
```

---

## 11. I6X 富斯 iBus 遥控器

**文件**：`modules/I6X/i6x.h` / `i6x.c`

### 协议说明

| 参数 | 值 |
|------|-----|
| 物理层 | UART, 115200, 8N1, 3.3V 直连 |
| 帧长 | 32 字节 |
| 发送周期 | 7ms |
| 通道值 | 1000 ~ 2000（中值 1500） |
| 校验和 | `0xFFFF - sum(byte[0..29])` |

### `i6x_frame_sync_t` — 字节流帧同步器

```c
typedef struct {
    uint8_t buffer[I6X_FRAME_SIZE];  // 帧累积缓冲（32 字节）
    size_t index;                    // 当前累积字节数
} i6x_frame_sync_t;
```

#### `I6X_FrameSyncReset` — 复位帧同步器

```c
void I6X_FrameSyncReset(i6x_frame_sync_t *sync);
```

- 第一个参数 `*sync` —— `i6x_frame_sync_t` 结构体指针，该结构体包含 32 字节累积缓冲 (`buffer`) 和当前累积索引 (`index`)，Reset 时清空这些字段

#### `I6X_FrameSyncPush` — 输入 1 字节到帧同步器

```c
bool I6X_FrameSyncPush(i6x_frame_sync_t *sync, uint8_t byte,
                       uint8_t out_frame[I6X_FRAME_SIZE]);
```

- 第一个参数 `*sync` —— `i6x_frame_sync_t` 结构体指针，该结构体包含累积缓冲 (`buffer`) 和索引 (`index`)，Push 时将字节写入缓冲并检测帧头 `0x20 0x40`
- 第二个参数 `byte` —— 从 UART 接收到的 1 个字节
- 第三个参数 `*out_frame` —— 输出缓冲区，当函数返回 `true` 时包含 32 字节的完整候选帧

| 返回值 | 含义 |
|--------|------|
| `true` | 已拼出完整候选帧（存于 `out_frame`），可调用 `I6X_DecodeFrame` 解码 |
| `false` | 仍在累积中 |

**帧同步逻辑**：等待连续 `0x20 0x40` 帧头 → 累积到 32 字节后返回完整候选帧 → 自动复位重新搜索

---

#### `I6X_DecodeFrame` — 解码 iBus 帧

```c
err_t I6X_DecodeFrame(const uint8_t frame[I6X_FRAME_SIZE], i6x_cmd_rc_t *cmd);
```

- 第一个参数 `*frame` —— 32 字节 iBus 原始帧数据，由 `I6X_FrameSyncPush` 拼出
- 第二个参数 `*cmd` —— `i6x_cmd_rc_t` 结构体指针，该结构体包含归一化通道值 (`ch`)、14 路原始通道值 (`channel`)、辅助通道 (`aux`)、校验和和帧头状态，解码成功后填入这些字段

| 返回值 | 含义 |
|--------|------|
| `OK` | 解码成功，`cmd` 已填入有效数据 |
| `FAILED` | 帧头错误（`frame[0] != 0x20` 或 `frame[1] != 0x40`） |
| `CHECK_ERR` | 校验和不匹配 |
| `OUT_OF_RANGE` | 某通道值不在 `[1000, 2000]` 范围内 |

**校验内容**：帧头校验 → 14 通道范围校验 (`1000~2000`) → 校验和校验 (`0xFFFF - sum(frame[0..29])`)

---

### `i6x_cmd_rc_t` — 解码数据结构

```c
typedef struct {
    struct {
        vector2_t l;  // 左摇杆 (x: CH4 航向, y: CH3 油门)
        vector2_t r;  // 右摇杆 (x: CH1 横滚, y: CH2 俯仰)
    } ch;

    uint16_t channel[I6X_CHANNEL_COUNT];  // 14 路原始值 (1000~2000)
    float aux[I6X_AUX_CHANNEL_COUNT];     // CH5~CH14 归一化 (-1~1)
    uint16_t checksum_cal;                // 本地校验和
    uint16_t checksum_rx;                 // 帧内校验和
    struct {
        uint8_t length, command;
        bool valid;
    } frame;
} i6x_cmd_rc_t;
```

**通道映射**：

| 通道 | 字段 | 含义 |
|------|------|------|
| CH1 | `ch.r.x` | 右摇杆左右（横滚 Roll） |
| CH2 | `ch.r.y` | 右摇杆前后（俯仰 Pitch） |
| CH3 | `ch.l.y` | 左摇杆前后（油门 Throttle） |
| CH4 | `ch.l.x` | 左摇杆左右（航向 Yaw） |
| CH5~CH14 | `aux[0..9]` | 辅助通道，归一化 -1~1 |

---

### `I6X_t` — I6X 遥控器对象

```c
typedef struct {
    TaskHandle_t thread_alert;   // ISR 通知用的 FreeRTOS 任务句柄，用户必须在 Start 前赋值
    i6x_cmd_rc_t cmd;            // 最近一次解码结果（含归一化通道和原始值）
    bool online_;                // 在线标志
    STM32UART_t uart_;           // UART DMA 接收通道（内部使用）
    err_t init_error_;
    uint8_t raw_frame[I6X_FRAME_SIZE];  // 最近一次原始帧快照
} I6X_t;
```

---

### `I6X_Init` — 初始化（绑定 UART 硬件）

```c
err_t I6X_Init(I6X_t *self, UART_HandleTypeDef *uart_handle);
```

- 第一个参数 `*self` —— `I6X_t` 结构体指针，该结构体包含 UART DMA 接收通道 (`uart_`)、解码后的命令 (`cmd`)、在线标志 (`online_`) 和原始帧缓冲 (`raw_frame`)，Init 时清零所有字段并绑定到指定 UART 外设
- 第二个参数 `uart_handle` —— HAL UART 句柄，指定使用哪个串口连接 I6X 接收机，通常 `&huart4`

---

### `I6X_Start` — 启动 DMA 接收

```c
err_t I6X_Start(I6X_t *self);
```

- 第一个参数 `*self` —— `I6X_t` 结构体指针，该结构体包含 UART DMA 接收通道 (`uart_`)，Start 时调用 `STM32UART_SetRxDMA` 启动该通道的循环 DMA 接收

---

### `I6X_Update` — 更新遥控器状态（任务周期调用）

```c
void I6X_Update(I6X_t *self, uint32_t timeout_ms);
```

- 第一个参数 `*self` —— `I6X_t` 结构体指针，该结构体包含 UART 接收通道 (`uart_`)、任务句柄 (`thread_alert`) 和在线标志，Update 时等待 ISR 通知并更新这些字段
- 第二个参数 `timeout_ms` —— 超时时间（ms），超过此时间未收到合法帧则置 `online_ = false`

---

### 完整任务示例

```c
#include "i6x.h"

I6X_t *i6x = NULL;

void I6X_task(void *argument) {
    static I6X_t i6x_instance;

    err_t status = I6X_Init(&i6x_instance, &huart4);
    i6x = &i6x_instance;
    i6x->thread_alert = xTaskGetCurrentTaskHandle();

    if (status == OK) status = I6X_Start(i6x);
    ASSERT(status == OK);
    if (status != OK) vTaskDelete(NULL);

    for (;;) {
        I6X_Update(i6x, I6X_OFFLINE_TIMEOUT_MS);

        if (i6x->online_) {
            float roll  = i6x->cmd.ch.r.x;   // CH1
            float pitch = i6x->cmd.ch.r.y;   // CH2
            float throttle = i6x->cmd.ch.l.y; // CH3
            float yaw  = i6x->cmd.ch.l.x;     // CH4
            float aux5 = i6x->cmd.aux[0];      // CH5
        }
    }
}
```

---

## 12. LX824 总线串口舵机

**文件**：`modules/LX824/lx824.h` / `lx824.c`

### 协议说明

| 参数 | 值 |
|------|-----|
| 物理层 | 半双工 UART |
| 帧头 | `0x55 0x55` |
| 帧格式 | `[0x55][0x55][ID][Length][Cmd][Params...][Checksum]` |
| 校验和 | `~(ID + Length + Cmd + Params)` 取最低字节 |
| 舵机 ID | `0 ~ 253`（`0xFE` = 广播） |
| 角度范围 | `0 ~ 1000`（对应 0° ~ 240°） |
| 运动时间 | `0 ~ 30000` ms |

---

### `LX824_t` — 舵机总线对象

```c
typedef struct {
    TaskHandle_t thread_alert;          // FreeRTOS 任务句柄（ISR→任务通知）
    STM32UARTDoubleBufTx_t uart_send_;  // 发送通道（双缓冲 DMA TX）→ 发指令到舵机
    STM32UART_t uart_receive_;           // 接收通道（循环 DMA RX）→ 收舵机应答
    err_t tx_init_error_;               // TX 初始化结果
    err_t rx_init_error_;               // RX 初始化结果
} LX824_t;
```

### `LX824_Init` — 初始化（绑定 UART 硬件）

```c
err_t LX824_Init(LX824_t *self, UART_HandleTypeDef *uart_handle);
```

- 第一个参数 `*self` —— `LX824_t` 结构体指针，该结构体包含 TX 双缓冲 DMA 发送通道 (`uart_send_`)、RX 循环 DMA 接收通道 (`uart_receive_`)、任务通知句柄 (`thread_alert`) 和初始化错误码，Init 时绑定到指定 UART 外设并初始化收发 DMA 通道。**本工程使用 USART1（`&huart1`）连接 LX824 舵机总线**
- 第二个参数 `uart_handle` —— HAL UART 句柄，指定使用哪个串口与舵机总线通信

**内部操作**：
- `self->uart_send_` ← 初始化 TX 双缓冲 DMA（发指令给舵机）
- `self->uart_receive_` ← 初始化 RX 循环 DMA（收舵机应答）
- 注册 ISR 回调 `LX824_RxCallback`（收到字节后写入 FIFO 并通知任务）
- 设置 `instance_ = self`（全局单例，供 ISR 回调使用）

**重要**：
- 初始化后 `self` 即绑定了 USART1，所有后续 API 通过 `self` 间接操作该 UART
- 同一总线上只需一个 `LX824_t` 实例，所有舵机共享此总线

---

### `LX824_Start` — 启动收发 DMA

```c
err_t LX824_Start(LX824_t *self);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| **`self`** | `LX824_t *` | 已初始化的舵机总线对象 |

**内部操作**：
1. `STM32UARTDoubleBufTx_SetTxDMA(&self->uart_send_)` — 启动 TX DMA
2. `STM32UART_SetRxDMA(&self->uart_receive_)` — 启动 RX 循环 DMA

**说明**：必须在调用任何读写 API **之前**调用。成功后，`self->uart_send_` 和 `self->uart_receive_` 的 DMA 通道开始工作。

---

### `LX824_Update` — 任务周期处理

```c
void LX824_Update(LX824_t *self, uint32_t timeout_ms);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| **`self`** | `LX824_t *` | 舵机总线对象 |
| `timeout_ms` | `uint32_t` | 等待 ISR 通知的超时时间 |

**行为**：
- 调用 `xTaskNotifyWait` 等待 ISR 通知
- 排空 `self->uart_receive_` 的 FIFO，逐字节喂给应答解析状态机
- 如果业务代码中使用了读指令 API（阻塞式请求-应答），**读指令内部已包含等待逻辑**，不需要额外调用 `Update`；但若想同时处理背景应答（如舵机主动上报），仍需周期性调用

---

### 写指令 API 详解

所有写指令的第一个参数 **必须** 是 `LX824_t *self`——它指定了往哪条总线发指令。返回值均为 `err_t`，`OK` 表示组帧成功并写入 TX DMA 缓冲。

---

#### `LX824_MoveTimeWrite` — 立即转动（Cmd 1）

```c
err_t LX824_MoveTimeWrite(LX824_t *self, uint8_t id, uint16_t angle, uint16_t time_ms);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** — 指令通过 `self->uart_send_` 发出 |
| `id` | `uint8_t` | `0 ~ 253` / `0xFE` | 目标舵机 ID，`0xFE`=广播（所有舵机都执行） |
| `angle` | `uint16_t` | `0 ~ 1000` | 目标角度，0=0°, 1000=240° |
| `time_ms` | `uint16_t` | `0 ~ 30000` | 运动时间（毫秒） |

**行为**：舵机以匀速在指定时间内从当前位置转动到 `angle`。`time_ms` 越短速度越快。

---

#### `LX824_MoveTimeWaitWrite` — 预设动作（Cmd 7）

```c
err_t LX824_MoveTimeWaitWrite(LX824_t *self, uint8_t id, uint16_t angle, uint16_t time_ms);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** |
| `id` | `uint8_t` | `0 ~ 253` / `0xFE` | 舵机 ID |
| `angle` | `uint16_t` | `0 ~ 1000` | 预设目标角度 |
| `time_ms` | `uint16_t` | `0 ~ 30000` | 预设运动时间 |

**用途**：多个舵机同步动作。先给各舵机发 `MoveTimeWaitWrite` 预设，再统一发 `MoveStart` 同时启动。

**示例**：

```c
// 舵机 1 和 2 同步运动
LX824_MoveTimeWaitWrite(lx824, 1, 500, 1000);  // 舵机1: 到500, 耗时1秒
LX824_MoveTimeWaitWrite(lx824, 2, 800, 1500);  // 舵机2: 到800, 耗时1.5秒
LX824_MoveStart(lx824, 0xFE);                  // 一起启动
```

---

#### `LX824_MoveStart` — 启动预设动作（Cmd 11）

```c
err_t LX824_MoveStart(LX824_t *self, uint8_t id);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** |
| `id` | `uint8_t` | `0 ~ 253` / `0xFE` | 舵机 ID |

启动之前通过 `MoveTimeWaitWrite` 预设的动作。通常用 `0xFE` 广播触发所有舵机。

---

#### `LX824_MoveStop` — 立即停止（Cmd 12）

```c
err_t LX824_MoveStop(LX824_t *self, uint8_t id);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** |
| `id` | `uint8_t` | `0 ~ 253` / `0xFE` | 舵机 ID |

舵机立即停止在**当前角度**，不会回零。紧急停止用。

---

#### `LX824_IdWrite` — 写舵机 ID（Cmd 13，掉电保存）

```c
err_t LX824_IdWrite(LX824_t *self, uint8_t id, uint8_t new_id);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** |
| `id` | `uint8_t` | `0 ~ 253` / `0xFE` | 当前舵机 ID（未知时用 `0xFE`） |
| `new_id` | `uint8_t` | `0 ~ 253` | 新 ID |

**说明**：ID 写入后掉电保存。总线只挂一个舵机时可用广播 `0xFE` 设置。

---

#### `LX824_AngleOffsetAdjust` — 调整偏差（Cmd 17，不保存）

```c
err_t LX824_AngleOffsetAdjust(LX824_t *self, uint8_t id, int8_t offset);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** |
| `id` | `uint8_t` | `0 ~ 253` | 舵机 ID |
| `offset` | `int8_t` | `-125 ~ +125` | 偏差值，约 -30° ~ +30° |

临时调整舵机中位偏差，掉电后恢复。如需永久保存，需再调用 `LX824_AngleOffsetWrite`。

---

#### `LX824_AngleOffsetWrite` — 保存偏差（Cmd 18，掉电保存）

```c
err_t LX824_AngleOffsetWrite(LX824_t *self, uint8_t id);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** |
| `id` | `uint8_t` | `0 ~ 253` | 舵机 ID |

将上次 `AngleOffsetAdjust` 设定的偏差写入非易失存储。

---

#### `LX824_AngleLimitWrite` — 写角度限制（Cmd 20）

```c
err_t LX824_AngleLimitWrite(LX824_t *self, uint8_t id, uint16_t min_angle, uint16_t max_angle);
```

| 参数 | 类型 | 范围 | 要求 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** |
| `id` | `uint8_t` | `0 ~ 253` | 舵机 ID |
| `min_angle` | `uint16_t` | `0 ~ 1000` | `< max_angle` |
| `max_angle` | `uint16_t` | `0 ~ 1000` | `> min_angle` |

---

#### `LX824_VinLimitWrite` — 写电压限制（Cmd 22）

```c
err_t LX824_VinLimitWrite(LX824_t *self, uint8_t id, uint16_t min_mv, uint16_t max_mv);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** |
| `id` | `uint8_t` | `0 ~ 253` | 舵机 ID |
| `min_mv` | `uint16_t` | `4500 ~ 12000` | 最低电压（mV） |
| `max_mv` | `uint16_t` | `4500 ~ 12000` | 最高电压（mV） |

---

#### `LX824_TempMaxLimitWrite` — 写温度限制（Cmd 24）

```c
err_t LX824_TempMaxLimitWrite(LX824_t *self, uint8_t id, uint8_t temp_c);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** |
| `id` | `uint8_t` | `0 ~ 253` | 舵机 ID |
| `temp_c` | `uint8_t` | `50 ~ 100` | 最高温度（摄氏度） |

---

#### `LX824_OrMotorModeWrite` — 写舵机/电机模式（Cmd 29）

```c
err_t LX824_OrMotorModeWrite(LX824_t *self, uint8_t id, uint8_t mode, int16_t speed);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** |
| `id` | `uint8_t` | `0 ~ 253` | 舵机 ID |
| `mode` | `uint8_t` | `0` 或 `1` | `0`=位置控制（舵机模式），`1`=电机控制 |
| `speed` | `int16_t` | `-1000 ~ +1000` | 电机模式下速度值（电机控制时有效） |

---

#### `LX824_LoadOrUnloadWrite` — 装载/卸载电机（Cmd 31）

```c
err_t LX824_LoadOrUnloadWrite(LX824_t *self, uint8_t id, uint8_t load);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** |
| `id` | `uint8_t` | `0 ~ 253` | 舵机 ID |
| `load` | `uint8_t` | `0` 或 `1` | `0`=卸载（掉电无力矩），`1`=装载（输出力矩） |

---

#### `LX824_LedCtrlWrite` — LED 控制（Cmd 33）

```c
err_t LX824_LedCtrlWrite(LX824_t *self, uint8_t id, uint8_t off);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** |
| `id` | `uint8_t` | `0 ~ 253` | 舵机 ID |
| `off` | `uint8_t` | `0` 或 `1` | `0`=常亮，`1`=常灭 |

---

#### `LX824_LedErrorWrite` — LED 故障报警（Cmd 35）

```c
err_t LX824_LedErrorWrite(LX824_t *self, uint8_t id, uint8_t fault);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** |
| `id` | `uint8_t` | `0 ~ 253` | 舵机 ID |
| `fault` | `uint8_t` | `0 ~ 7` | LED 故障报警值 |

---

### 读指令 API 详解

所有读指令使用**阻塞式请求-应答**模式：
1. 通过 `self->uart_send_` 发送读指令帧到舵机
2. 通过 `self->uart_receive_` 等待舵机应答
3. 逐字节推进应答解析状态机
4. 匹配到对应 Cmd 的合法应答后返回

**必须在 FreeRTOS 任务上下文中调用**（内部使用 `xTaskNotifyWait`），超时返回 `TIMEOUT`。

---

#### `LX824_MoveTimeRead` — 读角度+时间（Cmd 2）

```c
err_t LX824_MoveTimeRead(LX824_t *self, uint8_t id, uint16_t *angle, uint16_t *time_ms);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** — 通过它发读指令并收应答 |
| `id` | `uint8_t` | `0 ~ 253` | 舵机 ID |
| `angle` | `uint16_t *` | 输出 | 当前目标角度 |
| `time_ms` | `uint16_t *` | 输出 | 当前运动时间 |

---

#### `LX824_MoveTimeWaitRead` — 读预设角度+时间（Cmd 8）

```c
err_t LX824_MoveTimeWaitRead(LX824_t *self, uint8_t id, uint16_t *angle, uint16_t *time_ms);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| **`self`** | `LX824_t *` | **总线对象** |
| `id` | `uint8_t` | 舵机 ID |
| `angle` | `uint16_t *` | 输出：预设目标角度 |
| `time_ms` | `uint16_t *` | 输出：预设运动时间 |

---

#### `LX824_IdRead` — 读舵机 ID（Cmd 14）

```c
err_t LX824_IdRead(LX824_t *self, uint8_t id, uint8_t *servo_id);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| **`self`** | `LX824_t *` | **总线对象** — 必须在同一任务中调用 |
| `id` | `uint8_t` | 待查询舵机 ID（已知时）或 `0xFE`（广播读，限总线单舵机） |
| `servo_id` | `uint8_t *` | 输出：舵机返回的 ID |

---

#### `LX824_AngleOffsetRead` — 读偏差（Cmd 19）

```c
err_t LX824_AngleOffsetRead(LX824_t *self, uint8_t id, int8_t *offset);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| **`self`** | `LX824_t *` | **总线对象** |
| `id` | `uint8_t` | 舵机 ID |
| `offset` | `int8_t *` | 输出：偏差值（-125 ~ +125） |

---

#### `LX824_AngleLimitRead` — 读角度限制（Cmd 21）

```c
err_t LX824_AngleLimitRead(LX824_t *self, uint8_t id, uint16_t *min_angle, uint16_t *max_angle);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| **`self`** | `LX824_t *` | **总线对象** |
| `id` | `uint8_t` | 舵机 ID |
| `min_angle` | `uint16_t *` | 输出：角度下限 |
| `max_angle` | `uint16_t *` | 输出：角度上限 |

---

#### `LX824_VinLimitRead` — 读电压限制（Cmd 23）

```c
err_t LX824_VinLimitRead(LX824_t *self, uint8_t id, uint16_t *min_mv, uint16_t *max_mv);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| **`self`** | `LX824_t *` | **总线对象** |
| `id` | `uint8_t` | 舵机 ID |
| `min_mv` | `uint16_t *` | 输出：最低电压限制（mV） |
| `max_mv` | `uint16_t *` | 输出：最高电压限制（mV） |

---

#### `LX824_TempMaxLimitRead` — 读温度限制（Cmd 25）

```c
err_t LX824_TempMaxLimitRead(LX824_t *self, uint8_t id, uint8_t *temp_c);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| **`self`** | `LX824_t *` | **总线对象** |
| `id` | `uint8_t` | 舵机 ID |
| `temp_c` | `uint8_t *` | 输出：最高温度限制（摄氏度） |

---

#### `LX824_TempRead` — 读实时温度（Cmd 26）

```c
err_t LX824_TempRead(LX824_t *self, uint8_t id, uint8_t *temp_c);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| **`self`** | `LX824_t *` | **总线对象** |
| `id` | `uint8_t` | 舵机 ID |
| `temp_c` | `uint8_t *` | 输出：舵机内部当前温度（摄氏度） |

---

#### `LX824_VinRead` — 读实时电压（Cmd 27）

```c
err_t LX824_VinRead(LX824_t *self, uint8_t id, uint16_t *mv);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| **`self`** | `LX824_t *` | **总线对象** |
| `id` | `uint8_t` | 舵机 ID |
| `mv` | `uint16_t *` | 输出：舵机供电电压（毫伏） |

---

#### `LX824_PosRead` — 读实时位置（Cmd 28）

```c
err_t LX824_PosRead(LX824_t *self, uint8_t id, int16_t *pos);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** |
| `id` | `uint8_t` | `0 ~ 253` | 舵机 ID |
| `pos` | `int16_t *` | `-1000 ~ 1000` | 输出：舵机当前角度位置（可负） |

---

#### `LX824_OrMotorModeRead` — 读模式+速度（Cmd 30）

```c
err_t LX824_OrMotorModeRead(LX824_t *self, uint8_t id, uint8_t *mode, int16_t *speed);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| **`self`** | `LX824_t *` | **总线对象** |
| `id` | `uint8_t` | 舵机 ID |
| `mode` | `uint8_t *` | 输出：当前模式（0=位置控制，1=电机控制） |
| `speed` | `int16_t *` | 输出：电机模式下当前速度 |

---

#### `LX824_LoadOrUnloadRead` — 读装载状态（Cmd 32）

```c
err_t LX824_LoadOrUnloadRead(LX824_t *self, uint8_t id, uint8_t *load);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| **`self`** | `LX824_t *` | **总线对象** |
| `id` | `uint8_t` | 舵机 ID |
| `load` | `uint8_t *` | 输出：0=卸载，1=装载 |

---

#### `LX824_LedCtrlRead` — 读 LED 状态（Cmd 34）

```c
err_t LX824_LedCtrlRead(LX824_t *self, uint8_t id, uint8_t *off);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| **`self`** | `LX824_t *` | **总线对象** |
| `id` | `uint8_t` | 舵机 ID |
| `off` | `uint8_t *` | 输出：0=常亮，1=常灭 |

---

#### `LX824_LedErrorRead` — 读故障值（Cmd 36）

```c
err_t LX824_LedErrorRead(LX824_t *self, uint8_t id, uint8_t *fault);
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| **`self`** | `LX824_t *` | 已初始化并 Start | **总线对象** — 所有舵机通信的入口 |
| `id` | `uint8_t` | `0 ~ 253` | 舵机 ID |
| `fault` | `uint8_t *` | `0 ~ 7` | 输出：LED 故障报警值 |

---

### 完整使用示例

```c
#include "lx824.h"

LX824_t *lx824 = NULL;  // 全局指针，供其他业务模块复用舵机总线

void lx824_task(void *argument) {
    /* ========== 第 1 步：创建总线对象实例（静态存储，生命周期 = 任务生命周期） ========== */
    static LX824_t lx824_instance;

    /* ========== 第 2 步：初始化 —— 将 self 绑定到 USART1 硬件 ========== */
    //  LX824_Init() 内部将 self->uart_send_ 和 self->uart_receive_
    //  绑定到 &huart1，之后所有 API 都通过 self 操作这条总线
    err_t status = LX824_Init(&lx824_instance, &huart1);
    lx824 = &lx824_instance;  // 暴露全局指针

    /* ========== 第 3 步：注册任务句柄 —— ISR 通过它通知任务 ========== */
    lx824->thread_alert = xTaskGetCurrentTaskHandle();

    /* ========== 第 4 步：启动 DMA ========== */
    if (status == OK) status = LX824_Start(lx824);  // 启动 self->uart_send_ 和 self->uart_receive_
    ASSERT(status == OK);
    if (status != OK) vTaskDelete(NULL);

    // 示例：通过 self 总线发送指令给舵机 1
    LX824_MoveTimeWrite(lx824,     // ← self：指定往这条 UART 总线发
                        1,         //     舵机 ID
                        750,       //     目标角度（750/1000 × 240° = 180°）
                        2000);     //     运动时间 2 秒

    for (;;) {
        LX824_Update(lx824, 20);   // 排空 RX FIFO（维持应答解析状态机）

        // 每 100ms 通过 self 总线读取舵机 1 的位置
        int16_t pos;
        err_t err = LX824_PosRead(lx824,  // ← self：指定从这个总线读
                                  1,      //     舵机 ID
                                  &pos);  //     输出：角度位置
        if (err == OK) {
            // pos = -1000 ~ +1000
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

> ⚠️ **关于 `LX824_t *self`**：该结构体包含 TX 双缓冲发送通道和 RX 循环 DMA 接收通道，**任何舵机指令都必须通过 `self` 才能发送到物理总线上**。

---

## 13. VOFA+ 上位机调试协议

**文件**：`modules/Vofa/vofa.h` / `vofa.c`

### 上行协议（firewater）

发送浮点数数组，尾部追加固定帧尾 `00 00 80 7F`（IEEE 754 +0.0 的十六进制，VOFA+ 用它识别帧结束）。

```c
float data[3] = {1.0f, 2.0f, 3.0f};
Vofa_Send(&vofa, data, 3);
// 实际发送：01 00 00 40 00 00 40 40 00 00 40 40 00 00 80 7F
//          [--CH1--] [--CH2--] [--CH3--] [---tail---]
```

### 下行协议（ASCII）

接收格式：`name=value!`

示例：
```
speed=100!
angle=30!
mit=0.5!
```

---

### `Vofa_t` — VOFA 对象

```c
typedef struct {
    TaskHandle_t thread_alert;
    STM32UARTDoubleBufTx_t uart_send_;            // TX 发送通道
    STM32UART_t uart_receive_;                     // RX 接收通道
    err_t tx_init_error_;
    err_t rx_init_error_;
    Vofa_CommandCallback_t command_callback_;      // 命令回调
} Vofa_t;
```

---

### `Vofa_Init` — 初始化（绑定 UART 硬件）

```c
err_t Vofa_Init(Vofa_t *self, UART_HandleTypeDef *uart_handle);
```

- 第一个参数 `*self` —— `Vofa_t` 结构体指针，该结构体包含 TX 双缓冲 DMA 发送通道 (`uart_send_`)、RX 循环 DMA 接收通道 (`uart_receive_`)、初始化错误码 (`tx_init_error_`/`rx_init_error_`) 和命令回调 (`command_callback_`)，Init 时绑定到指定 UART 硬件
- 第二个参数 `uart_handle` —— HAL UART 句柄，指定使用哪个串口与上位机通信

| 返回值 | 含义 |
|--------|------|
| `OK` | 初始化成功 |
| 其他 | TX 或 RX BSP 初始化失败的错误码 |

---

### `Vofa_Start` — 启动 TX/RX DMA

```c
err_t Vofa_Start(Vofa_t *self);
```

- 第一个参数 `*self` —— `Vofa_t` 结构体指针，该结构体包含 TX 双缓冲 DMA 发送通道和 RX 循环 DMA 接收通道，Start 时启动这两个 DMA 通道开始工作

| 返回值 | 含义 |
|--------|------|
| `OK` | 启动成功 |
| 其他 | TX 或 RX 的 DMA 初始化错误码（通过 `tx_init_error_` / `rx_init_error_` 返回） |

---

### `Vofa_Update` — 任务周期处理（排空 RX FIFO + 解析 ASCII 命令）

```c
void Vofa_Update(Vofa_t *self, uint32_t timeout_ms);
```

- 第一个参数 `*self` —— `Vofa_t` 结构体指针，该结构体通过 `uart_receive_` 接收上位机下发的 ASCII 命令，Update 时排空 RX FIFO 并解析 `name=value!` 格式命令
- 第二个参数 `timeout_ms` —— 等待 ISR 通知的超时时间（ms），超过此时间没有新数据则继续执行

**处理流程**：
1. 检查并消费 RX FIFO 溢出标志 → 溢出时清空 FIFO 和命令缓冲区
2. 批量取出 RX FIFO 数据
3. 逐字节喂给命令解析器（查找 `name=value!` 格式）
4. 解析成功后更新参数表并调用 `command_callback_`

---

### `Vofa_Send` — 发送 firewater 数据帧

```c
err_t Vofa_Send(Vofa_t *self, const float *data, size_t size);
```

- 第一个参数 `*self` —— `Vofa_t` 结构体指针，该结构体包含 TX 双缓冲 DMA 发送通道 (`uart_send_`)，Send 时通过该通道发出 firewater 格式的浮点数据帧到上位机
- 第二个参数 `*data` —— 待发送的 float 数组首地址，指向要上传给 VOFA+ 显示的数据
- 第三个参数 `size` —— float 个数，范围 `1 ~ 6`（最多 6 个 float 通道）

**帧格式**：`[float1(4B)][float2(4B)]...[floatN(4B)][00 00 80 7F(4B)]`

---

### `Vofa_GetParameter` — 按名称读取参数

```c
bool Vofa_GetParameter(const char *name, float *value);
```

- 第一个参数 `*name` —— 参数名称字符串，填 `"speed"` / `"angle"` / `"mit"`
- 第二个参数 `*value` —— 输出参数值的 float 指针，填 `&my_var`

| 返回值 | 含义 |
|--------|------|
| `true` | 参数存在且至少收到过一次有效值，`*value` 已写入 |
| `false` | 参数不存在或从未收到过 |

---

### `Vofa_GetSpeed` / `Vofa_GetAngle` — 快捷读取

```c
bool Vofa_GetSpeed(float *speed);
bool Vofa_GetAngle(float *angle);
```

- 第一个参数 `*speed` / `*angle` —— 输出参数值的 float 指针，填 `&my_speed` / `&my_angle`

等价于 `Vofa_GetParameter("speed", speed)` 和 `Vofa_GetParameter("angle", angle)`。

---

### `Vofa_SetCommandCallback` — 设置命令回调

```c
void Vofa_SetCommandCallback(Vofa_t *self, Vofa_CommandCallback_t callback);
```

- 第一个参数 `*self` —— `Vofa_t` 结构体指针，该结构体包含命令回调函数指针 (`command_callback_`)，调用此函数将 callback 注册到该结构体中，当上位机下发 `name=value!` 命令解析成功后调用
- 第二个参数 `callback` —— 命令回调函数指针，签名 `void (*)(const char *name, float value)`

`name=value!` 命令解析成功后调用。`name` 生命周期仅在回调执行期间有效，不能异步保存指针。

---

### 预定义参数表

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `speed` | 0.0f | 速度设定值，上位机通过 `speed=100!` 写入 |
| `angle` | 0.0f | 角度设定值，上位机通过 `angle=30!` 写入 |
| `mit` | 0.0f | MIT 模式参数，上位机通过 `mit=0.5!` 写入 |

参数表可通过修改 `vofa.c` 中的 `vofa_parameters[]` 静态数组扩展。

---

### 命令回调扩展点

任务层提供弱函数，用户可在其他 .c 中提供强定义覆盖：

```c
// 默认实现（弱符号）—— 将命令值写入上行数据数组
__attribute__((weak)) void VofaTask_OnCommand(const char *name, float value) {
    if (strcmp(name, "speed") == 0) data[0] = value;
    else if (strcmp(name, "angle") == 0) data[1] = value;
    else if (strcmp(name, "mit") == 0) data[2] = value;
}
```

**扩展示例**（在其他 .c 中提供同名强定义，不需要修改 VOFA 驱动）：

```c
void VofaTask_OnCommand(const char *name, float value) {
    if (strcmp(name, "kp") == 0) pid.Kp = value;
    else if (strcmp(name, "ki") == 0) pid.Ki = value;
    else if (strcmp(name, "kd") == 0) pid.Kd = value;
}
```

---

### 完整任务示例

```c
#include "vofa.h"

float data[3] = {0.0f, 0.0f, 0.0f};  // 上行 3 通道数据

static void OnVofaCommand(const char *name, float value) {
    VofaTask_OnCommand(name, value);
}

void vofa_task(void *argument) {
    static Vofa_t vofa;

    // 初始化：*self=&vofa 绑定到 huart6，*uart_handle=&huart6
    err_t status = Vofa_Init(&vofa, &huart6);
    vofa.thread_alert = xTaskGetCurrentTaskHandle();
    Vofa_SetCommandCallback(&vofa, OnVofaCommand);

    if (status == OK) status = Vofa_Start(&vofa);
    ASSERT(status == OK);
    if (status != OK) vTaskDelete(NULL);

    for (;;) {
        data[0] = actual_speed;   // 通道 1：实际转速
        data[1] = target_speed;   // 通道 2：目标转速
        data[2] = pid_output;     // 通道 3：PID 输出
        Vofa_Send(&vofa, data, 3);   // 通过 *self=&vofa 的 TX DMA 发出

        Vofa_Update(&vofa, 10);      // 通过 *self=&vofa 的 RX DMA 接收命令
    }
}
```

---

## 14. Game RoboMaster 裁判系统

**文件**：`modules/game/game.h` / `game.c`

### 协议版本

RoboMaster 2026 赛季通信协议 V1.1.0（20251217）

### 帧格式

```
[SOF=0xA5][DATA_LENGTH(2)][SEQ(1)][CRC8(1)][CMD_ID(2)][DATA(N)][CRC16(2)]
```

| 段 | 长度 | 说明 |
|----|------|------|
| SOF | 1 | 起始字节，固定 `0xA5` |
| DATA_LENGTH | 2 | data 长度（小端） |
| SEQ | 1 | 包序号 |
| CRC8 | 1 | 帧头 CRC8 |
| CMD_ID | 2 | 命令码 |
| DATA | N | 有效载荷 |
| CRC16 | 2 | 全帧 CRC16（小端） |

---

### `Game_t` — 裁判系统接收对象

```c
typedef struct {
    TaskHandle_t thread_alert;           // 任务通知句柄
    CommuniCateTypeDef *target;          // 解析结果写入目标
    STM32UART_t uart_receive_;           // UART DMA 接收通道
    err_t init_error_;
    uint8_t rx_fifo[GAME_RX_FIFO_LEN];  // ISR→任务 FIFO（512 字节）
    volatile uint16_t rx_head, rx_tail;
    volatile bool rx_fifo_overflowed;
    uint32_t rx_fifo_overflow_count;
    uint8_t stash[GAME_STASH_CAPACITY];  // 半包暂存
    size_t stash_len;
} Game_t;
```

---

### `CommuniCateTypeDef` — 全局裁判数据

```c
typedef struct RM_PACKED {
    data_t judge_data;
    struct robot_interaction_data_t robot_interaction_data;
    struct custom_robot_data_t custom_robot_data;
    struct map_command_t map_command;
    struct remote_control_t remote_control_data;
} CommuniCateTypeDef;

extern CommuniCateTypeDef custom_robot_data;
```

---

### `Game_Init` — 初始化（绑定 UART 硬件和解析目标）

```c
err_t Game_Init(Game_t *self, UART_HandleTypeDef *uart_handle,
                CommuniCateTypeDef *target);
```

- 第一个参数 `*self` —— `Game_t` 结构体指针，该结构体包含 UART DMA 接收通道 (`uart_receive_`)、环形 FIFO (`rx_fifo`)、粘包暂存缓冲 (`stash`) 和解析目标指针 (`target`)，Init 时清零所有字段并绑定到指定 UART 外设
- 第二个参数 `uart_handle` —— HAL UART 句柄，指定使用哪个串口接收裁判系统数据，通常 `&huart3`
- 第三个参数 `*target` —— `CommuniCateTypeDef` 结构体指针，解析完成的数据帧写入此目标，通常为 `&custom_robot_data`

---

### `Game_Start` — 启动接收

```c
err_t Game_Start(Game_t *self);
```

- 第一个参数 `*self` —— `Game_t` 结构体指针，该结构体包含 UART 接收通道 (`uart_receive_`)、FIFO 读写指针和暂存状态，Start 时复位这些状态并启动 UART 循环 DMA 接收

---

### `Game_Update` — 任务周期处理

```c
void Game_Update(Game_t *self, uint32_t timeout_ms);
```

- 第一个参数 `*self` —— `Game_t` 结构体指针，该结构体包含 FIFO、暂存缓冲和目标数据指针，Update 时等待 ISR 通知，从 FIFO 批量取出字节并喂给粘包解析状态机
- 第二个参数 `timeout_ms` —— 等待 ISR 通知的超时时间（ms）

**处理流程**：`xTaskNotifyWait` 等待 → 检查 FIFO 溢出（溢出时清空） → 批量取出字节 → `Game_FeedBytes` 做帧同步和校验 → 再次检查溢出

---

### `remote_control_data_init` — 清零全局裁判数据

```c
void remote_control_data_init(void);
```

**说明**：在 `Game_Init` 前调用，`memset` 清零全局 `custom_robot_data` 结构体

---

### `remote_process` — 帧解析（内部函数）

```c
void remote_process(uint8_t *data, CommuniCateTypeDef *custom_robot_data);
```

- 第一个参数 `*data` —— 完整的裁判系统数据帧
- 第二个参数 `*custom_robot_data` —— 解析结果的写入目标

**流程**：校验帧头 `SOF == 0xA5` → 校验 CRC8 → 校验 CRC16 → 按 `CMD_ID` 通过 `COPY_PAYLOAD` 宏分发到对应字段

---

### 关键数据字段

| 命令 ID | 结构体 | 发送频率 |
|---------|--------|---------|
| `GAME_STATE_ID` (0x0001) | `game_status_t` | 1Hz |
| `GAME_RESULT_ID` (0x0002) | `game_result_t` | 结束触发 |
| `GAME_ROBOT_ID` (0x0003) | `game_robot_hp_t` | 3Hz |
| `EVENT_DATA_ID` (0x0101) | `event_data_t` | 1Hz |
| `JUDGE_WARN_ID` (0x0104) | `referee_warning_t` | 1Hz |
| `ROBOT_STATUS_ID` (0x0201) | `robot_status_t` | 10Hz |
| `POWER_HEAR_ID` (0x0202) | `power_heat_data_t` | 50Hz |
| `ROBOT_POS_ID` (0x0203) | `robot_pos_t` | 1~10Hz |
| `BUFF_ID` (0x0204) | `buff_t` | 3Hz |
| `SHOOT_ID` (0x0207) | `shoot_data_t` | 事件触发 |
| `SHOOT_ALLOW_ID` (0x0208) | `projectile_allowance_t` | 10Hz |
| `CUSTOM_CONTROLLER_ID` (0x0302) | `custom_robot_data_t` | 30Hz |
| `remote_control_ID` (0x0304) | `remote_control_t` | 30Hz |

---

### 使用示例

```c
#include "game.h"

Game_t *game = NULL;

void game_task(void *argument) {
    static Game_t game_instance;

    remote_control_data_init();
    err_t status = Game_Init(&game_instance, &huart3, &custom_robot_data);
    game = &game_instance;
    game->thread_alert = xTaskGetCurrentTaskHandle();

    if (status == OK) status = Game_Start(game);
    ASSERT(status == OK);
    if (status != OK) vTaskDelete(NULL);

    for (;;) {
        Game_Update(game, 10);

        // 读取血量
        uint16_t hp = custom_robot_data.judge_data.robot_status.current_hp;
        uint16_t max_hp = custom_robot_data.judge_data.robot_status.maximum_hp;

        // 读取底盘功率限制
        uint16_t power_limit = custom_robot_data.judge_data.robot_status.chassis_power_limit;

        // 读取位置
        float pos_x = custom_robot_data.judge_data.robot_pos.x;
        float pos_y = custom_robot_data.judge_data.robot_pos.y;
    }
}
```

---

## 15. 增量式 PID 控制器

**文件**：`arithmetic/pid/pid_incremental/pid_incremental.h` / `.c`

### 算法公式

```
Δu(k) = Kp·[e(k) - e(k-1)] + Ki·e(k)·dt + Kd·[e(k) - 2e(k-1) + e(k-2)]/dt
 u(k) = u(k-1) + Δu(k) + ΔFF(k)
```

---

### `PIDInstance_jie` — PID 运行实例

```c
typedef struct {
    // === 基础参数 ===
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
    float MaxOut;      // 输出限幅（绝对值）
    float DeadBand;    // 死区（误差绝对值小于此值禁用 PID）
    float dt;          // 采样周期（秒），USE_TIME 时自动测量

    uint32_t DWT_CNT;  // DWT 时间戳

    // === 改进标志 ===
    uint32_t Improve;  // 改进功能位或组合

    // === 积分改进参数 ===
    float IntegralLimit; // 积分限幅
    float CoefA;         // 变速积分上限系数
    float CoefB;         // 变速积分下限系数

    // === 滤波器 ===
    float Derivative_LPF_RC;  // 微分低通 RC 时间常数
    float Output_LPF_RC;      // 输出低通 RC 时间常数

    // === 前馈 ===
    float Kf;              // 前馈系数
    float FeedForward_Max; // 前馈限幅

    // === 运行时缓存（不应手动修改） ===
    float actual, Last_actual;
    float target, Last_target;
    float Err, Last_Err, Err_Pre;
    float Pout, Iout, Dout;
    float Output, Output_Inc, Last_Output, Last_Dout;
    float Output_Inc_max;
} PIDInstance_jie;

```
---

### `PID_Improvement_jie_e` — 改进功能标志

```c
typedef enum {
    PID_IMPROVE_NONE              = 0b00000000,
    PID_Integral_Limit            = 0b00000001,  // 积分限幅
    PID_Derivative_On_Measurement = 0b00000010,  // 微分先行
    PID_Trapezoid_Intergral       = 0b00000100,  // 梯形积分
    PID_FeedForward               = 0b00001000,  // 前馈控制
    PID_OutputFilter              = 0b00010000,  // 输出滤波
    PID_ChangingIntegrationRate   = 0b00100000,  // 变速积分
    PID_DerivativeFilter          = 0b01000000,  // 微分低通滤波
} PID_Improvement_jie_e：
```

---

### `PID_Init_Params_jie` — 参数化初始化（配置 PID 参数和改进标志）

```c
void PID_Init_Params_jie(PIDInstance_jie *pid,
                         float kp, float ki, float kd,
                         float dt,
                         float max_output, float max_integral,
                         float deadzone,
                         PID_Improvement_jie_e improve_flags);
```

- 第一个参数 `*pid` —— `PIDInstance_jie` 结构体指针，该结构体包含 PID 三参数 (Kp/Ki/Kd)、采样周期 (dt)、限幅值 (MaxOut/IntegralLimit)、死区 (DeadBand)、改进标志 (Improve) 和所有运行时状态，Init 时清零结构体并写入配置参数
- 第二个参数 `kp` —— 比例系数，控制误差响应速度，≥ 0
- 第三个参数 `ki` —— 积分系数，用于消除稳态误差，≥ 0
- 第四个参数 `kd` —— 微分系数，用于抑制超调和改善动态响应，≥ 0
- 第五个参数 `dt` —— 采样周期（秒），> 0；若定义了 `USE_TIME` 则由 DWT 自动测量
- 第六个参数 `max_output` —— 输出限幅绝对值，> 0
- 第七个参数 `max_integral` —— 积分限幅值，> 0
- 第八个参数 `deadzone` —— 死区范围，误差绝对值小于此值时 PID 不输出，≥ 0
- 第九个参数 `improve_flags` —— 改进功能使能标志位，按位或组合多个 `PID_Improvement_jie_e`

**内部自动设置的默认值**：
- `Derivative_LPF_RC` = `0.1f`，`Output_LPF_RC` = `0.05f`
- `CoefA` = `max_output * 0.5f`，`CoefB` = `max_output * 0.1f`
- `Kf` = `0.0f`（前馈默认禁用），`FeedForward_Max` = `max_output * 0.5f`

---

### `PID_Calculate_jie` — 计算增量式 PID 输出

```c
float PID_Calculate_jie(PIDInstance_jie *pid, float target, float actual);
```

- 第一个参数 `*pid` —— `PIDInstance_jie` 结构体指针，该结构体保存了全部 PID 参数、改进标志和运行时状态（误差链、输出历史），每调用一次 Calculate 就更新一次这些内部字段
- 第二个参数 `target` —— 设定值（期望达到的目标值）
- 第三个参数 `actual` —— 反馈值（传感器/编码器测量到的实际值）

| 返回值 | 含义 |
|--------|------|
| `float` | 位置式 PID 总输出（绝对值），已叠加之前的输出值 |

**计算公式**：`Δu = Kp[e(k)-e(k-1)] + Ki·e(k)·dt + Kd[e(k)-2e(k-1)+e(k-2)]/dt`，`u(k) = u(k-1) + Δu`

**计算流程**：

```
1. 更新 dt（USE_TIME 时用 DWT 自动测量）
2. 保存历史值，计算误差
3. 死区判断 → 死区内清零增量和积分，保持输出
4. 比例增量：ΔP = Kp * (e(k) - e(k-1))
5. 积分增量：ΔI = Ki * e(k) * dt（基础）
   - 启用梯形积分：使用 (e(k) + e(k-1))/2 * dt
   - 启用变速积分：按误差大小线性削弱
   - 启用积分限幅：限幅 Iout
6. 微分增量：ΔD（标准或微分先行）
   - 标准：Kd * (e(k) - 2e(k-1) + e(k-2)) / dt
   - 微分先行：Kd * (actual(k-1) - actual(k)) / dt
   - 启用微分滤波：一阶低通
7. 前馈：ΔFF = Kf * (target(k) - target(k-1)) / dt
8. 总增量 = ΔP + ΔI + ΔD + ΔFF
9. 总输出 = Last_Output + 总增量
10. 输出滤波 + 输出限幅
11. 返回 Output
```

---

### `PID_Get_Increment_jie` — 获取增量

```c
float PID_Get_Increment_jie(PIDInstance_jie *pid);
```

- 第一个参数 `*pid` —— `PIDInstance_jie` 结构体指针，读取其 `Output_Inc` 字段

| 返回值 | 含义 |
|--------|------|
| `float` | 最近一次 `PID_Calculate_jie` 计算的输出增量 Δu |

---

### `PID_Reset_jie` — 复位（清零运行时状态，保留配置参数）

```c
void PID_Reset_jie(PIDInstance_jie *pid);
```

- 第一个参数 `*pid` —— `PIDInstance_jie` 结构体指针，清零其所有运行时状态（Err/Output/历史值等），保留 Kp/Ki/Kd、限幅、Improve 等配置参数

---

### 完整使用示例

```c
PIDInstance_jie speed_pid;

void init_speed_pid(void) {
    PID_Init_Params_jie(&speed_pid,
        10.0f,       // Kp = 10
        0.5f,        // Ki = 0.5
        0.1f,        // Kd = 0.1
        0.01f,       // dt = 10ms
        1000.0f,     // MaxOut = 1000
        500.0f,      // IntegralLimit = 500
        0.5f,        // DeadBand = 0.5
        PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter
    );
}

void control_loop(void) {
    float target = 1000.0f;   // 目标转速
    float actual = get_speed_from_encoder();
    float output = PID_Calculate_jie(&speed_pid, target, actual);
    set_motor_current(output);
}
```

---

## 16. 位置式 PID 控制器

**文件**：`arithmetic/pid/pid_location/pid_location.h` / `.c`

### 算法公式

```
u(k) = Kp·e(k) + Ki·Σe(k)·dt + Kd·[e(k) - e(k-1)]/dt
```

---

### `PIDInstance` — PID 运行实例

```c
typedef struct {
    float Kp, Ki, Kd, MaxOut, DeadBand;
    PID_Improvement_e Improve;
    float IntegralLimit, CoefA, CoefB;
    float Output_LPF_RC, Derivative_LPF_RC;

    float Measure, Last_Measure, Err, Last_Err;
    float Pout, Iout, Dout, ITerm;
    float Output, Last_Output, Last_Dout;
    float Ref;
    float dt;
    PID_ErrorHandler_t ERRORHandler;
} PIDInstance;
```


---

### `PID_Improvement_e` — 改进标志

```c
typedef enum {
    PID_IMPROVE_NONE              = 0b00000000,
    PID_Integral_Limit            = 0b00000001,  // 积分限幅
    PID_Derivative_On_Measurement = 0b00000010,  // 微分先行
    PID_Trapezoid_Intergral       = 0b00000100,  // 梯形积分
    PID_Proportional_On_Measurement = 0b00001000, // 比例项测量值
    PID_OutputFilter              = 0b00010000,  // 输出滤波
    PID_ChangingIntegrationRate   = 0b00100000,  // 变速积分
    PID_DerivativeFilter          = 0b01000000,  // 微分滤波
    PID_ErrorHandle               = 0b10000000,  // 堵转检测
} PID_Improvement_e;
```

---

### `PID_Init_Config_s` — 初始化配置

```c
typedef struct {
    float Kp, Ki, Kd, MaxOut, DeadBand;
    PID_Improvement_e Improve;
    float IntegralLimit;
    float CoefA, CoefB;
    float Output_LPF_RC, Derivative_LPF_RC;
} PID_Init_Config_s;
```

---

### `PIDInit` — 初始化（栈上，传入配置结构体）

```c
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);
```

- 第一个参数 `*pid` —— `PIDInstance` 结构体指针，该结构体包含 PID 三参数、限幅值、改进标志和所有运行时状态，Init 时清零运行时状态并从 config 复制配置
- 第二个参数 `*config` —— `PID_Init_Config_s` 结构体指针，包含 Kp/Ki/Kd、MaxOut、DeadBand、Improve、IntegralLimit、CoefA/B、滤波器 RC 等配置参数

---

### `PIDRegister` — 初始化（堆上动态分配）

```c
PIDInstance *PIDRegister(PID_Init_Config_s *config);
```

- 第一个参数 `*config` —— `PID_Init_Config_s` 结构体指针，包含 PID 配置参数

| 返回值 | 含义 |
|--------|------|
| `PIDInstance *` | 成功：指向堆上分配的 PID 实例指针 |
| `NULL` | 失败：`malloc` 分配失败 |

---

### `PIDCalculate` — 计算位置式 PID 输出

```c
float PIDCalculate(PIDInstance *pid, float measure, float ref);
```

- 第一个参数 `*pid` —— `PIDInstance` 结构体指针，该结构体保存了所有 PID 参数、改进标志、堵转检测状态和运行时误差/输出历史，每调用一次 Calculate 就更新一次这些内部字段
- 第二个参数 `measure` —— 反馈值（传感器/编码器测量到的实际值）
- 第三个参数 `ref` —— 设定值（期望达到的目标值）

| 返回值 | 含义 |
|--------|------|
| `float` | 位置式 PID 总输出 |

**计算公式**：`u(k) = Kp·e(k) + Ki·Σe(k)·dt + Kd·[e(k) - e(k-1)]/dt`

**计算流程**：

```
1. 如果启用 ErrorHandle：检查堵转状态
2. 用 DWT 更新 dt
3. 计算误差：Err = ref - measure
4. 堵转故障时反向 ref
5. 死区判断 → 死区内输出=0，清空积分
6. Pout = Kp * Err（或启用 POM 时基于测量值变化）
7. ITerm = Ki * Err * dt（基础）
   - 梯形积分 / 变速积分 / 积分限幅
8. Iout += ITerm
9. Dout = Kd * (Err - Last_Err) / dt
   - 或微分先行：Kd * (Last_Measure - Measure) / dt
   - 微分滤波
10. Output = Pout + Iout + Dout
11. 输出滤波 + 输出限幅
12. 保存历史值
13. 返回 Output
```

**堵转检测**（`PID_ErrorHandle`）：
- 当 `|Output| > 0.001*MaxOut` 且 `|ref - last_measure| / |ref| > 0.95` 时，计数 +1
- 计数 > 500 时设置 `PID_MOTOR_BLOCKED_ERROR`，自动反向 ref 以释放堵转

---

### 使用示例

```c
// 方法 1：栈上初始化
PIDInstance position_pid;
PID_Init_Config_s cfg = {
    .Kp = 5.0f,
    .Ki = 0.1f,
    .Kd = 0.5f,
    .MaxOut = 3000.0f,
    .DeadBand = 1.0f,
    .Improve = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_OutputFilter,
    .IntegralLimit = 1000.0f,
    .Output_LPF_RC = 0.05f,
    .Derivative_LPF_RC = 0.1f,
};
PIDInit(&position_pid, &cfg);

// 方法 2：堆上初始化（动态分配）
PIDInstance *pid = PIDRegister(&cfg);

// 控制循环
float output = PIDCalculate(&position_pid, encoder_angle, target_angle);
motor_set_output(output);
```

---

## 17. 裁判系统 CRC 校验

**文件**：`arithmetic/referee/crc_ref.h` / `crc_ref.c`

提供查表法 CRC8 和 CRC16，用于 RoboMaster 裁判系统帧校验。

---

### CRC8

#### `Get_CRC8_Check_Sum` — 计算 CRC8

```c
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength, uint8_t ucCRC8);
```

| 参数 | 说明 |
|------|------|
| `pchMessage` | 数据缓冲区 |
| `dwLength` | 数据长度 |
| `ucCRC8` | 种子值（首次传 `0xff`） |

| 返回值 |
|--------|
| 8 位 CRC 值 |

**分段计算示例**：

```c
uint8_t crc = Get_CRC8_Check_Sum(buf, 10, 0xff);
crc = Get_CRC8_Check_Sum(buf + 10, 5, crc);  // 续算
```

#### `Verify_CRC8_Check_Sum` — 校验 CRC8

```c
uint32_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
```

| 参数 | 说明 |
|------|------|
| `pchMessage` | 包含末尾 CRC8 字节的缓冲区 |
| `dwLength` | 包含 CRC8 的总长度 |

| 返回值 | 含义 |
|--------|------|
| `TRUE` (1) | 校验通过 |
| `FALSE` (0) | 校验失败或参数无效 |

#### `Append_CRC8_Check_Sum` — 写入 CRC8

```c
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
```

将 CRC8 计算结果写入 `pchMessage[dwLength - 1]`。CRC8 字节本身不参与计算。

---

### CRC16

#### `Get_CRC16_Check_Sum` — 计算 CRC16

```c
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
```

| 参数 | 说明 |
|------|------|
| `wCRC` | 种子值（首次传 `0xffff`） |

| 返回值 |
|--------|
| 16 位 CRC 值 |

#### `Verify_CRC16_Check_Sum` — 校验 CRC16

```c
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
```

校验倒数第 2、1 字节是否为正确的 CRC16（小端）。

| 返回值 | 含义 |
|--------|------|
| `TRUE` | 校验通过 |
| `FALSE` | 校验失败 |

#### `Append_CRC16_Check_Sum` — 写入 CRC16

```c
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
```

小端写入：`pchMessage[dwLength-2]` = 低字节，`pchMessage[dwLength-1]` = 高字节。

---

### 应用示例

```c
// 构建裁判系统帧头，假设帧头 5 字节（SOF+Length+Seq+CRC8）
uint8_t header[5];
header[0] = 0xA5;    // SOF
header[1] = 0x10;    // data length low
header[2] = 0x00;    // data length high
header[3] = 0x01;    // seq
// header[4] = CRC8 待填充
Append_CRC8_Check_Sum(header, 5);  // 自动写入 header[4]

// 构建整帧（header + cmd_id + payload + CRC16）
uint8_t frame[256];
// ... 填充 header, cmd_id, payload ...
Append_CRC16_Check_Sum(frame, total_len);
```

---

## 18. Task FreeRTOS 任务

### 任务总表

| 任务名 | 入口函数 | 外设 | 全局数据 | 建议优先级 |
|--------|---------|------|---------|-----------|
| `dr16_task` | `dr16_task()` | USART2 | `DR16_t *dr16` | 中 |
| `i6x_task` | `I6X_task()` | UART4 | `I6X_t *i6x` | 中 |
| `lx824_task` | `lx824_task()` | USART1 | `LX824_t *lx824` | 低 |
| `vofa_task` | `vofa_task()` | USART6 | `float data[3]` | 低 |
| `game_task` | `game_task()` | USART3 | `CommuniCateTypeDef custom_robot_data` | 中高 |

### 全局数据共享说明

| 符号 | 类型 | 定义位置 | 说明 |
|------|------|---------|------|
| `dr16` | `DR16_t *` | `task/dr16_task/dr16_task.c` | DR16 遥控器状态 |
| `i6x` | `I6X_t *` | `task/i6x_task/i6x_task.c` | I6X 遥控器状态 |
| `lx824` | `LX824_t *` | `task/lx824_task/lx824_task.c` | LX824 总线接口 |
| `custom_robot_data` | `CommuniCateTypeDef` | `modules/game/game.c` | 裁判系统数据 |
| `data[3]` | `float[]` | `task/vofa_task/vofa_task.c` | VOFA+ 上行数据 |

---

## 19. 构建系统

### CMake 构建命令

```bash
# Debug（启用 ASSERT）
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build

# Release
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

### 编译定义

| 定义 | 启用条件 | 作用 |
|------|---------|------|
| `MCU_DEBUG_BUILD` | Debug 配置 | 启用 `ASSERT` / `VERIFY` |

### 链接参数

```cmake
LINKER:-u,_printf_float  # 启用 printf %f 浮点输出
```

---

## 20. 快速上手指南

### 开发环境搭建

1. 安装 ARM GCC 工具链（arm-none-eabi-gcc ≥ 10.3）
2. 安装 CMake ≥ 3.22
3. 安装 STM32CubeMX（可选，用于重新生成代码）
4. （可选）安装 OpenOCD 或 JLink 用于烧录调试

### 首次编译烧录

```bash
cd diankong
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
# 使用 JLink / OpenOCD 烧录 build/jie_max.elf
```

### 添加新硬件驱动

1. 在 `bsp/` 下新建目录
2. 实现 Init/Start/Stop/GetLastError 等接口
3. 在 `bsp/CMakeLists.txt` 添加源文件和头文件路径
4. 在 `Core/Src/freertos.c` 中创建对应任务

### VOFA+ 调试

1. 打开 VOFA+ 上位机
2. 选择串口协议，波特率与 USART6 匹配
3. 选择 firewater 协议
4. 发送 `speed=100!` 调整参数
5. 观察波形

---

## 21. 常见问题与调试

### 1. 遥控器 `online_` 始终 false

| 检查项 | 措施 |
|--------|------|
| 波特率 | DR16: 100kbps 8E2 / I6X: 115200 8N1 |
| 电平 | SBUS 需反相电平转换（3.3V） |
| DMA 缓冲 | `dma_buffer_` 是否足够大（≥ 2× 帧长） |
| 任务句柄 | `thread_alert` 是否在 Start 前赋值 |

### 2. DMA FIFO 溢出

**现象**：`rx_fifo_overflow_count` 持续增长

**排查**：
- 任务优先级是否过低
- `RX_DMA_BUF_LEN` 或 `RX_FIFO_LEN` 是否过小
- 任务处理周期是否过长

### 3. ASSERT 触发死循环

**现象**：程序卡在 `verify_failed()` 的 while(1)

**排查**：在 `while(1)` 处打断点，查看调用栈，检查 ASSERT 条件

### 4. PID 调节贴士

| 步骤 | 操作 | 观察 |
|------|------|------|
| 1 | Ki=Kd=0，增大 Kp 直到轻微振荡 | 响应速度 |
| 2 | 减小 Kp 30%，增大 Kd 抑制超调 | 超调量 |
| 3 | 增大 Ki 消除稳态误差 | 稳态误差 |
| 4 | 启用改进标志逐项测试 | 单项效果 |

### 5. CAN 通信无响应

| 检查项 | 措施 |
|--------|------|
| 终端电阻 | CAN 总线两端需 120Ω 终端电阻 |
| 波特率 | CAN1/CAN2 均预设 500kbps |
| ID 匹配 | 滤波器配置是否正确 |
| 中断优先级 | `HAL_NVIC_SetPriority` 配置 |

---

> **文档版本**：v2.0（完整函数参考版）  
> **项目名称**：Diankong（电控）  
> **战队**：FHU RoboMaster 飞虎战队  
> **MCU**：STM32F405RG  
> **RTOS**：FreeRTOS (CMSIS-RTOS v2)  
> **CubeMX**：jie_max.ioc  
> **协议**：RoboMaster 2026 V1.1.0

