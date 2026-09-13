# Diankong（电控）代码使用手册 —— 完整函数参考

> **项目概述**：基于 STM32F405 + FreeRTOS 的 RoboMaster 机器人电控固件  
> **适用 MCU**：STM32F405RG  
> **RTOS**：FreeRTOS (CMSIS-RTOS v2)  
> **协议年份**：RoboMaster 2026  
> **文档版本**：v2.1（同步底盘 / 机械臂 / IMU / VT13 代码，2026-09-12）

---

## 目录

1. [项目结构总览](#1-项目结构总览)
2. [硬件资源分配](#2-硬件资源分配)
3. [Component 层 —— comp_cmd / comp_def / comp_utils](#3-component-层)
4. [Tool 层 —— convert / process 工具](#4-tool-工具层)
5. [BSP 层 — CAN 驱动 (bsp_can)](#5-bsp-can-驱动)
6. [BSP 层 — UART DMA 驱动 (bsp_uart)](#6-bsp-uart-dma-驱动)
7. [BSP 层 — PWM 驱动 (bsp_pwm)](#7-bsp-pwm-驱动)
8. [BSP 层 — I²C 驱动 (bsp_iic)](#8-bsp-iic-驱动)
9. [BSP 层 — USB CDC 驱动 (bsp_usb)](#9-bsp-usb-cdc-驱动)
10. [BSP 层 — DWT 精密定时 (bsp_dwt)](#10-bsp-dwt-精密定时)
11. [Modules 层 — DR16 遥控器 (dr16)](#11-dr16-大疆遥控器sbus)
12. [Modules 层 — I6X 遥控器 (i6x)](#12-i6x-富斯-ibus-遥控器)
13. [Modules 层 — LX824 舵机 (lx824)](#13-lx824-总线串口舵机)
14. [Modules 层 — VOFA+ 上位机 (vofa)](#14-vofa-上位机调试协议)
15. [Modules 层 — 裁判系统 (game)](#15-game-robomaster-裁判系统)
16. [Modules 层 — DJI 电机 (dj_motor)](#16-dji-电机dj_motor)
17. [Modules 层 — 达妙电机 (dm_motor)](#17-达妙电机dm_motor)
18. [Modules 层 — 达妙 IMU (dm_imu)](#18-达妙-imudm_imu)
19. [Modules 层 — VT13 图传遥控器 (vt13)](#19-vt13-图传遥控器vt13)
20. [Arithmetic 层 — 增量式 PID](#20-增量式-pid-控制器)
21. [Arithmetic 层 — 位置式 PID](#21-位置式-pid-控制器)
22. [Arithmetic 层 — CRC 校验](#22-裁判系统-crc-校验)
23. [App 层 — 底盘动力学 (chassis_dynamics)](#23-底盘动力学chassis_dynamics)
24. [Calculate 层 — 底盘与关节控制 (calculate)](#24-calculate-控制层calculate)
25. [Task 层 — FreeRTOS 任务](#25-task-freertos-任务)
26. [构建系统](#26-构建系统)
27. [快速上手指南](#27-快速上手指南)
28. [常见问题与调试](#28-常见问题与调试)

---

## 1. 项目结构总览

```
diankong/
├── Core/                    # STM32CubeMX 生成的 HAL 层
│   ├── Inc/                 #   头文件
│   └── Src/                 #   源文件（main.c 中完成 CAN/DWT 对象初始化）
├── component/               # 通用组件
│   ├── comp_cmd.h           #   err_t / cmd_rc_t 等公共类型
│   ├── comp_def.h           #   FreeRTOS 任务通知信号位
│   └── comp_utils.*         #   数学工具与断言
├── tool/                    # 基础工具库
│   ├── convert.*            #   浮点↔整数量化（DM MIT 协议）、角度宏
│   └── process.h            #   CONSTRAIN / DEADZONE / LERP / MAP 等通用宏
├── bsp/                     # 板级支持包
│   ├── bsp_can/             #   CAN 总线驱动（订阅式 RX 广播）
│   ├── bsp_iic/             #   I²C 驱动（当前已注释禁用）
│   ├── bsp_pwm/             #   PWM 驱动
│   ├── bsp_uart/            #   UART DMA 驱动（RX + FrameTx + 硬件 DBM TX）
│   ├── bsp_usb/             #   USB CDC 驱动（当前已注释禁用）
│   └── dwt/                 #   DWT 精密定时
├── modules/                 # 协议与设备驱动（DR16/I6X/LX824/Vofa/game 为 submodule）
│   ├── DR16/                #   DR16 SBUS 遥控器
│   ├── I6X/                 #   I6X iBus 遥控器
│   ├── LX824/               #   LX-824 总线舵机
│   ├── Vofa/                #   VOFA+ 调试协议
│   ├── game/                #   RoboMaster 裁判系统
│   ├── motor/dj_motor/      #   DJI 电机（M3508/M2006/GM6020，对象式管理）
│   ├── motor/dm_motor/      #   达妙电机（MIT/POS/SPD/PSI 协议）
│   ├── dm_imu/              #   达妙 IMU（CAN 与 RS485 两种链路）
│   └── vt13/                #   VT13 图传遥控器
├── arithmetic/              # 算法
│   ├── pid/pid_incremental/ #   增量式 PID
│   ├── pid/pid_location/    #   位置式 PID
│   └── referee/             #   CRC 校验
├── app/                     # 应用层动力学
│   └── chassis/             #   麦克纳姆轮运动学逆解 + 重力前馈
├── calculate/               # 控制解算层（按遥控器分变体，二选一编译）
│   ├── chassis_control_dr16/  #   底盘控制（DR16 遥控源，当前启用）
│   ├── chassis_control_vt13/  #   底盘控制（VT13 遥控源）
│   ├── joint_control_dr16/    #   六关节 DM 电机控制（DR16 遥控源，当前启用）
│   └── joint_control_vt13/    #   六关节 DM 电机控制（VT13 遥控源）
├── task/                    # FreeRTOS 任务实现
│   ├── dr16_task/           #   DR16 遥控器接收
│   ├── vt13_task/           #   VT13 遥控器接收（当前已注释禁用）
│   ├── i6x_task/            #   I6X 遥控器接收（当前已注释禁用）
│   ├── lx824_task/          #   舵机总线
│   ├── vofa_task/           #   VOFA+ 调试
│   ├── game_task/           #   裁判系统
│   ├── imu_485_task/        #   达妙 IMU（RS485）
│   ├── imu_can_task/        #   达妙 IMU（CAN）
│   ├── chassis_task/        #   底盘控制
│   ├── gimbal_task/         #   云台控制（占位，空实现）
│   └── joint_task/          #   机械臂六关节控制
├── CMakeLists.txt
├── CMakePresets.json
└── jie_max.ioc
```

> **变体选择**：`calculate/` 中 DR16 与 VT13 两套控制变体二选一参与编译（见 `calculate/CMakeLists.txt`，当前启用 DR16 变体）；`task/CMakeLists.txt` 中 `vt13_task` 与 `i6x_task` 当前被注释禁用（两者与 `game_task` 等共用串口资源），`freertos.c` 中的同名 weak 空实现兜底，任务创建调用保持不变。

---

## 2. 硬件资源分配

| 外设 | 用途 | GPIO | 通信参数 |
|------|------|------|----------|
| **USART2** | DR16 SBUS 遥控器 | PD6 (RX) | 100kbps, 8E2, 需电平转换 |
| **UART4** | I6X iBus 遥控器（任务当前禁用） | - | 115200, 8N1, 3.3V 直连 |
| **USART3** | 裁判系统 / VT13 图传链路 | - | 115200, 8N1 |
| **USART1** | LX824 总线舵机 | - | 115200, 半双工 |
| **USART6** | VOFA+ 上位机 | - | 115200, firewater 协议 |
| **UART5** | 达妙 IMU RS485 | - | 921600, 8N1 |
| **CAN1** | 关节 DM 电机 + DM IMU(CAN) | PA11(RX), PA12(TX) | 1Mbps（见下） |
| **CAN2** | 底盘 DJ 电机（M3508 × 4） | PB5(RX), PB6(TX) | 1Mbps（见下） |
| **PA7** | LED 指示灯 | PA7 | GPIO 推挽输出 |
| **PB3** | GPIO 输出（预留） | PB3 | GPIO 推挽输出 |
| **TIM10** | 1kHz 周期定时中断 | - | 见时钟配置 |

### 时钟配置

| 时钟域 | 配置 | 频率 |
|--------|------|------|
| HSE | 外部晶振 | 8 MHz |
| SYSCLK | PLL（M=4, N=168, P=2） | **168 MHz** |
| APB1 (/4) | - | 42 MHz |
| APB2 (/2) | - | 84 MHz |

### CAN 位时序（CAN1 / CAN2 相同）

| 参数 | 值 |
|------|-----|
| 预分频 | 3（APB1 42MHz / 3 = 14MHz 时间量子时钟） |
| 位时序 | 同步段 1 + BS1(6) + BS2(7) = 14 TQ |
| 波特率 | 14MHz / 14 ≈ **1 Mbps**，采样点约 50% |

### CAN 对象与过滤器分配（`main.c` USER CODE 2）

| CAN | 控制块 | Filter Bank | FIFO | 说明 |
|-----|--------|-------------|------|------|
| CAN1 | `can1_instance` | 0 | FIFO1 | 接收所有帧；关节 DM 电机 + DM IMU(CAN) |
| CAN2 | `can2_instance` | 14 | FIFO0 | 接收所有帧；底盘 M3508（CAN2 仅能使用 Bank 14-27） |

`main.c` 在调度器启动前完成：`STM32CAN_Init(can1/can2)` → `STM32CAN_ConfigFilter` → `DWT_Init(168)`。

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

### 3.2 任务通知信号位 `comp_def.h`

**文件**：`component/comp_def.h`

ISR 收到完整帧后通过 FreeRTOS 任务通知（`xTaskNotifyFromISR` + `eSetBits`）唤醒对应任务解析。每个模块占用独立的 bit 位，**不同信号不能共用同一个 bit**，否则会导致任务通知混乱：

```c
#define SIGNAL_I6X_RAW_READY    (1u << 5)   // I6X：ISR 收到完整帧
#define SIGNAL_VT13_RAW_REDY    (1u << 6)   // VT13：ISR 收到完整帧
#define SIGNAL_DR16_RAW_REDY    (1u << 7)   // DR16：ISR 收到完整帧
#define SIGNAL_LX824_RX_READY   (1u << 8)   // LX824：ISR 收到字节
#define SIGNAL_IMU485_RAW_READY (1u << 9)   // IMU485：ISR 收到完整帧
#define VOFA_SIGNAL_RAW_READY   (1u << 10)  // VOFA：ISR 收到字节
#define SIGNAL_IMUCAN_RAW_READY (1u << 11)  // IMU CAN：ISR 收到数据帧
```

新增使用任务通知的模块时，在此文件追加新的 bit 位并保持唯一。

---

### 3.3 通用数据类型


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

### 3.4 数学工具函数

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

### 3.5 辅助宏

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

## 4. Tool 工具层

**文件**：`tool/convert.h` / `convert.c`、`tool/process.h`

### 4.1 convert —— 浮点量化与角度工具

供 DM 电机 MIT 协议等模块使用的公共转换模块，统一负责浮点与定点的线性映射和角度折叠。

#### `float_to_uint` — 浮点 → 无符号整数

```c
int float_to_uint(float x_float, float x_min, float x_max, int bits);
```

| 参数 | 说明 |
|------|------|
| `x_float` | 待转换的浮点数 |
| `x_min` / `x_max` | 线性映射范围（`x_min < x_max`） |
| `bits` | 目标无符号整数位数（MIT 位置 16 位，速度/扭矩 12 位） |

**返回值**：`(x - x_min) * (2^bits - 1) / (x_max - x_min)`，即把 `[x_min, x_max]` 线性映射到 `[0, 2^bits - 1]`。

#### `uint_to_float` — 无符号整数 → 浮点

```c
float uint_to_float(int x_int, float x_min, float x_max, int bits);
```

`float_to_uint` 的逆映射：`x_int * (x_max - x_min) / (2^bits - 1) + x_min`。

#### 角度相关宏

```c
#define jie_MI 3.141592f             // π
#define TWO_PI (2.0f * jie_MI)       // 2π
#define Limit_Radian(angle)          // 弧度折叠到 (-π, π]
#define GetAngleBetween360(a)        // 角度折叠到 [0, 360)
#define GetAngleBetween180(angle)    // 角度折叠到 (-180, 180]
#define YAW_ALIGN_ANGLE 0            // Yaw 对齐零点
```

---

### 4.2 process —— 通用数值宏

**文件**：`tool/process.h`

限幅、插值、位操作等通用宏。除标注"指针版"外均为语句表达式宏（GCC 扩展），**参数只求值一次**，可安全传入带副作用的表达式。

| 宏 | 作用 |
|----|------|
| `CONSTRAIN(x, min, max)` | 区间限幅（返回值版） |
| `CONSTRAIN_PTR(ptr, min, max)` | 区间限幅（就地修改指针目标） |
| `ABS(x)` / `ABS_PTR(ptr)` | 绝对值 |
| `MAX(a,b)` / `MIN(a,b)`、`MAX3` / `MIN3` | 极值 |
| `DEADZONE(x, th)` / `DEADZONE_PTR(ptr, th)` | 死区：`\|x\| < th` 时置 0 |
| `SIGN(x)` | 符号函数，返回 -1 / 0 / 1 |
| `LERP(a,b,t)` / `LERP_CLAMP(a,b,t)` | 线性插值（后者将 t 限幅到 [0,1]） |
| `MAP(x, in_min, in_max, out_min, out_max)` | 区间线性映射 |
| `MAP_CLAMP(...)` | 区间映射并限幅到目标区间 |
| `CONSTRAIN_01` / `CONSTRAIN_PM1` / `CONSTRAIN_PI` | 限幅到 [0,1] / [-1,1] / [-π,π] |
| `IS_IN_RANGE(x, min, max)` | 左闭右开范围判断 |
| `IS_CLOSE(a, b, tol)` / `IS_EQUAL_FLOAT(a, b)` | 容差比较（后者 tol = 0.0001f） |
| `ROUND(x)` | 四舍五入取整 |
| `SATURATE_ADD(a,b,max)` / `SATURATE_SUB(a,b,min)` | 饱和加减（防溢出） |
| `ARRAY_SIZE(arr)` | 数组元素个数（勿用于指针） |
| `SET_BITS / CLEAR_BITS / BIT_IS_SET / BIT_IS_CLEARED` | 位操作 |
| `HIGH_BYTE / LOW_BYTE / MAKE_WORD` | 字节拆分与合成 |
| `SQUARE(x)` / `CUBE(x)` | 平方 / 立方 |
| `IS_EVEN / IS_ODD / IS_POWER_OF_TWO` | 奇偶 / 2 的幂判断 |
| `DIV_CEIL(a, b)` / `ALIGN_UP(x, n)` | 向上取整整除 / 对齐 |

**常量**：`RAD_PER_DEG`（度→弧度）、`DEG_PER_RAD`（弧度→度）、`M_PI` / `M_2PI` / `M_PI_2`。

```c
float out = CONSTRAIN(pid_out, -16384.0f, 16384.0f);  // 输出限幅
DEADZONE_PTR(&cmd, 0.02f);                            // 就地加死区
float deg  = rad * DEG_PER_RAD;                       // 弧度 → 角度
float mapped = MAP_CLAMP(raw, 0, 4095, -1.0f, 1.0f);  // ADC 映射到 ±1
```

---

## 5. BSP CAN 驱动

**文件**：`bsp/bsp_can/bsp_can.h` / `bsp_can.c`

CAN BSP 采用**对象式 + 订阅广播**模型：一个 `STM32CAN_t` 控制块绑定一个 HAL CAN 句柄；发送只支持标准数据帧；接收时把帧快照广播给最多 4 个订阅者。协议打包、帧 ID 路由全部归业务模块（DJI / DM 电机、DM IMU），BSP 不承载任何电机协议。

### 核心数据结构

#### `STM32CAN_t` — CAN 控制块

```c
struct STM32CAN {
    BSP_CAN_t id_;                       // BSP 逻辑设备编号（BSP_CAN1 / BSP_CAN2）
    CAN_HandleTypeDef *can_handle_;      // HAL CAN 句柄（&hcan1 / &hcan2）
    STM32CAN_RxSubscriber_t
        subscribers_[STM32CAN_RX_SUBSCRIBER_CAPACITY];  // 固定容量订阅表（4 槽）
    uint8_t subscriber_count_;           // 已注册订阅者数量 [0, 4]
    bool started_;                       // true：已成功 Start，此后禁止再 Subscribe
    err_t last_error_;                   // 最近一次操作错误码
};
```

#### `BSP_CAN_Frame_t` — 接收帧快照

```c
typedef struct {
    uint32_t id_;                      // 帧 ID：标准帧为 StdId，扩展帧为 ExtId
    uint32_t ide_;                     // IDE 标志（CAN_ID_STD / CAN_ID_EXT）
    uint32_t rtr_;                     // RTR 标志（CAN_RTR_DATA / CAN_RTR_REMOTE）
    uint32_t fifo_;                    // 来源 FIFO（CAN_RX_FIFO0 / CAN_RX_FIFO1）
    uint8_t size_;                     // 有效数据长度 DLC，范围 0..8
    uint8_t data_[BSP_CAN_DATA_SIZE];  // 数据区（8 字节，DLC 之外的字节已清零）
} BSP_CAN_Frame_t;
```

#### `STM32CAN_RxCallback_t` — 订阅回调类型

```c
typedef void (*STM32CAN_RxCallback_t)(STM32CAN_t *self,
                                      const BSP_CAN_Frame_t *frame,
                                      void *context);
```

- 在 ISR 上下文中调用，**必须短且非阻塞**（禁止日志输出、等待、动态分配、复杂协议状态机）
- `frame` 指向栈上快照，仅本次回调有效；延后处理必须先拷贝到自有缓冲
- `context` 为订阅时传入的用户上下文，生命周期须覆盖 CAN 运行期

#### `BSP_CAN_t` — 逻辑设备编号

```c
typedef enum {
#ifdef CAN1
    BSP_CAN1,
#endif
#ifdef CAN2
    BSP_CAN2,
#endif
    BSP_CAN_NUMBER,   // 合法 CAN 数量（数组上界）
    BSP_CAN_ID_ERROR  // 无效 ID / 未识别外设
} BSP_CAN_t;
```

**关键常量**：`BSP_CAN_DATA_SIZE = 8`，`STM32CAN_RX_SUBSCRIBER_CAPACITY = 4`。不使用堆内存，不支持运行期退订。

---

### 对象式 API

---

#### `BSP_CAN_get_id` — 外设地址映射逻辑编号

```c
BSP_CAN_t BSP_CAN_get_id(CAN_TypeDef *addr);
```

| 返回值 | 含义 |
|--------|------|
| `BSP_CAN1` / `BSP_CAN2` | 识别成功 |
| `BSP_CAN_ID_ERROR` | `addr` 为 NULL 或未识别 |

---

#### `STM32CAN_GetInstance` — 获取已注册控制块

```c
STM32CAN_t *STM32CAN_GetInstance(BSP_CAN_t id);
```

**返回值**：已 `Init` 的控制块指针；未注册或 ID 非法返回 `NULL`。业务模块（如 `chassis_control_init()`、`joint_init_can()`）通过它获取 `main.c` 中注册的 `can1_instance` / `can2_instance`。

---

#### `STM32CAN_Init` — 初始化 CAN 控制块（绑定 HAL 句柄）

```c
err_t STM32CAN_Init(STM32CAN_t *self, CAN_HandleTypeDef *can_handle);
```

- 第一个参数 `*self` —— `STM32CAN_t` 控制块指针（调用方静态分配），Init 时清零结构体并注册到全局对象表
- 第二个参数 `can_handle` —— HAL CAN 句柄，指定使用哪个 CAN 外设

| 返回值 | 含义 |
|--------|------|
| `OK` | 初始化成功 |
| `PTR_NULL` | 空指针 |
| `NOT_FOUND` | 无法将 `can_handle` 映射到 BSP 逻辑设备 |
| `BUSY` | 该 CAN 外设已被另一个 `STM32CAN_t` 占用 |

**说明**：仅完成对象初始化与对象表注册，不启动外设、不配置过滤器。本工程在 `main.c` 的 USER CODE 2 中完成 `can1_instance` / `can2_instance` 的 Init。

---

#### `STM32CAN_SubscribeRx` — 注册 RX 订阅者（必须在启动前）

```c
err_t STM32CAN_SubscribeRx(STM32CAN_t *self,
                           STM32CAN_RxCallback_t callback,
                           void *context);
```

- 第一个参数 `*self` —— CAN 控制块指针，SubscribeRx 时向其固定容量订阅表追加一个订阅槽
- 第二个参数 `callback` —— 数据到达回调，不可为 NULL；每帧到达时按注册顺序依次调用
- 第三个参数 `context` —— 用户上下文，可为 NULL，回调时原样传回

| 返回值 | 含义 |
|--------|------|
| `OK` | 注册成功（相同 callback + context 重复注册幂等返回 OK） |
| `PTR_NULL` | `self` 或 `callback` 为 NULL |
| `STATE_ERR` | 已经 `Start`，禁止再订阅 |
| `FULL` | 订阅槽已满（共 4 个） |

**说明**：订阅只允许在 `STM32CAN_Start()` 成功前完成。本工程 CAN1 上 DM IMU 占 1 个订阅槽（`dm_motor_attach_can` 未调用时），CAN2 上 DJ 电机总线占 1 个订阅槽。

---

#### `STM32CAN_Start` — 启动 CAN（开启通信和 RX 中断）

```c
err_t STM32CAN_Start(STM32CAN_t *self);
```

- 第一个参数 `*self` —— CAN 控制块指针，Start 时调用 `HAL_CAN_Start` 并激活 FIFO0/FIFO1 消息挂起中断

| 返回值 | 含义 |
|--------|------|
| `OK` | 启动成功 |
| `PTR_NULL` | 空指针 |
| `INIT_ERR` | `HAL_CAN_Start` 或 `ActivateNotification` 失败 |

**说明**：可重复调用（已处于 LISTENING 状态时跳过 Start）。成功后 `started_ = true`。

---

#### `STM32CAN_ConfigFilter` — 配置接收滤波器

```c
err_t STM32CAN_ConfigFilter(STM32CAN_t *self, const CAN_FilterTypeDef *filter);
```

- 第一个参数 `*self` —— CAN 控制块指针，ConfigFilter 时通过 `can_handle_` 调用 `HAL_CAN_ConfigFilter` 设置滤波器
- 第二个参数 `*filter` —— HAL 滤波器配置结构体，由调用方按业务填充

**本工程滤波器分配**：CAN1 使用 FilterBank 0 + FIFO1，CAN2 使用 FilterBank 14 + FIFO0，均配置为接收所有帧（掩码全 0）。

---

#### `STM32CAN_Send` — 发送标准数据帧

```c
err_t STM32CAN_Send(STM32CAN_t *self, uint32_t std_id,
                    const uint8_t *data, size_t size);
```

- 第一个参数 `*self` —— CAN 控制块指针，Send 时通过 `can_handle_` 将数据帧发送到 CAN 总线
- 第二个参数 `std_id` —— 标准帧 ID，范围 `0x000 ~ 0x7FF`
- 第三个参数 `*data` —— 待发送的数据缓冲区首地址
- 第四个参数 `size` —— 数据长度（字节），范围 `1 ~ 8`

| 返回值 | 含义 |
|--------|------|
| `OK` | 发送成功 |
| `PTR_NULL` | 空指针 |
| `OUT_OF_RANGE` | `std_id > 0x7FF` 或 `size > 8` |
| `SIZE_ERR` | `size == 0` |
| `BUSY` | 无空闲邮箱（**不排队、不重试、不覆盖**） |
| `FAILED` | 有邮箱但 `HAL_CAN_AddTxMessage` 失败 |

**说明**：发送不排队。邮箱满时立即返回 `BUSY`，由上层在下一控制周期用最新数据重新组帧发送。

---

#### `STM32CAN_SendByHandle` — 通过 HAL 句柄适配发送

```c
err_t STM32CAN_SendByHandle(CAN_HandleTypeDef *can_handle,
                            uint32_t std_id, const uint8_t *data, size_t size);
```

供仍只持有 HAL 句柄的边界代码使用：先按句柄反查已 `Init` 注册的控制块，再复用 `STM32CAN_Send`。句柄未注册时返回 `NOT_FOUND`。不是独立的发送实现。

---

#### `STM32CAN_HandleRxFrame` — 向全部订阅者广播一帧

```c
void STM32CAN_HandleRxFrame(STM32CAN_t *self, const BSP_CAN_Frame_t *frame);
```

由内部 ISR 路径在构帧成功后调用；也可供测试/上层注入使用。校验 `frame` 非空且 `size_ ≤ 8` 后，按注册顺序调用各订阅回调。

---

#### `STM32CAN_GetLastError` — 读取最近错误码

```c
err_t STM32CAN_GetLastError(const STM32CAN_t *self);
```

**返回值**：`last_error_`；`self` 为 NULL 时返回 `PTR_NULL`。

---

### 接收路径

```text
HAL_CAN_RxFifo0MsgPendingCallback / HAL_CAN_RxFifo1MsgPendingCallback
  → HAL_CAN_GetRxMessage
  → 构造 BSP_CAN_Frame_t（仅复制 DLC 有效字节，尾部清零）
  → STM32CAN_HandleRxFrame
  → 按注册顺序调用全部订阅者回调
```

HAL 读取失败或 DLC 超过 8 时不广播，错误通过 `STM32CAN_GetLastError()` 暴露。

---

### 已移除的旧接口

以下旧接口已随订阅式重构移除，不再提供：

`STM32CAN_SendDjiCurrent`、`STM32CAN_SetRxCallback`、`CAN_Init`、`CAN_Filter_Mask_Config_16bit/32bit`、`CAN_Send_Data_X8`、`canx_receive`、`dm_can_send_data`、`dm_can1/2_rx_callback`、`dj_motor_can1/2_rx_callback` 等全部 weak 回调。

DJI 电机电流控制统一由 `modules/motor/dj_motor/` 的 `dj_motor_set_command()` 等对象式接口完成（见第 16 节）。

---

### 完整使用示例

```c
/* main.c（调度器启动前）：初始化对象 + 过滤器 */
STM32CAN_t can1_instance, can2_instance;
STM32CAN_Init(&can1_instance, &hcan1);
STM32CAN_Init(&can2_instance, &hcan2);
/* ... ConfigFilter ... */

/* 业务模块内：订阅（必须在 Start 之前） */
static void my_rx(STM32CAN_t *self, const BSP_CAN_Frame_t *frame, void *ctx) {
    if (frame->ide_ == CAN_ID_STD && frame->rtr_ == CAN_RTR_DATA) {
        // 按 frame->id_ 解析 frame->data_[0 .. frame->size_-1]
    }
}
STM32CAN_SubscribeRx(&can2_instance, my_rx, NULL);

/* 任务内：启动 + 周期发送 */
STM32CAN_Start(&can2_instance);
const uint8_t tx[8] = {0};
STM32CAN_Send(&can2_instance, 0x200, tx, 8);
```

---

## 6. BSP UART DMA 驱动

**文件**：`bsp/bsp_uart/bsp_uart.h` / `bsp_uart.c`

RX 统一使用 `ReceiveToIdle + DMA_CIRCULAR` 单缓冲循环接收；TX 按发送语义分两条独立路径：

| 接口 | DMA 模式 | 用途 |
|------|----------|------|
| `STM32UARTFrameTx_*` | `DMA_NORMAL`，一帧发送、一帧等待 | 变长协议帧（VOFA、LX824），每帧只发送一次 |
| `STM32UARTDoubleBufTx_*` | 硬件 DBM，M0/M1 自动循环切换 | 定长连续数据流（如音频流/周期波形） |

同一 UART 只能绑定一个 TX 对象，可以同时绑定 RX 对象。

### 通用类型

#### 回调类型

```c
// 用户接收回调：data 指向本次新收到的数据片段（ISR 上下文，回绕时最多分两段）
typedef void (*STM32UART_RxCallback_t)(uint8_t *data, size_t size);

// 变长帧 TX 的 UART TC 完成回调（最后一个停止位已发出）
typedef void (*STM32UART_TxCompleteCallback_t)(void);

// 硬件 DBM 换页回调：data 指向刚发送完的空闲页，size 为固定页长
// 在 DMA ISR 中填充下一轮数据；不更新则该页内容循环重发
typedef void (*STM32UART_TxRefillCallback_t)(uint8_t *data, size_t size);
```

#### `BSP_UART_RawData_t` — 原始缓冲描述

```c
typedef struct {
    void *addr_;   // 缓冲区起始地址
    size_t size_;  // 缓冲区容量（字节）
} BSP_UART_RawData_t;
```

BSP 只保存地址和长度，不拥有缓冲区生命周期，调用方必须保证缓冲区长期有效（通常用 `{buffer, sizeof(buffer)}` 复合字面量）。

#### `BSP_UART_t` — 逻辑设备编号

```c
typedef enum {
    BSP_USART1, BSP_USART2, BSP_USART3, BSP_USART6,
    BSP_UART4,  BSP_UART5,
    BSP_UART_NUMBER,
    BSP_UART_ID_ERROR
} BSP_UART_t;
```

（枚举项按 CubeMX 实际启用的外设宏展开。）

---

### 6.1 单缓冲 DMA RX

#### `STM32UART_t` — RX 控制块

```c
typedef struct {
    BSP_UART_t id_;                       // BSP 逻辑串口编号
    size_t last_rx_pos_;                  // 软件读指针（上次处理到的 DMA 写入位置）
    BSP_UART_RawData_t dma_buff_rx_;      // RX 循环 DMA 原始缓冲
    UART_HandleTypeDef *uart_handle_;     // CubeMX 生成的 HAL UART 句柄
    STM32UART_RxCallback_t rx_callback_;  // 新数据片段回调
    err_t last_error_;                    // 最近一次 BSP 操作结果
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
- 第三个参数 `dma_buff_rx` —— 循环 DMA 缓冲的地址和大小
- 第四个参数 `callback` —— 接收回调函数指针，UART 接收到新数据片段时在 ISR 中调用

| 返回值 | 含义 |
|--------|------|
| `OK` | 成功 |
| `NOT_FOUND` | 无法映射到 BSP 逻辑串口 ID |
| `SIZE_ERR` | DMA 缓冲大小为 0 |
| `BUSY` | 该 UART 外设已被其他 RX 控制块占用 |

**说明**：DMA 缓冲在整个接收期间必须保持有效；同一 UART 外设只能注册一个 RX 控制块。

---

#### `STM32UART_SetRxDMA` — 启动 DMA 接收

```c
err_t STM32UART_SetRxDMA(STM32UART_t *self);
```

将 RX DMA 配置为 `DMA_CIRCULAR`，并调用 `HAL_UARTEx_ReceiveToIdle_DMA()` 开始接收。空闲线中断自动切分数据包，无需外部干预。

---

#### `STM32UART_SetRxCallback` / `STM32UART_HandleRxData` / `STM32UART_GetLastError`

```c
void  STM32UART_SetRxCallback(STM32UART_t *self, STM32UART_RxCallback_t callback);
void  STM32UART_HandleRxData(STM32UART_t *self, uint8_t *data, size_t size);
err_t STM32UART_GetLastError(const STM32UART_t *self);
```

- `SetRxCallback`：运行期替换回调，不重启 DMA
- `HandleRxData`：处理一段已切分好的数据（内部做空指针/长度保护后调用用户回调），由 HAL RX 事件分发路径调用，回绕时自动拆分为两段
- `GetLastError`：读取 `last_error_`

---

### 6.2 变长帧 DMA TX（FrameTx）

#### `STM32UARTFrameTx_t` — 变长帧 TX 控制块

```c
typedef struct {
    BSP_UART_t id_;                                // BSP 逻辑串口编号
    size_t last_tx_pos_;                           // 预留发送位置记录
    BSP_UART_RawData_t dma_buff_0_;                // TX 软件缓冲 0
    BSP_UART_RawData_t dma_buff_1_;                // TX 软件缓冲 1
    UART_HandleTypeDef *uart_handle_;              // HAL UART 句柄
    STM32UART_TxCompleteCallback_t tx_callback_;   // UART TC 发送完成回调
    err_t last_error_;                             // 最近一次操作结果
    volatile uint8_t active_buf_;                  // 当前 DMA 正在发送的缓冲编号
    volatile size_t pending_size_;                 // 另一块缓冲中待发送字节数
    volatile bool tx_busy_;                        // HAL DMA 是否正在发送
    bool dma_ready_;                               // SetTxDMA 是否成功
} STM32UARTFrameTx_t;
```

两块 TX 缓冲须等长，单次写入不能超过缓冲容量。

#### `STM32UARTFrameTx_Init` — 初始化（绑定 UART 硬件和双缓冲）

```c
err_t STM32UARTFrameTx_Init(STM32UARTFrameTx_t *self,
                            UART_HandleTypeDef *uart_handle,
                            BSP_UART_RawData_t dma_buff_0,
                            BSP_UART_RawData_t dma_buff_1,
                            STM32UART_TxCompleteCallback_t callback);
```

- 第一个参数 `*self` —— `STM32UARTFrameTx_t` 结构体指针，该结构体包含两块 TX 缓冲、发送状态机变量和完成回调，Init 时绑定到指定 UART
- 第二个参数 `uart_handle` —— HAL UART 句柄，指定使用哪个串口的 TX 通道
- 第三 / 四个参数 `dma_buff_0` / `dma_buff_1` —— 两块等长的 TX 缓冲
- 第五个参数 `callback` —— UART TC 发送完成回调，可 NULL

---

#### `STM32UARTFrameTx_SetTxDMA` — 配置 TX DMA

```c
err_t STM32UARTFrameTx_SetTxDMA(STM32UARTFrameTx_t *self);
```

将 TX DMA 配置为 `DMA_NORMAL` 并复位双缓冲发送状态。

---

#### `STM32UARTFrameTx_Write` — 写入一帧待发送数据

```c
err_t STM32UARTFrameTx_Write(STM32UARTFrameTx_t *self,
                             const uint8_t *data, size_t size);
```

- 第一个参数 `*self` —— 变长帧 TX 控制块指针，Write 时按 `tx_busy_` 状态决定立即发送或排队
- 第二个参数 `*data` —— 待发送数据首地址（实际长度 `size`，不补齐、不转义）
- 第三个参数 `size` —— 数据长度，范围 `1 ~ 缓冲容量`

**发送语义**：

| 场景 | 行为 |
|------|------|
| DMA 空闲 | 立即 `Flush` 启动发送 |
| 一帧在发、队列为空 | 复制到 pending 缓冲，TC 完成后自动续发 |
| 一帧在发、一帧已排队 | 返回 `BUSY`，**新帧不入队、不覆盖旧帧**，调用方可稍后重试 |

**说明**：BSP 内部保存/恢复 `PRIMASK` 临界区，模块无需再包裹 FreeRTOS 临界区。初次提交失败不保留未接受的新帧；已接受的排队帧续发失败时保留，可用 `Flush()` 重试。

---

#### `STM32UARTFrameTx_Flush` 等其余接口

```c
err_t STM32UARTFrameTx_Flush(STM32UARTFrameTx_t *self);            // 提交 pending 数据
void  STM32UARTFrameTx_SetTxCompleteCallback(                       // 更新 TC 回调
    STM32UARTFrameTx_t *self, STM32UART_TxCompleteCallback_t callback);
err_t STM32UARTFrameTx_GetLastError(const STM32UARTFrameTx_t *self); // 读取错误码
void  STM32UARTFrameTx_HandleTxComplete(STM32UARTFrameTx_t *self);  // UART TC 事件（内部分发）
```

---

### 6.3 硬件双缓冲连续 TX（DBM）

#### `STM32UARTDoubleBufTx_t` — 硬件 DBM 控制块

```c
typedef struct {
    BSP_UART_t id_;                              // BSP 逻辑串口编号
    BSP_UART_RawData_t dma_buff_0_;              // 硬件 M0 缓冲
    BSP_UART_RawData_t dma_buff_1_;              // 硬件 M1 缓冲
    UART_HandleTypeDef *uart_handle_;            // HAL UART 句柄
    STM32UART_TxRefillCallback_t tx_callback_;   // 空闲页填充回调
    volatile err_t last_error_;                  // 最近一次操作结果
    volatile uint8_t active_buf_;                // 完成 ISR 采样的 CT（硬件自动切换）
    volatile bool tx_busy_;                      // 连续 DMA 是否已启动
    bool dma_ready_;                             // SetTxDMA 是否成功
} STM32UARTDoubleBufTx_t;
```

M0/M1 以相同 NDTR 自动循环，缓冲区必须位于 DMA 可访问的 SRAM（**不能使用 F405 CCM**），两页等长、非重叠，容量 1~65535 字节。

#### `STM32UARTDoubleBufTx_Init` / `SetTxDMA` — 初始化与配置

```c
err_t STM32UARTDoubleBufTx_Init(STM32UARTDoubleBufTx_t *self,
                                UART_HandleTypeDef *uart_handle,
                                BSP_UART_RawData_t dma_buff_0,
                                BSP_UART_RawData_t dma_buff_1,
                                STM32UART_TxRefillCallback_t callback);
err_t STM32UARTDoubleBufTx_SetTxDMA(STM32UARTDoubleBufTx_t *self);
```

- `Init` 的第五个参数 `callback` —— 换页填充回调（签名 `void (*)(uint8_t *data, size_t size)`），在 DMA ISR 中收到刚发送完的空闲页，应尽快填满 `size` 字节
- `SetTxDMA` 配置 `DMA_CIRCULAR + DBM`、`PAR`、`M0AR`、`M1AR` 和固定 `NDTR`，此时尚未产生 UART DMA 请求

---

#### `STM32UARTDoubleBufTx_Write` / `Flush` / `Stop`

```c
err_t STM32UARTDoubleBufTx_Write(STM32UARTDoubleBufTx_t *self,
                                 const uint8_t *data, size_t size);
err_t STM32UARTDoubleBufTx_Flush(STM32UARTDoubleBufTx_t *self);
err_t STM32UARTDoubleBufTx_Stop(STM32UARTDoubleBufTx_t *self);
```

- `Write`：仅供空闲时启动——长度必须等于页容量，复制同一份数据到两页后调用 `Flush()`；运行中返回 `BUSY`，不会改写 DMA 使用中的页
- `Flush`：调用 `HAL_DMAEx_MultiBufferStart_IT()` 并开启 UART `DMAT` 启动连续发送；**两页必须都已准备好**；运行中再次调用返回 `BUSY`
- `Stop`：只能在任务上下文且未进入临界区时调用，只关闭 TX DMA（RX 不受影响）。停止可能截断当前页，UART 中已有的字节仍会发出；需要完整单次帧时请使用 FrameTx

**填充回调期限契约**：**最大 DMA 中断延迟 + 回调填充耗时必须小于一页发送时间**（8N1 时一页 ≈ `size × 10 / baud_rate` 秒，高优先级中断和临界区耗时也要计入）。应先在任务中准备数据、在回调中快速复制，不能阻塞等待。不更新页内容时该页会在下一轮重复发送。

---

#### `STM32UARTDoubleBufTx_SetTxCompleteCallback` 等其余接口

```c
void  STM32UARTDoubleBufTx_SetTxCompleteCallback(    // 更新换页填充回调
    STM32UARTDoubleBufTx_t *self, STM32UART_TxRefillCallback_t callback);
err_t STM32UARTDoubleBufTx_GetLastError(             // 读取错误码
    const STM32UARTDoubleBufTx_t *self);
void  STM32UARTDoubleBufTx_HandleTxComplete(         // DMA M0/M1 完成事件（内部分发）
    STM32UARTDoubleBufTx_t *self);
```

`HandleTxComplete` 从 DMA 完成回调进入，读取硬件 `CT` 并把刚完成的页交给填充回调；不会重启 DMA，也不经过 `HAL_UART_TxCpltCallback()` 重复分发。

---

### 6.4 HAL 回调分发

```c
// RX 空闲线事件 → 按 DMA 计数切分新数据 → 用户回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

// 变长帧 TX：UART TC → 释放当前页 → 续发排队帧 → 用户 TC 回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

// 硬件 DBM TX：DMA M0/M1 完成 → 读取 CT → 换页填充回调
// （从 DMA 层回调进入，不走 HAL_UART_TxCpltCallback）
```

**说明**：BSP 自动注册并分发到正确的对象表。用户在 CubeMX 生成代码后不需要修改 `stm32f4xx_it.c` 中的中断处理。

**模块迁移**：`Vofa` 与 `LX824` 的发送通道已从旧 `DoubleBuf` 语义迁移到 `STM32UARTFrameTx_*`，保留原有 firewater 帧和舵机请求-应答协议。

---

## 7. BSP PWM 驱动

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

## 8. BSP I²C 驱动

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

## 9. BSP USB CDC 驱动

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

## 10. BSP DWT 精密定时

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

## 11. DR16 大疆遥控器（SBUS）

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

## 12. I6X 富斯 iBus 遥控器

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

## 13. LX824 总线串口舵机

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
    STM32UARTFrameTx_t uart_send_;  // 发送通道（变长帧 DMA TX）→ 发指令到舵机
    STM32UART_t uart_receive_;           // 接收通道（循环 DMA RX）→ 收舵机应答
    err_t tx_init_error_;               // TX 初始化结果
    err_t rx_init_error_;               // RX 初始化结果
} LX824_t;
```

### `LX824_Init` — 初始化（绑定 UART 硬件）

```c
err_t LX824_Init(LX824_t *self, UART_HandleTypeDef *uart_handle);
```

- 第一个参数 `*self` —— `LX824_t` 结构体指针，该结构体包含 TX 变长帧 DMA 发送通道 (`uart_send_`)、RX 循环 DMA 接收通道 (`uart_receive_`)、任务通知句柄 (`thread_alert`) 和初始化错误码，Init 时绑定到指定 UART 外设并初始化收发 DMA 通道。**本工程使用 USART1（`&huart1`）连接 LX824 舵机总线**
- 第二个参数 `uart_handle` —— HAL UART 句柄，指定使用哪个串口与舵机总线通信

**内部操作**：
- `self->uart_send_` ← 初始化 TX 变长帧 DMA（发指令给舵机）
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
1. `STM32UARTFrameTx_SetTxDMA(&self->uart_send_)` — 启动 TX DMA
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

> ⚠️ **关于 `LX824_t *self`**：该结构体包含 TX 变长帧发送通道和 RX 循环 DMA 接收通道，**任何舵机指令都必须通过 `self` 才能发送到物理总线上**。

---

## 14. VOFA+ 上位机调试协议

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
    STM32UARTFrameTx_t uart_send_;            // TX 变长帧发送通道
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

- 第一个参数 `*self` —— `Vofa_t` 结构体指针，该结构体包含 TX 变长帧 DMA 发送通道 (`uart_send_`)、RX 循环 DMA 接收通道 (`uart_receive_`)、初始化错误码 (`tx_init_error_`/`rx_init_error_`) 和命令回调 (`command_callback_`)，Init 时绑定到指定 UART 硬件
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

- 第一个参数 `*self` —— `Vofa_t` 结构体指针，该结构体包含 TX 变长帧 DMA 发送通道和 RX 循环 DMA 接收通道，Start 时启动这两个 DMA 通道开始工作

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

- 第一个参数 `*self` —— `Vofa_t` 结构体指针，该结构体包含 TX 变长帧 DMA 发送通道 (`uart_send_`)，Send 时通过该通道发出 firewater 格式的浮点数据帧到上位机
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

## 15. Game RoboMaster 裁判系统

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

## 16. DJI 电机（dj_motor）

**文件**：`modules/motor/dj_motor/dj_motor_def.h`、`dj_motor_drv.h/.c`、`dj_motor_ctrl.h/.c`

`dj_motor_ctrl.h` 是应用层唯一需要包含的 DJ 公共头。调用方静态持有 `dj_motor_bus_t`（每条 `STM32CAN_t` 一条总线）和 `dj_motor_t`（每个电机一个实例），模块通过 bsp_can 的订阅广播接收反馈，占用每条 CAN 1 个订阅槽。

### 地址模型

| 型号 | 设备 ID | 反馈帧 ID | 控制组 ID | 帧内槽位 | 指令限幅 |
|------|---------|-----------|-----------|----------|----------|
| M3508 / C620 | 1–4 | `0x200 + ID` | `0x200` | `ID - 1` | ±16384 |
| M3508 / C620 | 5–8 | `0x200 + ID` | `0x1FF` | `ID - 5` | ±16384 |
| M2006 / C610 | 1–4 | `0x200 + ID` | `0x200` | `ID - 1` | ±10000 |
| M2006 / C610 | 5–8 | `0x200 + ID` | `0x1FF` | `ID - 5` | ±10000 |
| GM6020 | 1–4 | `0x204 + ID` | `0x1FF` | `ID - 1` | ±30000 |
| GM6020 | 5–7 | `0x204 + ID` | `0x2FF` | `ID - 5` | ±30000 |

> GM6020 无设备 ID 8；其控制量为**电压**原始指令（限幅 ±30000），勿按电流理解。减速比：M3508 = 19，M2006 = 36，GM6020 直驱 = 1。编码器分辨率 8192/圈。

### 核心数据结构

#### `dj_motor_bus_t` — 单条 CAN 总线上的 DJ 协议对象

```c
typedef struct {
    STM32CAN_t *can;             // 绑定的 BSP CAN 设备
    dj_motor_t *motors[8];       // 注册顺序保存的电机指针表（容量 DJ_MOTOR_BUS_CAPACITY）
    dj_motor_group_state_t groups[3];  // 0:0x200  1:0x1FF  2:0x2FF
    uint8_t motor_count;         // 当前已注册电机数量
    bool initialized;            // bus_init 是否成功
} dj_motor_bus_t;
```

#### `dj_motor_group_state_t` — 控制组拼包状态（写齐再发）

```c
typedef struct {
    dj_motor_group_e id;              // 本控制组的标准帧 ID
    dj_motor_t *members[4];           // 槽位 0..3 对应的电机指针
    uint8_t group_mask;               // 已注册成员位图，bit0 对应槽 0
    uint8_t pending_mask;             // 本轮已写命令成员位图
    int16_t tx_buff[4];               // 物理方向命令缓存（大端打包），空槽为 0
} dj_motor_group_state_t;
```

当 `pending_mask == group_mask` 且非 0 时，ctrl 层自动打包 8 字节大端帧并通过 `STM32CAN_Send` 发出。

#### `dj_motor_t` — 单个电机软件实例

```c
struct dj_motor {
    dj_motor_bus_t *bus;                   // 所属总线，init 后只读
    dj_motor_type_e type;                  // DJ_MOTOR_M3508 / M2006 / GM6020
    uint8_t device_id;                     // 设备 ID
    bool reversed;                         // true 时命令与反馈速度/电流/角度取反
    bool initialized;
    dj_motor_route_t route;                // 自动推导的 feedback_id / group / slot
    int16_t command;                       // 逻辑方向最新命令（限幅后）
    dj_motor_raw_feedback_t raw_feedback;  // ISR 写入的最近原始反馈
    uint32_t last_feedback_tick;           // 最近反馈时间戳
    bool feedback_valid;                   // 是否至少收过一帧合法反馈
};
```

#### `dj_motor_feedback_t` — 任务侧反馈快照

```c
typedef struct {
    uint16_t encoder_raw;         // 硬件编码器原始值 0..8191（不因 reversed 翻转）
    int16_t  speed_rpm;           // 逻辑方向转子 RPM
    int16_t  current;             // 逻辑方向电流原始量
    uint8_t  temperature;         // 温度 ℃
    float    position_rad;        // 逻辑方向位置角，单位 rad
    float    speed_rad_s;         // 输出轴角速度 rad/s（已除减速比）
    uint32_t last_feedback_tick;  // 最近一次合法反馈的 HAL_GetTick
    bool     valid;               // 至少成功解析过一帧反馈
} dj_motor_feedback_t;
```

---

### API

---

#### `dj_motor_bus_init` — 初始化总线对象并订阅 CAN 回调

```c
err_t dj_motor_bus_init(dj_motor_bus_t *bus, STM32CAN_t *can);
```

- 第一个参数 `*bus` —— 调用方静态分配的总线对象，bus_init 时清零并写入三组控制组 ID
- 第二个参数 `*can` —— 已 `STM32CAN_Init` 但通常尚未 Start 的 BSP CAN 设备，向其注册 1 个 RX 订阅（context 指向本 bus）

| 返回值 | 含义 |
|--------|------|
| `OK` | 成功 |
| `PTR_NULL` | 空指针 |
| `STATE_ERR` / `FULL` 等 | `STM32CAN_SubscribeRx` 错误码透传 |

**说明**：须在 `STM32CAN_Start()` 之前调用；订阅失败会清零 bus，保证无半初始化。

---

#### `dj_motor_init` — 初始化电机实例并注册到总线

```c
err_t dj_motor_init(dj_motor_t *motor, dj_motor_bus_t *bus,
                    dj_motor_type_e type, uint8_t device_id, bool reversed);
```

- 第一个参数 `*motor` —— 调用方静态分配的电机对象
- 第二个参数 `*bus` —— 已成功 `bus_init` 的总线
- 第三个参数 `type` —— 电机型号，决定合法 ID 范围、反馈/控制路由、指令限幅和减速比
- 第四个参数 `device_id` —— 设备 ID（M3508/M2006 为 1..8，GM6020 为 1..7）
- 第五个参数 `reversed` —— `true` 时命令与反馈速度/电流/角度取反

| 返回值 | 含义 |
|--------|------|
| `OK` | 成功 |
| `PTR_NULL` | 空指针 |
| `STATE_ERR` | 总线未初始化 / CAN 已 Start / 重复 init |
| `FULL` | 总线容量满（8 台） |
| `ARG_ERR` | 路由非法（如 GM6020 ID 8）或同 bus 冲突 |

**说明**：失败时不修改总线注册表，无半注册。初始化后禁止注销、换总线、换型号或换 ID。同 bus 禁止重复反馈 ID 与 `(group, slot)`；跨 bus 可复用设备 ID。

---

#### `dj_motor_set_command` — 写入命令（组内写齐后自动发送）

```c
err_t dj_motor_set_command(dj_motor_t *motor, int16_t command);
```

- 第一个参数 `*motor` —— 已初始化的电机实例
- 第二个参数 `command` —— 逻辑方向协议原始命令（先按型号限幅，再可选反向写入物理缓存）

**发送语义（半写不发）**：命令写入组 `tx_buff` 并置 `pending_mask` 位；当组内**全部**成员本轮都写过命令时才自动打包发送。若组内 4 个成员只写了 3 个，本周期不会发送。发送失败时 `pending` 保留并返回 BSP 错误码，可下周期重写或 `force_flush`。

---

#### `dj_motor_force_flush_group` — 按当前缓存立即发送

```c
err_t dj_motor_force_flush_group(dj_motor_bus_t *bus, dj_motor_group_e group);
```

不要求写齐，按当前组缓存立即发送（未写过命令的槽保持缓存值，注册时已置 0）。发送成功后清零 `pending_mask`。组内无注册电机返回 `NOT_FOUND`。

---

#### `dj_motor_zero_and_flush` — 清零并发送全零帧（安全停机唯一推荐路径）

```c
err_t dj_motor_zero_and_flush(dj_motor_bus_t *bus, dj_motor_group_e group);
```

将组内已注册电机命令清零并立即发送全零帧。**底盘停机、遥控离线、电机超时、执行器总开关关闭等路径必须调用本接口**，而不是依赖 `set_command(0)` 碰巧写齐。

---

#### `dj_motor_get_feedback` / `dj_motor_is_online` — 反馈查询

```c
err_t dj_motor_get_feedback(const dj_motor_t *motor, dj_motor_feedback_t *feedback);
bool  dj_motor_is_online(const dj_motor_t *motor, uint32_t now_tick,
                         uint32_t timeout_ticks);
```

- `get_feedback`：短临界区内只拷贝原始整数与 tick，编码器→弧度、RPM→rad/s 及 reverse 换算在临界区外完成
- `is_online`：已初始化、至少收过一帧反馈、且 `(now - last)` 无符号差不超过超时返回 `true`（无符号减法天然兼容 tick 回绕）。底盘使用 20ms 超时

---

### 完整使用示例（底盘，CAN2 + 四个 M3508）

```c
static dj_motor_bus_t chassis_bus;
static dj_motor_t     chassis_motors[4];

/* 1. 获取 main.c 注册的 CAN2 对象（CAN Start 之前） */
STM32CAN_t *can2 = STM32CAN_GetInstance(BSP_CAN_get_id(CAN2));
dj_motor_bus_init(&chassis_bus, can2);              // 占用 1 个订阅槽

/* 2. 注册 4 台电机（设备 ID 1-4 → 控制组 0x200） */
for (uint8_t i = 0; i < 4; i++)
    dj_motor_init(&chassis_motors[i], &chassis_bus, DJ_MOTOR_M3508, i + 1, false);

/* 3. 启动 CAN（之后不能再注册新电机） */
STM32CAN_Start(can2);

/* 4. 控制周期内：速度环输出写命令，写齐自动发送 0x200 帧 */
dj_motor_set_command(&chassis_motors[0], (int16_t)pid_out);

/* 5. 停机 / 遥控离线 / 电机超时 */
dj_motor_zero_and_flush(&chassis_bus, DJ_MOTOR_GROUP_200);
```

---

## 17. 达妙电机（dm_motor）

**文件**：`modules/motor/dm_motor/dm_motor_def.h`、`dm_motor_drv.h/.c`、`dm_motor_ctrl.h/.c`

达妙电机协议层由 def / drv / ctrl 五文件组成，保留全局六路 `motor[]`，支持 MIT / POS / SPD / PSI 四种控制模式和寄存器读写。

### 控制模式

发送控制帧时，实际标准帧 ID = 电机基础 ID + 模式偏移量：

| 模式 | 偏移量 | `mode_e` | 控制量 |
|------|--------|----------|--------|
| MIT 力矩/阻抗 | `MIT_MODE` 0x000 | `mit_mode` (1) | pos / vel / Kp / Kd / tor |
| 位置-速度 | `POS_MODE` 0x100 | `pos_mode` (2) | pos / vel |
| 纯速度 | `SPD_MODE` 0x200 | `spd_mode` (3) | vel |
| 位置-速度-电流 | `PSI_MODE` 0x300 | `psi_mode` (4) | pos / vel / cur |

MIT 增益量化范围：`KP_MIN 0 ~ KP_MAX 500`，`KD_MIN 0 ~ KD_MAX 5`（12 bit）。

### 核心数据结构

#### `motor_t` — 单个达妙电机对象

```c
typedef struct {
    uint16_t id;          // 电机命令标准帧 ID（主控发送方向）
    uint16_t mst_id;      // 电机反馈标准帧 ID（主控接收方向）
    motor_fbpara_t para;  // 最近一次实时反馈解析结果
    motor_ctrl_t   ctrl;  // 当前控制模式及目标值
    esc_inf_t      tmp;   // 参数读取状态与驱动器寄存器镜像
} motor_t;

extern motor_t motor[num];   // 全局六路电机数组（Motor1 ~ Motor6）
```

`dm_motor_init()` 将 6 台电机命令 ID 初始化为 `0x10 ~ 0x15`、反馈 ID 为 `0x20 ~ 0x25`，默认位置-速度模式。

#### `motor_ctrl_t` — 控制目标

```c
typedef struct {
    uint8_t mode;    // 控制模式（mode_e）
    float pos_set;   // 目标位置 rad
    float vel_set;   // 目标速度 rad/s
    float tor_set;   // 目标扭矩 N·m
    float cur_set;   // 目标电流 A
    float kp_set;    // MIT 比例增益
    float kd_set;    // MIT 微分增益
} motor_ctrl_t;
```

#### `motor_fbpara_t` — 实时反馈

```c
typedef struct {
    int id, state;                       // 电机 CAN ID 与状态码
    int p_int, v_int, t_int;             // 位置/速度/扭矩原始整数（调试量化用）
    int kp_int, kd_int;                  // MIT 增益原始整数
    float pos, vel, tor;                 // 还原后：rad / rad·s⁻¹ / N·m
    float Kp, Kd;                        // MIT 增益
    float Tmos, Tcoil;                   // MOS / 线圈温度 ℃
} motor_fbpara_t;
```

#### `esc_inf_t` — 驱动器寄存器镜像

`read_all_motor_data()` 轮询得到的参数表，包含欠压/过温/过流阈值（`UV/OT/OC_Value`）、加减速（`ACC/DEC`）、`MAX_SPD`、`MST_ID/ESC_ID`、MIT 映射范围（`PMAX/VMAX/TMAX`）、速度环/位置环增益（`KP_ASR/KI_ASR/KP_APR/KI_APR`）、极对数 `NPP`、减速比 `Gr` 等 40 余项，详见 `dm_motor_def.h`。

---

### API

---

#### `dm_motor_init` — 初始化六个电机对象

```c
void dm_motor_init(void);
```

初始化 `motor[]` 的 ID / 反馈 ID / 默认模式（位置-速度）和 MIT 映射范围。`joint_task` 启动时调用。

---

#### `dm_motor_attach_can` — 订阅 CAN 反馈回调

```c
err_t dm_motor_attach_can(STM32CAN_t *can);
```

把 `dm_can1_rx_callback` 订阅到指定 BSP CAN 设备（占 1 个订阅槽），订阅后仅接收 11 bit 标准数据帧，反馈 ID `0x20 ~ 0x25` 路由到对应电机并实时解码。

> ⚠️ 当前工程**未调用**此函数（无调用点），DM 实时反馈不会自动解析；`motor[].para` 保持初值。关节控制仅使用发送通路。如需反馈闭环，须在 CAN Start 前调用，并注意与 DM IMU CAN（ID 0x10 / 0x20）的 ID 冲突。

---

#### `dm_motor_ctrl_send` — 按当前模式发送控制命令

```c
void dm_motor_ctrl_send(STM32CAN_t *hcan, motor_t *motor);
```

按 `motor->ctrl.mode` 选择 `mit_ctrl` / `pos_ctrl` / `spd_ctrl` / `psi_ctrl` 编码发送。

#### 特殊命令帧

```c
void dm_motor_enable (STM32CAN_t *hcan, motor_t *motor);  // 使能（按当前模式）
void dm_motor_disable(STM32CAN_t *hcan, motor_t *motor);  // 失能
void dm_motor_clear_err(STM32CAN_t *hcan, motor_t *motor); // 清除错误
void dm_motor_clear_para(motor_t *motor);                  // 仅清 RAM 控制目标，不发包
void enable_motor_mode (STM32CAN_t *hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(STM32CAN_t *hcan, uint16_t motor_id, uint16_t mode_id);
void save_pos_zero     (STM32CAN_t *hcan, uint16_t motor_id, uint16_t mode_id); // 保存零点
void clear_err         (STM32CAN_t *hcan, uint16_t motor_id, uint16_t mode_id);
```

#### 直发控制帧（不经过 `motor->ctrl`）

```c
void mit_ctrl(STM32CAN_t *hcan, motor_t *motor, uint16_t motor_id,
              float pos, float vel, float kp, float kd, float tor);
void pos_ctrl(STM32CAN_t *hcan, uint16_t motor_id, float pos, float vel);
void spd_ctrl(STM32CAN_t *hcan, uint16_t motor_id, float vel);
void psi_ctrl(STM32CAN_t *hcan, uint16_t motor_id, float pos, float vel, float cur);
```

- `mit_ctrl`：位置/速度/增益/扭矩按电机 `PMAX/VMAX/TMAX` 范围量化为 16/12/12/12/12 bit
- `pos_ctrl` / `spd_ctrl`：浮点以 IEEE 754 原始字节发送
- `psi_ctrl`：协议发送值为 `vel × 100`、`cur × 10000`

#### 寄存器读写

```c
void read_motor_data (STM32CAN_t *hcan, uint16_t id, uint8_t rid);   // 读寄存器（0x7FF）
void write_motor_data(STM32CAN_t *hcan, uint16_t id, uint8_t rid,
                      uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3); // 写 32bit 寄存器
void save_motor_data (STM32CAN_t *hcan, uint16_t id, uint8_t rid);   // 参数掉电保存
void read_motor_ctrl_fbdata(STM32CAN_t *hcan, uint16_t id);          // 请求控制状态回传
```

#### 参数轮询状态机（任务上下文）

```c
void read_all_motor_data(STM32CAN_t *hcan, motor_t *motor);
void receive_motor_data(motor_t *motor, const uint8_t *data);
```

- `read_all_motor_data`：消费参数反馈快照并按 `tmp.read_flag` 请求下一项寄存器，**须在任务上下文周期调用**（不要在 CAN 中断回调中调用），每次最多处理一个快照并发出一个新的读命令
- `receive_motor_data`：解析一帧命令字 0x33 的寄存器反馈，写入 `tmp` 对应成员并推进轮询

#### MIT 量化辅助

```c
int   read_dm_motor_pos(float pos, float p_max, float p_min);  // 16 bit
int   read_dm_motor_vel(float vel, float v_max, float v_min);  // 12 bit
int   read_dm_motor_tor(float tor, float t_max, float t_min);  // 12 bit
float set_dm_motor_pos(int pos_int, float p_max, float p_min);
float set_dm_motor_vel(int vel_int, float v_max, float v_min);
float set_dm_motor_tor(int tor_int, float t_max, float t_min);
void  dm_motor_fbdata(motor_t *motor, const uint8_t *rx_data); // 解码实时反馈帧
```

---

### 完整使用示例（关节控制，CAN1）

```c
/* joint_task 启动时 */
dm_motor_init();                    // 初始化 6 台电机对象（ID 0x10~0x15 / 0x20~0x25）

STM32CAN_t *can1 = STM32CAN_GetInstance(BSP_CAN_get_id(CAN1));
dm_motor_enable(can1, &motor[0]);   // 按当前模式发送使能帧（逐台、间隔 osDelay(1)）

/* 控制周期（1ms）内：分 3 相轮发，每周期 2 台 pos_ctrl，均衡 CAN 负载 */
pos_ctrl(can1, motor[Motor1].id, pos[0], 0.5f);
pos_ctrl(can1, motor[Motor2].id, pos[1], 0.5f);
joint_send_phase = (joint_send_phase + 1U) % 3U;

/* 失能 */
dm_motor_disable(can1, &motor[i]);
```

---

## 18. 达妙 IMU（dm_imu）

提供 CAN 与 RS485 两种链路的达妙 IMU 驱动，二者共用 `comp_def.h` 中独立的任务通知信号位。

### 18.1 CAN 链路（imu_can）

**文件**：`modules/dm_imu/imu_can.h` / `imu_can.c`

| 参数 | 值 |
|------|-----|
| 物理层 | CAN（本工程挂 CAN1） |
| 默认命令 ID / 反馈 ID | `0x10` / `0x20`（`IMU_CAN_DEFAULT_ID / MST_ID`） |
| 数据量化范围 | 加速度 ±235.2，角速度 ±34.88，Pitch ±90°，Roll ±180°，Yaw ±180° |
| 离线超时 | 20ms（`IMUCAN_OFFLINE_TIMEOUT_MS`） |

> ⚠️ 默认 ID `0x10` / `0x20` 与 DM 电机 `0x10~0x15` / `0x20~0x25` 重叠，同挂 CAN1 时需重新分配其中一方。

#### `IMUCAN_t` — CAN IMU 对象

```c
typedef struct {
    TaskHandle_t thread_alert;  // 任务通知句柄（用户须在 Start 前赋值）
    STM32CAN_t *can_;           // 绑定的 BSP CAN 设备
    uint8_t can_id, mst_id;     // 命令 / 反馈 ID
    imu_t data;                 // 姿态数据（pitch/roll/yaw、gyro[3]、accel[3]、q[4]、温度）
    bool online_;               // 在线标志
    err_t init_error_;
    uint8_t raw_data[8];
} IMUCAN_t;

extern imu_t imu;               // 兼容旧驱动的全局数据
```

#### 核心接口

```c
err_t imu_can_init(IMUCAN_t *self, STM32CAN_t *can,
                   uint8_t can_id, uint8_t mst_id);   // 绑定 CAN 并订阅 RX（1 个订阅槽）
err_t imu_can_start(IMUCAN_t *self);                  // 记录任务句柄、启动接收
void  imu_can_update(IMUCAN_t *self, uint32_t timeout_ms); // 任务周期调用：等通知 → 更新 data/online_
```

#### 寄存器 / 校准命令

```c
void imu_write_reg(uint8_t reg_id, uint32_t data);   // 写寄存器（CMD_WRITE）
void imu_read_reg(uint8_t reg_id);                   // 读寄存器（CMD_READ）
void imu_reboot(void);                               // 复位（REBOOT_IMU）
void imu_accel_calibration(void);                    // 加速度计校准
void imu_gyro_calibration(void);                     // 陀螺仪校准
void imu_change_com_port(imu_com_port_e port);       // 切换输出接口（USB/RS485/CAN/VOFA）
void imu_set_active_mode_delay(uint32_t delay);      // 主动回传延时
void imu_change_to_active(void);                     // 切主动回传
void imu_change_to_request(void);                    // 切应答（询问）模式
void imu_set_baud(imu_baudrate_e baud);              // CAN 波特率（1M/500K/.../25K）
void imu_set_can_id(uint8_t can_id);                 // 修改命令 ID
void imu_set_mst_id(uint8_t mst_id);                 // 修改反馈 ID
void imu_save_parameters(void);                      // 保存参数（reg 254）
void imu_restore_settings(void);                     // 恢复出厂（reg 255）
void imu_request_accel(void);     // 应答模式下主动请求加速度
void imu_request_gyro(void);      // 请求角速度
void imu_request_euler(void);     // 请求欧拉角
void imu_request_quat(void);      // 请求四元数
void IMU_UpdateData(uint8_t *data);                  // 反馈帧分发（内部使用）
```

`reg_id_e` 寄存器编号与 `imu_read_reg` / `imu_write_reg` 配合使用：`ACCEL_DATA / GYRO_DATA / EULER_DATA / QUAT_DATA / SET_ZERO / ACCEL_CALI / GYRO_CALI / MAG_CALI / CHANGE_COM / SET_BAUD / SET_CAN_ID / SET_MST_ID / SAVE_PARAM(254) / RESTORE_SETTING(255)` 等。

---

### 18.2 RS485 链路（imu_rs485）

**文件**：`modules/dm_imu/imu_rs485.h` / `imu_rs485.c`

| 参数 | 值 |
|------|-----|
| 物理层 | UART5（RS485） |
| 波特率 | 921600, 8N1 |
| 帧头 / 帧尾 | `0x55` / `0x0A` |
| 聚合帧长 | 80 字节 = 3 × 19B 普通包 + 1 × 23B 扩展包 |
| 离线超时 | 20ms（`IMU485_OFFLINE_TIMEOUT_MS`） |

**聚合帧布局**（每包 `header, tag, slave_id, reg` + float 数据 + `crc16` + `tail`）：

| 偏移 | 包类型 | reg | 内容 |
|------|--------|-----|------|
| 0 | 普通包 19B | 1 | `accel[3]` |
| 19 | 普通包 19B | 2 | `gyro[3]` |
| 38 | 普通包 19B | 3 | `roll / pitch / yaw` |
| 57 | 扩展包 23B | 4 | `quaternion[4]` |

#### `IMU485_t` — RS485 IMU 对象

```c
typedef struct {
    TaskHandle_t thread_alert;      // 任务通知句柄
    dm_imu_t_t data;                // accel[3]/gyro[3]/roll/pitch/yaw/quaternion[4]
    bool online_;
    STM32UART_t uart_;              // 复用 bsp_uart 循环 DMA RX 通道
    err_t init_error_;
    uint8_t raw_frame[IMU485_FRAME_SIZE];
} IMU485_t;

extern dm_imu_t_t imu_485;          // 兼容旧控制代码的全局数据
```

#### 核心接口

```c
err_t imu_485_init(IMU485_t *self, UART_HandleTypeDef *uart_handle);
// 绑定 UART（本工程 &huart5）、80B DMA 缓冲，注册 RX 回调

err_t imu_485_start(IMU485_t *self);        // 启动循环 DMA 接收
void  imu_485_update(IMU485_t *self, uint32_t timeout_ms);
// 任务周期调用：等通知 → 累积 80B 聚合帧 → 解析 → 刷新 data/online_

void imu_485_data_unpack(const uint8_t *pData);  // 解析一帧完整 80B 聚合帧到 imu_485
```

**接收流程**：ISR 回调接受任意长度的 DMA 片段，累积四个协议包到 80 字节快照，**凑齐完整聚合帧才通知任务**；协议解析与浮点解码全部在任务上下文完成（同 DR16 的快照-通知模式）。

### 完整任务示例

```c
static IMU485_t instance;
err_t status = imu_485_init(&instance, &huart5);
imu_485_device = &instance;
instance.thread_alert = xTaskGetCurrentTaskHandle();
if (status == OK) status = imu_485_start(imu_485_device);

for (;;) {
    imu_485_update(imu_485_device, IMU485_OFFLINE_TIMEOUT_MS);
    // 使用 imu_485_device->data.pitch / .yaw / .accel[...] 等
}
```

---

## 19. VT13 图传遥控器（vt13）

**文件**：`modules/vt13/vt13.h` / `vt13.c`

### 协议说明

| 参数 | 值 |
|------|-----|
| 物理层 | USART3, 115200, 8N1（图传链路，与裁判系统共用串口） |
| 帧长 | `sizeof(vt13_data_t)` = 21 字节 |
| 帧头 | `0xA9 0x53`（`sof_1` / `sof_2`） |
| 通道原始值 | 364 ~ 1684（中值 1024，11 bit × 4 通道 + 滚轮 11 bit） |
| 校验 | 全帧 CRC16（复用 `arithmetic/referee` 的 `Verify_CRC16_Check_Sum`） |

### `vt13_data_t` — 原始帧结构（packed）

```c
typedef struct __attribute__((packed)) {
    uint8_t sof_1, sof_2;                       // 帧头 0xA9 0x53
    uint64_t ch_0 : 11;   uint64_t ch_1 : 11;   // 左右摇杆 4 通道（11 bit）
    uint64_t ch_2 : 11;   uint64_t ch_3 : 11;
    uint64_t mode_sw : 2;                       // 模式拨杆
    uint64_t pause : 1;   uint64_t fn_1 : 1;    // VT13 专有功能位
    uint64_t fn_2 : 1;
    uint64_t wheel : 11;                        // 滚轮
    uint64_t trigger : 1;                       // 扳机
    int16_t mouse_x, mouse_y, mouse_z;          // 鼠标 X/Y/滚轮
    uint8_t mouse_left : 2;  uint8_t mouse_right : 2;  uint8_t mouse_middle : 2;
    uint16_t ket;                               // 键盘位图
    uint16_t crc16;
} vt13_data_t;
```

### `vt13_cmd_rc_t` — 解码后的遥控器命令

```c
typedef struct __attribute__((packed)) {
    struct { vector2_t l, r; } ch;  // 摇杆，归一化 ±1（上/右为正）
    float wheel;                    // 滚轮，归一化 [-0.5, 0.5]
    vt13_cmd_switch_pos_t mode_sw;  // 模式拨杆（枚举在 comp_cmd.h 中）
    cmd_mouse_t mouse;              // 鼠标（保留原始左右键语义）
    bool mouse_middle;
    uint16_t W, S, A, D, shift, ctrl;         // 键盘键值（按下为 1，随帧更新）
    uint16_t Q, E, R, F, G, Z, X, C, V, B;
    uint16_t crc16;                 // 原始帧 CRC16
    vt13_func_t func;               // pause / fn_1 / fn_2 / trigger 功能位
    struct { uint8_t sof_1, sof_2; } frame;
} vt13_cmd_rc_t;
```

> VT13 专有功能位压缩在 `cmd_rc_t.res` 中跨板转发：`VT13_COMPAT_RES_PAUSE`（bit0）、`VT13_COMPAT_RES_FN_1`（bit1）、`VT13_COMPAT_RES_FN_2`（bit2）、`VT13_COMPAT_RES_TRIGGER`（bit3）、`VT13_COMPAT_RES_MOUSE_M`（bit4）。

### `vt13_t` — VT13 对象

```c
typedef struct {
    vt13_data_t data;         // 最近一次原始帧快照
    TaskHandle_t thread_alert; // 任务通知句柄（用户须在 Start 前赋值）
    vt13_cmd_rc_t cmd;        // 解码后的命令
    bool online_;             // 在线标志
    err_t init_error_;
    STM32UART_t uart_;        // UART DMA 接收通道（内部使用）
} vt13_t;
```

---

### API

---

#### `vt13_init` / `vt13_start` — 初始化与启动

```c
err_t vt13_init(vt13_t *vt13, UART_HandleTypeDef *uart_handle);
err_t vt13_start(vt13_t *vt13);
```

- `init`：清零对象，`STM32UART_Init` 绑定 UART（本工程 `&huart3`）和 `2 × 帧长` 的 DMA 缓冲，注册 RX 回调
- `start`：启动循环 DMA 接收。ISR 收到完整 21 字节帧后校验帧头与 CRC16，合法帧快照并通过 `SIGNAL_VT13_RAW_REDY` 通知任务

---

#### `vt13_update` — 任务周期更新

```c
void vt13_update(vt13_t *vt13, uint32_t timeout_ms);
```

- 第一个参数 `*vt13` —— VT13 对象指针
- 第二个参数 `timeout_ms` —— 超时时间（ms），超过未收到合法帧则 `online_ = false` 并清零 `cmd`，建议 20

---

#### 解码与转换辅助

```c
err_t vt13_parse_rc(const vt13_t *vt13, vt13_cmd_rc_t *rc);            // 解析原始帧
err_t vt13_cmd_rc_to_cmd_rc(const vt13_cmd_rc_t *vt13_rc, cmd_rc_t *rc);
// 将 VT13 命令映射为通用 cmd_rc_t（摇杆/拨杆/鼠标/键盘 + res 兼容位），
// 便于上层业务与 DR16 共用同一套控制代码

err_t vt13_handle_offline(const vt13_t *vt13, vt13_cmd_rc_t *rc);      // 离线清零
err_t vt13_restart(vt13_t *vt13);                                      // 重启接收
err_t vt13_start_dma_recv(vt13_t *vt13);                               // 重启 DMA 接收
bool  vt13_wait_dma_cplt(uint32_t timeout);                            // 等待一次 DMA 接收完成
```

---

### 完整任务示例

```c
static vt13_t instance;
err_t status = vt13_init(&instance, &huart3);
vt13 = &instance;
instance.thread_alert = xTaskGetCurrentTaskHandle();
if (status == OK) status = vt13_start(vt13);
ASSERT(status == OK);

for (;;)
    vt13_update(vt13, 20u);   // 20ms 超时判离线
```

> ⚠️ **串口冲突提示**：VT13 与裁判系统（`game_task`）配置在同一 USART3。当前 `task/CMakeLists.txt` 将 `vt13_task` 注释禁用，仅保留 `game_task` 接管 USART3；如需启用 VT13，必须先解决串口分配（bsp_uart 同一外设仅允许一个 RX 控制块，后注册者返回 `BUSY`）。

---

## 20. 增量式 PID 控制器

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

## 21. 位置式 PID 控制器

**文件**：`arithmetic/pid/pid_location/pid_location.h` / `.c`

### 算法公式

```
u(k) = Kp·e(k) + Ki·Σe(k)·dt + Kd·[e(k) - e(k-1)]/dt
```

---

### `PIDInstance` — PID 运行实例

```c
typedef struct {
    // === 配置参数（PIDInit 写入，调参入口） ===
    float Kp;                   // 比例系数
    float Ki;                   // 积分系数
    float Kd;                   // 微分系数
    float MaxOut;               // 输出限幅（绝对值）
    float DeadBand;             // 死区（误差绝对值小于此值禁用 PID）
    PID_Improvement_e Improve;  // 改进功能位或组合
    float IntegralLimit;        // 积分限幅
    float CoefA;                // 变速积分上限系数
    float CoefB;                // 变速积分下限系数
    float Output_LPF_RC;        // 输出低通 RC = 1/ωc
    float Derivative_LPF_RC;    // 微分低通 RC

    // === 运行时缓存（PIDCalculate 维护，勿在控制循环中直接改写） ===
    float Measure, Last_Measure;      // 本次/上次测量值
    float Err, Last_Err, Last_ITerm;  // 误差链
    float Pout, Iout, Dout, ITerm;    // 三项输出与积分项
    float Output, Last_Output, Last_Dout;
    float Ref;                        // 设定值
    uint32_t DWT_CNT;                 // DWT 时间戳
    float dt;                         // 采样周期（秒，DWT 自动测量）
    PID_ErrorHandler_t ERRORHandler;  // 堵转检测状态（ERRORCount / ERRORType）
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

### `PIDInit` — 初始化（栈上，显式参数）

```c
void PIDInit(PIDInstance *pid, float kp, float ki, float kd, float max_output,
             float max_integral, float deadzone,
             PID_Improvement_e improve_flags, float coef_a, float coef_b,
             float output_lpf_rc, float derivative_lpf_rc);
```

- 第一个参数 `*pid` —— `PIDInstance` 结构体指针，该结构体包含 PID 三参数、限幅值、改进标志和所有运行时状态，Init 时清零运行时状态并写入配置参数
- 第二~四个参数 `kp` / `ki` / `kd` —— PID 三参数，≥ 0
- 第五个参数 `max_output` —— 输出限幅绝对值，> 0
- 第六个参数 `max_integral` —— 积分限幅值，> 0
- 第七个参数 `deadzone` —— 死区范围，误差绝对值小于此值时 PID 不输出，≥ 0
- 第八个参数 `improve_flags` —— 改进功能使能标志，按位或组合 `PID_Improvement_e`
- 第九 / 十个参数 `coef_a` / `coef_b` —— 变速积分系数（未启用时传 0）
- 第十一 / 十二个参数 `output_lpf_rc` / `derivative_lpf_rc` —— 输出 / 微分低通滤波 RC（未启用时传 0）

> 旧版 `PID_Init_Config_s` 配置结构体仍在头文件中保留定义，但初始化 API 已改为显式参数直传，不再接收配置结构体。

---

### `PIDRegister` — 初始化（堆上动态分配）

```c
PIDInstance *PIDRegister(float kp, float ki, float kd, float max_output,
                         float max_integral, float deadzone,
                         PID_Improvement_e improve_flags, float coef_a,
                         float coef_b, float output_lpf_rc,
                         float derivative_lpf_rc);
```

参数含义与 `PIDInit` 相同，实例改为从堆上分配。

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
PIDInstance position_pid;

/* 方法 1：栈上初始化（显式参数） */
PIDInit(&position_pid,
    5.0f,              // Kp
    0.1f,              // Ki
    0.5f,              // Kd
    3000.0f,           // MaxOut 输出限幅
    1000.0f,           // max_integral 积分限幅
    1.0f,              // deadzone 死区
    PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_OutputFilter,
    0.0f, 0.0f,        // coef_a / coef_b（未启用变速积分传 0）
    0.05f, 0.1f);      // 输出 / 微分低通 RC

/* 方法 2：堆上初始化（动态分配） */
PIDInstance *pid = PIDRegister(5.0f, 0.1f, 0.5f, 3000.0f, 1000.0f, 1.0f,
                               PID_Trapezoid_Intergral, 0.0f, 0.0f, 0.05f, 0.1f);

/* 控制循环 */
float output = PIDCalculate(&position_pid, encoder_angle, target_angle);
motor_set_output(output);
```

底盘四轮速度环的实际配置见第 24.1 节 `chassis_speed_pid_init()`。

---

## 22. 裁判系统 CRC 校验

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

## 23. 底盘动力学（chassis_dynamics）

**文件**：`app/chassis/chassis_dynamics.h` / `chassis_dynamics.c`

实现麦克纳姆轮底盘的运动学逆解和基于姿态的重力前馈补偿。

### 物理参数

| 宏 | 值 | 说明 |
|----|-----|------|
| `G` | 20.0 | 车重，用于重力补偿力矩 |
| `R` | 0.076 | 麦克纳姆轮有效半径（m） |
| `K` | 1000.0 | 力→电机输出的力矩转换系数 |

### API

---

#### `chassis_dynamics_set_attitude` — 更新姿态角

```c
void chassis_dynamics_set_attitude(float yaw, float pitch, float roll,
                                   float motor_yaw, float motor_pitch);
```

- 第一个参数 `yaw` —— 偏航角（rad），本模块未使用，保留接口
- 第二个参数 `pitch` —— 俯仰角（rad），底盘前后倾斜角度
- 第三个参数 `roll` —— 横滚角（rad），底盘左右倾斜角度
- 第四个参数 `motor_yaw` —— 偏航角速度（rad/s），未使用
- 第五个参数 `motor_pitch` —— 机械俯仰角偏移（rad），机械结构固有俯仰偏置

**说明**：通常由 IMU 姿态解算周期调用，更新后供 `chassis_dynamics_feedforward` 计算（实际俯仰 = 测量俯仰 − 机械偏移）。

---

#### `chassis_dynamics_inverse` — 麦克纳姆轮运动学逆解

```c
void chassis_dynamics_inverse(float vx, float vy, float wz, float out[4]);
```

- 第一个参数 `vx` —— 底盘 X 方向线速度（m/s），正方向前进
- 第二个参数 `vy` —— 底盘 Y 方向线速度（m/s），正方向左移
- 第三个参数 `wz` —— 绕 Z 轴角速度（rad/s），正方向顺时针
- 第四个参数 `out[4]` —— 输出四轮目标角速度（rad/s）：`out[0]` 左前、`out[1]` 右前、`out[2]` 左后、`out[3]` 右后

**轮系布局**（俯视，45° 麦轮，系数 s = √2/2）：

```text
    前                    o[0] = -s·vx - s·vy + wz   （左前）
    4     3                 o[1] =  s·vx - s·vy + wz   （右前）
        \ /                   o[2] = -s·vx + s·vy + wz   （左后）
        / \                   o[3] =  s·vx + s·vy + wz   （右后）
    2     1
    后
```

---

#### `chassis_dynamics_feedforward` — 姿态重力前馈

```c
void chassis_dynamics_feedforward(float out_current[4]);
```

根据当前俯仰/横滚角计算重力在各轮产生的力矩补偿（`q = √2·R·K` 归一化，各轮按 `±G·sin(pitch/roll)` 组合），输出应**叠加到 PID 输出**上，提高斜坡工况响应速度。输出序号与 `chassis_dynamics_inverse` 一致。

### 使用示例

```c
/* IMU 任务或控制周期内更新姿态 */
chassis_dynamics_set_attitude(imu.yaw, imu.pitch, imu.roll, 0.0f, 0.0f);

/* 底盘控制周期内 */
chassis_dynamics_feedforward(torque_ff_current);                    // 重力前馈
chassis_dynamics_inverse(vx, vy, wz, motor_target_speed);           // 逆解
float cur = PIDCalculate(&pid_speed[i], feedback.speed_rpm,
                         motor_target_speed[i]) + torque_ff_current[i];
```

---

## 24. Calculate 控制层（calculate）

**文件**：`calculate/chassis_control_dr16|vt13/`、`calculate/joint_control_dr16|vt13/`

控制解算层按遥控器分为 DR16 与 VT13 两个变体目录，**二选一参与编译**（`calculate/CMakeLists.txt`，当前启用 DR16 变体）。两个变体接口同名，业务代码无感切换。

### 24.1 底盘控制（chassis_control）

**接口**：`chassis_control.h`

```c
#define CHASSIS_MOTOR_COUNT (4U)
typedef enum {          // 轮序
    CHASSIS_MOTOR_FL,   // 左前
    CHASSIS_MOTOR_FR,   // 右前
    CHASSIS_MOTOR_RL,   // 左后
    CHASSIS_MOTOR_RR    // 右后
} chassis_motor_index_e;

typedef struct { float vx, vy, wz; } chassis_control_command_t;  // 三自由度指令
typedef struct {
    chassis_control_command_t command;
    float command_limit;
    uint8_t enabled;
} chassis_control_state_t;

err_t chassis_control_init(void);   // 初始化底盘总线与电机（须在 CAN Start 之前）
void  chassis_speed_pid_init(void); // 初始化四轮速度环 PID
void  Chassis_Mode(void);           // 底盘模式控制（任务周期调用）
```

#### `chassis_control_init` — 初始化底盘总线与电机

1. `BSP_CAN_get_id(CAN2)` + `STM32CAN_GetInstance` 获取 CAN2 控制块
2. `dj_motor_bus_init(&chassis_bus, can2)` 注册 RX 订阅（占 1 槽）
3. `dj_motor_init` 注册 4 台 **M3508**（FL/FR/RL/RR 对应设备 ID 4/3/2/1，控制组 0x200，`reversed = false`）

#### `chassis_speed_pid_init` — 四轮速度环 PID 参数

| 轮 | Kp | Ki | Kd | MaxOut | Improve |
|----|-----|-----|-----|--------|---------|
| FL | 12.0 | 0 | 0 | 12000 | 积分限幅(3000) + 微分先行 + 输出滤波 + 微分滤波 |
| FR | 8.0  | 0 | 0 | 12000 | 同上 |
| RL | 8.0  | 0 | 0 | 12000 | 同上 |
| RR | 14.0 | 2.0 | 0 | 12000 | 同上 |

#### `Chassis_Mode` — 底盘模式控制

| 条件 | 行为 |
|------|------|
| `dr16 == NULL` 或遥控器离线 | `chassis_stop()`（`dj_motor_zero_and_flush` 发零帧） |
| 左拨杆 MID 且 `joint_enable_single == 1` | `chassis_control()`：摇杆映射为 `vx/vy/wz`（×3000）→ 麦轮逆解 → 四轮速度 PID → `dj_motor_set_command`（写齐自动发送 0x200） |
| 左拨杆 DOWN（DR16） | `chassis_small_gyro_control()`：固定 `wz = 1000`，左摇杆控制平移 |
| 左拨杆 UP / 其他；VT13 的 DOWN | `chassis_stop()` 安全停机 |

**速度指令映射**：`vx = -ch.l.y × 3000`，`vy = -ch.l.x × 3000`，`wz = ch.r.x × 3000`。PID 输出叠加 `chassis_dynamics_feedforward` 重力前馈后限幅 ±16384。DR16 下档小陀螺使用左摇杆比例 `5000`，固定 `wz = 1000`。

> ⚠️ `joint_enable_single` 由机械臂侧置位，实现"底盘仅在与机械臂联动状态允许"的互锁。

---

### 24.2 关节控制（joint_control）

**接口**：`joint_control.h` —— 六台达妙关节电机（CAN1，Motor1~6，命令 ID 0x10~0x15）。

#### 关节角度限位（rad）

| 关节 | 下限 | 上限 |
|------|-------|-------|
| Motor1 | -0.3 | 2.4 |
| Motor2 | -3.14 | 0.0 |
| Motor3 | 0.06 | 1.2 |
| Motor4 | 1.3 | 3.00 |
| Motor5 | -2.5 | -0.5 |
| Motor6 | -0.5 | 0.66 |

#### 全局状态与接口

```c
uint8_t joint_enable_single;  // 底盘联动使能标志（与底盘互锁）
uint8_t joint_mode;           // 关节控制子模式
uint8_t joint_send_phase;     // 发送分相计数（0/1/2 轮转）
Joint_mode_t joint_mode_t;    // 遥控器键值快照
float pos[num];               // 六关节目标位置（初值 {1.2, 0, 0, -2.7, -1.15, 0.5}）

void Joint_Mode(void);        // 关节模式控制（joint_task 1ms 周期调用）
void joint_down_ctrl(void);   // 子模式 1：下位姿态调整（摇杆微调 pos[0..2]）
void joint_up_ctrl(void);     // 子模式 2：上位姿态调整（摇杆微调 pos[3..5]）
void joint_enable(void);      // 逐台使能 6 台电机（间隔 osDelay(1)）
```

#### `Joint_Mode` — 模式控制逻辑（DR16 变体）

| 左拨杆 | 行为 |
|--------|------|
| UP | `joint_disable()`：逐台失能，`joint_mode = 0` |
| MID | `joint_enable()` 保持使能，`joint_mode = 0` |
| DOWN | 右拨杆选择子模式：MID → `joint_down_ctrl()`，DOWN → `joint_up_ctrl()`；随后 `joint_send_pos()` 发送位置指令 |
| 遥控离线 | `joint_disable()` 安全失能 |

**发送节流**：`joint_send_pos()` 按 `joint_send_phase` 分 3 相轮转，每周期只对 2 台电机调用 `pos_ctrl`（速度 0.5 rad/s），均衡 1ms 周期下的 CAN 负载。目标位置由 `JOINT_CONSTRAIN_TARGET_BY_ID` 宏按各关节限位约束。

#### VT13 变体差异

`joint_control_vt13` / `chassis_control_vt13` 将遥控源从 `DR16_t *dr16` 换为 `vt13_cmd_rc_t`（VT13 按键为按帧计数值），并额外提供云台舵机（TB6210）鼠标控制：

```c
void joint_mouse_ctrl(void);  // 鼠标 X 轴 → TB6210_angle ± 20（限幅 0~180）
void one_return(void);        // B 键 → pos[0] = 1.2f（关节 1 回预设位）
```

---

## 25. Task FreeRTOS 任务

### 任务总表

任务在 `Core/Src/freertos.c`（CubeMX）中由 `MX_FREERTOS_Init()` 创建；任务体在各 `task/*/` 目录提供强定义，覆盖 freertos.c 中的 weak 空实现。

| 任务名 | 入口函数 | 外设 | 优先级 / 栈 | 当前状态 | 全局数据 |
|--------|---------|------|-------------|----------|----------|
| `defaultTask` | `StartDefaultTask()` | - | Normal / 128×4 | 空转 | - |
| `dr16` | `dr16_task()` | USART2 | High / 128×4 | ✅ 启用 | `DR16_t *dr16` |
| `vt13` | `vt13_task()` | USART3 | High / 128×4 | ⛔ CMake 注释禁用 | `vt13_t *vt13` |
| `vofa` | `vofa_task()` | USART6 | Low / 256×4 | ✅ 启用 | `float data[3]` |
| `imu_can` | `imu_can_task()` | CAN1 | High / 128×4 | ✅ 启用 | `IMUCAN_t *imu_can_device` |
| `lx824` | `lx824_task()` | USART1 | Low / 256×4 | ✅ 启用 | `LX824_t *lx824` |
| `i6X` | `i6x_task()` | UART4 | High / 128×4 | ⛔ CMake 注释禁用 | `I6X_t *i6x` |
| `imu_485` | `imu_485_task()` | UART5 | High / 128×4 | ✅ 启用 | `IMU485_t *imu_485_device` |
| `gimbal` | `gimbal_task()` | - | Normal / 512×4 | 占位（空实现） | - |
| `chassis` | `chassis_task()` | CAN2 | Low / 256×4 | ✅ 启用 | `chassis_status`、`chassis_bus`、`chassis_motors[4]` |
| `joint` | `joint_task()` | CAN1 | Normal / 256×4 | ✅ 启用 | `motor[6]`、`pos[6]`、`joint_mode` 等 |
| `game` | `game_task()` | USART3 | Low / 128×4 | ✅ 启用 | `Game_t *game`、`custom_robot_data` |

**禁用说明**：`task/CMakeLists.txt` 中 `vt13_task` 与 `i6x_task` 被注释——`vt13` 与 `game` 共用 USART3（同一 UART 只能注册一个 RX 控制块），启用前必须解决串口分配。被禁用任务的 `osThreadNew` 仍会执行，落到 freertos.c 的 weak 空实现（`osDelay(1)` 循环），不影响系统运行。

### 关键任务实现要点

| 任务 | 周期 / 超时 | 初始化流程 |
|------|-------------|------------|
| `chassis_task` | `Chassis_Mode()` + 2ms | `chassis_control_init()` → `chassis_speed_pid_init()` → `STM32CAN_Start(&can2_instance)`；任一步失败置 `chassis_status` 并 `vTaskSuspend(NULL)` |
| `joint_task` | `Joint_Mode()` + 1ms（稳定节拍，避免 CAN 发送抖动） | `dm_motor_init()` 初始化 6 台 DM 电机对象后进入循环 |
| `lx824_task` | 100ms 轮询 | `LX824_Init(&huart1)` → `LX824_Start`；循环 `LX824_IdRead(0xFE)` 广播读 ID，结果存入 `debug_lx824_*` 调试变量 |
| `imu_can_task` / `imu_485_task` | 20ms 离线超时 | Init → 保存 `thread_alert` → Start → 周期 Update |
| `game_task` | `Game_Update(10)` | `remote_control_data_init()` → `Game_Init(&huart3)` → Start → 周期 Update |

### 任务通知信号位分配

见第 3.2 节 `component/comp_def.h`。所有模块的 ISR→任务通知统一使用 `xTaskNotifyFromISR(..., eSetBits)` + 任务侧 `xTaskNotifyWait`，信号位互不重叠。

### 全局数据共享说明

| 符号 | 类型 | 定义位置 | 说明 |
|------|------|---------|------|
| `dr16` | `DR16_t *` | `task/dr16_task/dr16_task.c` | DR16 遥控器状态 |
| `vt13` | `vt13_t *` | `task/vt13_task/vt13_task.c` | VT13 遥控器状态（任务禁用时无效） |
| `i6x` | `I6X_t *` | `task/i6x_task/i6x_task.c` | I6X 遥控器状态（任务禁用时无效） |
| `lx824` | `LX824_t *` | `task/lx824_task/lx824_task.c` | LX824 总线接口 |
| `game` | `Game_t *` | `task/game_task/game_task.c` | 裁判系统对象 |
| `custom_robot_data` | `CommuniCateTypeDef` | `modules/game/game.c` | 裁判系统数据 |
| `imu_485` | `dm_imu_t_t` | `modules/dm_imu/imu_rs485.c` | RS485 IMU 姿态数据 |
| `imu` | `imu_t` | `modules/dm_imu/imu_can.c` | CAN IMU 姿态数据 |
| `imu_can_device` / `imu_485_device` | `IMUCAN_t *` / `IMU485_t *` | `task/imu_can_task` / `task/imu_485_task` | IMU 对象指针 |
| `data[3]` | `float[]` | `task/vofa_task/vofa_task.c` | VOFA+ 上行通道（speed/angle/mit） |
| `chassis_bus` / `chassis_motors[4]` | `dj_motor_bus_t` / `dj_motor_t[]` | `calculate/chassis_control_dr16/chassis_control.c` | 底盘电机总线与实例 |
| `chassis_status` | `volatile err_t` | `task/chassis_task/chassis_task.c` | 底盘任务状态（调试/监控） |
| `motor[6]` / `pos[6]` | `motor_t[]` / `float[]` | `modules/motor/dm_motor/dm_motor_ctrl.c` / `calculate/joint_control_*/joint_control.c` | DM 电机对象 / 关节目标位置 |
| `joint_enable_single` | `uint8_t` | `calculate/joint_control_*/joint_control.c` | 机械臂→底盘联动使能互锁 |
| `can1_instance` / `can2_instance` | `STM32CAN_t` | `Core/Src/main.c` | CAN BSP 全局控制块 |

### main.c 启动序列

```text
HAL_Init → SystemClock_Config(168MHz) → MX_GPIO/DMA/CAN/UART/TIM 初始化
  → STM32CAN_Init(&can2_instance, &hcan2)     // BSP CAN 对象注册
  → STM32CAN_Init(&can1_instance, &hcan1)
  → STM32CAN_ConfigFilter(CAN1: Bank0+FIFO1 / CAN2: Bank14+FIFO0)
  → DWT_Init(168)
  → osKernelInitialize → MX_FREERTOS_Init（创建全部任务） → osKernelStart
```

> 注意：`STM32CAN_Init` / `ConfigFilter` 在调度器启动前完成；各电机/IMU 的 `SubscribeRx` 与 `STM32CAN_Start` 在各自任务内进行（Start 后禁止再订阅）。

---

## 26. 构建系统

### CMake 构建命令

```bash
# Debug（启用 ASSERT）
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build

# Release
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

# 或使用 presets
cmake --preset Debug
cmake --build --preset Debug
```

### 源码目录组织（CMakeLists）

| 目录 | 说明 |
|------|------|
| `component/` `bsp/` `arithmetic/` `modules/` `tool/` `app/` `calculate/` `task/` | 各自独立 `CMakeLists.txt`，由根文件 `add_subdirectory` 引入 |
| `calculate/CMakeLists.txt` | DR16 / VT13 控制变体**二选一**（当前启用 dr16 目录） |
| `task/CMakeLists.txt` | 任务源文件开关（当前 `vt13_task`、`i6x_task` 被注释） |
| `bsp/CMakeLists.txt` | `bsp_iic`、`bsp_usb` 当前被注释禁用 |

### 编译定义

| 定义 | 启用条件 | 作用 |
|------|---------|------|
| `MCU_DEBUG_BUILD` | Debug 配置 | 启用 `ASSERT` / `VERIFY` |

### FreeRTOS 配置要点（`FreeRTOSConfig.h`）

| 配置 | 值 | 作用 |
|------|-----|------|
| `configCHECK_FOR_STACK_OVERFLOW` | 2 | 栈溢出检测（钩子中关中断死循环） |
| `configUSE_MALLOC_FAILED_HOOK` | 1 | 堆分配失败钩子（关中断死循环） |

钩子实现位于 `Core/Src/freertos.c`（`vApplicationStackOverflowHook` / `vApplicationMallocFailedHook`），触发即说明任务栈配置或堆大小不足。

### 链接参数

```cmake
LINKER:-u,_printf_float  # 启用 printf %f 浮点输出
```

---

## 27. 快速上手指南

### 开发环境搭建

1. 安装 ARM GCC 工具链（arm-none-eabi-gcc ≥ 10.3）
2. 安装 CMake ≥ 3.22 与 Ninja
3. 安装 STM32CubeMX（可选，用于重新生成代码）
4. （可选）安装 OpenOCD 或 JLink 用于烧录调试

### 首次编译烧录

```bash
cd diankong
git submodule update --init --recursive   # DR16/I6X/LX824/Vofa/game 为 submodule
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
# 使用 JLink / OpenOCD 烧录 build/jie_max.elf
```

### 添加新硬件驱动

1. 在 `bsp/` 下新建目录
2. 实现 Init / Start / Stop / GetLastError 等接口
3. 在 `bsp/CMakeLists.txt` 添加源文件和头文件路径
4. 在 `Core/Src/freertos.c` 中创建对应任务（任务体写在 `task/` 下覆盖 weak 实现）
5. 如需任务通知，在 `component/comp_def.h` 分配新的信号位

### 切换遥控器变体（DR16 ↔ VT13）

1. 修改 `calculate/CMakeLists.txt`：将 `chassis_control_dr16` / `joint_control_dr16` 替换为对应 `*_vt13` 目录
2. 修改 `task/CMakeLists.txt`：启用 `vt13_task`、按需禁用 `i6x_task`
3. 处理 USART3 归属：VT13 与裁判系统共用串口，二者只能有一个绑定 RX（见第 19 节提示）
4. 重新编译

### VOFA+ 调试

1. 打开 VOFA+ 上位机，串口选择 USART6（115200）
2. 选择 firewater 协议
3. 发送 `speed=100!` 调整参数
4. 观察波形

---

## 28. 常见问题与调试

### 1. 遥控器 `online_` 始终 false

| 检查项 | 措施 |
|--------|------|
| 波特率 | DR16: 100kbps 8E2 / I6X: 115200 8N1 / VT13: 115200 8N1 |
| 电平 | SBUS 需反相电平转换（3.3V） |
| DMA 缓冲 | 是否足够大（≥ 2× 帧长） |
| 任务句柄 | `thread_alert` 是否在 Start 前赋值 |
| 任务是否编译 | `vt13_task` / `i6x_task` 当前在 `task/CMakeLists.txt` 中被注释 |

### 2. DMA FIFO 溢出

**现象**：`rx_fifo_overflow_count` 持续增长

**排查**：
- 任务优先级是否过低
- `RX_DMA_BUF_LEN` 或 `RX_FIFO_LEN` 是否过小
- 任务处理周期是否过长

### 3. ASSERT 触发死循环

**现象**：程序卡在 `verify_failed()` 的 while(1)

**排查**：在 `while(1)` 处打断点，查看调用栈，检查 ASSERT 条件。另注意 FreeRTOS 栈溢出 / malloc 失败钩子（第 26 节）同样表现为关中断死循环，需区分调用栈。

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
| 波特率 | CAN1/CAN2 均为 1Mbps（Prescaler=3，14TQ，见第 2 节）；与 DM 设备默认 1Mbps 匹配，接 DJ 电调时注意 C620/C610 出厂为 1Mbps 可用 |
| ID 匹配 | 过滤器配置为全接收；确认设备反馈 ID 在订阅者匹配表中 |
| 订阅时机 | `STM32CAN_SubscribeRx` 必须在 `STM32CAN_Start` 之前，Start 后注册返回 `STATE_ERR` |
| 发送 BUSY | 发送不排队，邮箱满返回 `BUSY` 属正常，下周期重发即可 |
| ID 冲突 | CAN1 上 DM IMU（0x10/0x20）与 DM 电机（0x10~0x15/0x20~0x25）默认 ID 重叠，混挂时必须重新分配 |

### 6. DJI 电机"不发帧"

**现象**：`dj_motor_set_command` 返回 OK 但总线上没有 0x200 帧

**原因**：写齐再发机制——控制组内 4 台电机必须**全部**在本周期写过命令才打包发送。

**措施**：每周期对组内所有电机调用 `set_command`；调试单电机时改用 `dj_motor_force_flush_group`；停机用 `dj_motor_zero_and_flush`。

### 7. DM 电机无反馈数据

**原因**：`dm_motor_attach_can()` 当前无调用点，DM 反馈回调未订阅，`motor[].para` 不会更新（关节控制仅使用发送通路）。

**措施**：在 CAN1 Start 之前调用 `dm_motor_attach_can(&can1_instance)`，并处理与 DM IMU 的 ID 冲突（见问题 5）；寄存器参数读取需在任务中周期调用 `read_all_motor_data`。

### 8. USART3 串口冲突（VT13 与裁判系统）

**现象**：后初始化的一方 `STM32UART_Init` 返回 `BUSY`，任务 ASSERT 失效退出。

**现状**：`vt13_task` 在 CMake 中禁用，USART3 由 `game_task` 独占。启用 VT13 前必须二选一（或改用图传链路共享方案并合并解析）。

### 9. 变长帧发送偶发丢帧 / 返回 BUSY

**原因**：`STM32UARTFrameTx_Write` 在"一帧在发 + 一帧已排队"时拒绝新帧（不覆盖、不排队第三帧），属设计行为。

**措施**：控制调用节奏（VOFA 建议 ≥10ms 周期）；检查返回值后下周期重试；不要在中断里阻塞等待。

---

> **文档版本**：v2.1（完整函数参考版，同步底盘 / 机械臂 / IMU / VT13 代码）  
> **项目名称**：Diankong（电控）  
> **战队**：FHU RoboMaster 飞虎战队  
> **MCU**：STM32F405RG @ 168 MHz  
> **RTOS**：FreeRTOS (CMSIS-RTOS v2)  
> **CubeMX**：jie_max.ioc  
> **协议**：RoboMaster 2026 V1.1.0  
> **更新日期**：2026-09-12
