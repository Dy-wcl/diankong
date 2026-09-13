# 底盘任务 (Chassis Task)

## 概述

底盘任务负责初始化底盘电机系统并周期性执行底盘运动控制。任务使用新版 `dj_motor` 模块进行电机管理，控制周期为 2ms。

## 文件结构

```
task/chassis_task/
├── chassis_task.h     # 任务头文件
├── chassis_task.c     # 任务实现
└── README.md          # 本文件
```

## 依赖关系

```
chassis_task.c
    ├── chassis_control.h         (底盘控制模块)
    ├── bsp_can.h                 (CAN 总线 BSP)
    └── FreeRTOS.h                (RTOS)

chassis_control 模块
    ├── dj_motor_ctrl.h           (电机驱动)
    ├── pid_location.h            (PID 控制器)
    └── vt13.h                    (遥控器协议)
```

## 任务初始化流程

### 在任务内部执行的初始化

`start_chassis_task()` 函数在任务启动后依次执行：

```c
void start_chassis_task(void *argument) {
  /* 1. 初始化底盘总线与电机 */
  chassis_status = chassis_control_init();
  if (chassis_status != OK) {
    vTaskSuspend(NULL);  // 失败则挂起任务
    return;
  }

  /* 2. 初始化 PID 参数 */
  chassis_speed_pid_init();

  /* 3. 启动 CAN2 */
  err_t can_start_result = STM32CAN_Start(&can2_instance);
  if (can_start_result != OK) {
    chassis_status = can_start_result;
    vTaskSuspend(NULL);
    return;
  }

  /* 4. 进入控制循环 */
  while (1) {
    Chassis_Mode();               // 底盘控制
    vTaskDelay(pdMS_TO_TICKS(2U)); // 2ms 周期
  }
}
```

### 初始化顺序说明

**重要：** 以下顺序必须严格遵守：

1. ✅ `chassis_control_init()` - 注册电机到总线
2. ✅ `chassis_speed_pid_init()` - 配置 PID 参数
3. ✅ `STM32CAN_Start()` - 启动 CAN（之后禁止注册新电机）
4. ✅ 进入控制循环

## 主程序集成

### 在 main.c 中的准备工作

在启动 FreeRTOS 之前，需要完成以下初始化：

```c
#include "bsp_can.h"
#include "can.h"

/* 全局 CAN2 实例（需要在 chassis_task 之前初始化） */
STM32CAN_t can2_instance;

int main(void) {
  /* HAL 初始化 */
  HAL_Init();
  SystemClock_Config();
  
  /* 外设初始化 */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();  // 底盘使用 CAN2
  
  /* 初始化 BSP CAN2（必须在任务创建之前） */
  err_t result = STM32CAN_Init(&can2_instance, &hcan2);
  if (result != OK) {
    Error_Handler();
  }
  
  /* 配置 CAN2 过滤器（接受所有标准帧） */
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
  STM32CAN_ConfigFilter(&can2_instance, &filter);
  
  /* 创建底盘任务 */
  osThreadId_t chassis_task_handle = osThreadNew(
      start_chassis_task,
      NULL,
      &chassis_task_attributes
  );
  
  /* 启动 RTOS 调度器 */
  osKernelStart();
  
  while (1) {
    // 不应该到达这里
  }
}
```

### FreeRTOS 任务配置示例

在 `freertos.c` 或任务配置文件中：

```c
/* 底盘任务属性 */
const osThreadAttr_t chassis_task_attributes = {
  .name = "ChassisTask",
  .stack_size = 512 * 4,        // 2KB 栈空间
  .priority = (osPriority_t) osPriorityNormal,
};
```

## 任务状态监控

### chassis_status 变量

```c
extern volatile err_t chassis_status;
```

**用途：** 记录底盘任务最近一次的初始化或控制状态

**可能的值：**
- `PENDING` - 初始状态（任务尚未初始化）
- `OK` - 初始化成功，正常运行
- `PTR_NULL` - CAN 实例未初始化
- `NOT_FOUND` - CAN2 外设未找到
- `FULL` - 电机注册槽位已满
- `STATE_ERR` - 初始化顺序错误或状态异常
- 其他错误码 - 具体的初始化失败原因

### 在其他任务中监控底盘状态

```c
#include "chassis_task.h"

void monitor_task(void *argument) {
  while (1) {
    if (chassis_status != OK) {
      // 底盘任务异常，记录或处理
      log_error("Chassis error: %d", chassis_status);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
```

## 控制逻辑

### 遥控器模式映射

底盘控制逻辑在 `chassis_control.c` 的 `Chassis_Mode()` 中实现：

| 模式开关 | 状态 | 底盘行为 |
|---------|------|---------|
| 中档 (MID) | `joint_enable_single == 1` | 正常控制 |
| 中档 (MID) | `joint_enable_single == 0` | 停止 |
| 上档 (UP) | - | 停止 |
| 下档 (DOWN) | - | 停止（VT13；DR16 为小陀螺模式） |

### 控制输入

- **vx**：前后速度（左摇杆 Y 轴）
- **vy**：左右速度（左摇杆 X 轴）
- **wz**：旋转速度（右摇杆 X 轴）

## 故障排查

### 问题：任务启动后立即挂起

**可能原因：**
1. `can2_instance` 未初始化
2. `STM32CAN_Init()` 未在任务创建前调用
3. CAN2 外设未正确配置

**解决方案：**
- 检查 `main.c` 中的 `STM32CAN_Init(&can2_instance, &hcan2)` 是否在 `osThreadNew()` 之前调用
- 使用调试器检查 `chassis_status` 的值

### 问题：底盘不响应遥控器

**可能原因：**
1. 遥控器未在线
2. 模式开关不在中档
3. `joint_enable_single` 为 0

**解决方案：**
- 检查遥控器连接
- 确认遥控器模式开关在中档
- 检查 `joint_enable_single` 标志位

### 问题：电机不转或方向错误

**可能原因：**
1. 电机 `reversed` 参数配置错误
2. CAN 总线连接问题
3. 电机 ID 设置错误

**解决方案：**
- 在 `chassis_control_init()` 中调整 `reversed` 参数
- 检查 CAN 总线物理连接
- 确认电机 ID 拨码开关设置为 1-4

## 性能参数

| 参数 | 值 |
|------|-----|
| 控制周期 | 2ms (500Hz) |
| 任务优先级 | osPriorityNormal |
| 栈空间 | 2048 字节 |
| 电机数量 | 4 个 M3508 |
| CAN 总线 | CAN2 |
| 控制组 | 0x200 |

## 与旧版本的差异

### 初始化方式

**旧版 (jie_max):**
```c
dj_motor_system_init();        // 一次性初始化所有电机
dj_motor_speed_pid_init();
```

**新版 (diankong):**
```c
chassis_control_init();        // 初始化总线 + 注册电机
chassis_speed_pid_init();      // 初始化 PID
STM32CAN_Start(&can2_instance); // 启动 CAN
```

### 电机管理

**旧版：** 使用全局 `dj_motor[]` 数组和 ID 索引  
**新版：** 使用 `dj_motor_t` 实例和总线对象，面向对象设计

### 发送机制

**旧版：** 手动调用 `dj_motor_control_send(&hcan2)`  
**新版：** 写齐自动发送，无需手动调用

## 扩展功能示例

### 添加底盘状态指示灯

```c
void start_chassis_task(void *argument) {
  RM_UNUSED(argument);

  chassis_status = chassis_control_init();
  if (chassis_status != OK) {
    HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
    vTaskSuspend(NULL);
    return;
  }

  chassis_speed_pid_init();
  
  err_t can_start_result = STM32CAN_Start(&can2_instance);
  if (can_start_result != OK) {
    chassis_status = can_start_result;
    HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
    vTaskSuspend(NULL);
    return;
  }

  /* 初始化成功，点亮就绪指示灯 */
  HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, GPIO_PIN_SET);

  while (1) {
    Chassis_Mode();
    vTaskDelay(pdMS_TO_TICKS(2U));
  }
}
```

### 添加电机在线监控

```c
void start_chassis_task(void *argument) {
  RM_UNUSED(argument);

  /* 初始化代码... */

  uint32_t motor_offline_count = 0;

  while (1) {
    /* 检查电机在线状态（每 100 次循环检查一次） */
    if (++motor_offline_count >= 100) {
      motor_offline_count = 0;
      
      uint32_t now = HAL_GetTick();
      bool all_online = true;
      
      for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        if (!dj_motor_is_online(&chassis_motors[i], now, 20)) {
          all_online = false;
          chassis_status = TIMEOUT;
          break;
        }
      }
      
      if (all_online) {
        chassis_status = OK;
      }
    }

    Chassis_Mode();
    vTaskDelay(pdMS_TO_TICKS(2U));
  }
}
```

## 参考文档

- [底盘控制模块文档](../../calculate/chassis_control/README.md)
- [dj_motor 模块文档](../../modules/motor/dj_motor/README.md)
- [BSP CAN 文档](../../bsp/bsp_can/bsp_can.md)

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 2.0 | 2026-09-10 | 适配新版 dj_motor 模块 |
| 1.0 | - | 初始版本（使用旧版 API） |
