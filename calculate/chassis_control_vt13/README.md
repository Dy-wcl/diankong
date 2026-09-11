# 底盘控制模块 (Chassis Control)

## 概述

本模块实现了麦克纳姆轮底盘的运动控制，使用 `dj_motor` 模块管理四个 M3508 电机。

## 文件结构

```
calculate/chassis_control/
├── chassis_control.h       # 公开接口
├── chassis_control.c       # 实现
├── ADAPTATION.md          # 详细适配说明
└── README.md              # 本文件
```

## 快速集成

### 1. 初始化顺序

在主程序的初始化流程中按以下顺序调用：

```c
#include "chassis_control.h"
#include "bsp_can.h"

/* 全局 CAN 对象（需要在某处声明） */
STM32CAN_t can2_instance;

int main(void) {
  /* 1. HAL 初始化 */
  HAL_Init();
  SystemClock_Config();
  
  /* 2. 外设初始化 */
  MX_CAN2_Init();
  
  /* 3. BSP CAN 初始化 */
  STM32CAN_Init(&can2_instance, &hcan2);
  
  /* 4. 底盘控制模块初始化（必须在 CAN Start 之前） */
  err_t result = chassis_control_init();
  if (result != OK) {
    // 错误处理
    Error_Handler();
  }
  
  /* 5. PID 参数初始化 */
  chassis_speed_pid_init();
  
  /* 6. 启动 CAN（之后不能再注册新电机） */
  STM32CAN_Start(&can2_instance);
  
  /* 7. 启动 RTOS 或进入主循环 */
  osKernelStart();
}
```

### 2. 周期调用

在控制任务中周期调用 `Chassis_Mode()`：

```c
void ChassisControlTask(void *argument) {
  const TickType_t period = pdMS_TO_TICKS(2);  // 2ms 控制周期
  
  TickType_t last_wake = xTaskGetTickCount();
  
  for (;;) {
    Chassis_Mode();  // 根据遥控器模式控制底盘
    
    vTaskDelayUntil(&last_wake, period);
  }
}
```

## API 说明

### `err_t chassis_control_init(void)`

初始化底盘 CAN 总线和四个 M3508 电机。

**调用时机：** 必须在 `STM32CAN_Start()` 之前调用

**返回值：**
- `OK` - 初始化成功
- `NOT_FOUND` - CAN2 外设未找到
- `PTR_NULL` - CAN 实例未初始化
- 其他错误码 - 总线或电机初始化失败

### `void chassis_speed_pid_init(void)`

初始化四个底盘电机的速度环 PID 参数。

**调用时机：** 在 `chassis_control_init()` 之后，`STM32CAN_Start()` 之前或之后均可

### `void Chassis_Mode(void)`

底盘模式控制函数，根据遥控器模式开关控制底盘运动。

**控制逻辑：**
- 模式开关在中档 + `joint_enable_single == 1`：正常控制
- 模式开关在上档或下档：停止底盘

**调用时机：** 在控制任务中周期调用（推荐 2-5ms）

## 电机配置

### 硬件连接

| 电机位置 | 设备 ID | CAN 控制组 | 物理接口 |
|---------|--------|-----------|---------|
| 左前轮 (FL) | 1 | 0x200 | CAN2 |
| 右前轮 (FR) | 2 | 0x200 | CAN2 |
| 左后轮 (RL) | 3 | 0x200 | CAN2 |
| 右后轮 (RR) | 4 | 0x200 | CAN2 |

### 电机方向配置

当前所有电机的 `reversed` 参数均为 `false`。如果底盘运动方向不正确，需要在 `chassis_control.c` 的 `chassis_control_init()` 中修改：

```c
/* 示例：左侧电机反向 */
dj_motor_init(&chassis_motors[CHASSIS_MOTOR_FL], &chassis_bus,
              DJ_MOTOR_M3508, 1, true);  // 左前轮反向

dj_motor_init(&chassis_motors[CHASSIS_MOTOR_RL], &chassis_bus,
              DJ_MOTOR_M3508, 3, true);  // 左后轮反向
```

## PID 参数调整

在 `chassis_speed_pid_init()` 中调整各电机的 PID 参数：

```c
void chassis_speed_pid_init(void) {
  // FL: Kp=12, Ki=0, Kd=0, MaxOut=12000
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_FL], 12.0f, 0.0f, 0.0f, 12000.0f);
  
  // FR: Kp=8, Ki=0, Kd=0, MaxOut=12000
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_FR], 8.0f, 0.0f, 0.0f, 12000.0f);
  
  // RL: Kp=8, Ki=0, Kd=0, MaxOut=12000
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_RL], 8.0f, 0.0f, 0.0f, 12000.0f);
  
  // RR: Kp=14, Ki=2, Kd=0, MaxOut=12000
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_RR], 14.0f, 2.0f, 0.0f, 12000.0f);
}
```

## 麦克纳姆轮运动学

底盘使用标准的麦克纳姆轮运动学解算：

```
motor_FL = -vx + vy - wz
motor_FR =  vx + vy - wz
motor_RL =  vx - vy - wz
motor_RR = -vx - vy - wz
```

其中：
- `vx`：前后速度（遥控器左摇杆 Y 轴）
- `vy`：左右速度（遥控器左摇杆 X 轴）
- `wz`：旋转速度（遥控器右摇杆 X 轴）

## 依赖模块

- `dj_motor` - DJI 电机驱动模块
- `bsp_can` - CAN 总线 BSP
- `pid_location` - PID 控制器
- `vt13` - 遥控器协议

## 故障排查

### 问题：初始化失败

**检查项：**
1. 确认 CAN2 已正确初始化
2. 确认 `STM32CAN_Init(&can2_instance, &hcan2)` 已调用
3. 确认 `chassis_control_init()` 在 `STM32CAN_Start()` 之前调用

### 问题：底盘不动

**检查项：**
1. 遥控器模式开关是否在中档
2. `joint_enable_single` 是否为 1
3. 电机是否在线（使用 `dj_motor_is_online()` 检测）
4. CAN 总线连接是否正常

### 问题：运动方向错误

**解决方案：**
1. 调整电机 `reversed` 参数
2. 检查麦克纳姆轮安装方向
3. 确认遥控器通道映射

## 高级功能

### 添加电机在线检测

```c
static void chassis_control(void) {
  /* 检查电机在线状态 */
  uint32_t now = HAL_GetTick();
  for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
    if (!dj_motor_is_online(&chassis_motors[i], now, 20)) {
      // 电机离线，安全停机
      chassis_stop();
      return;
    }
  }
  
  /* 正常控制逻辑 */
  // ...
}
```

### 添加速度限幅

```c
static void chassis_control(void) {
  chassis_control_state_.command.vx = -vt13_cmd_rc.ch.l.x * 3000;
  chassis_control_state_.command.vy = -vt13_cmd_rc.ch.l.y * 3000;
  chassis_control_state_.command.wz = -vt13_cmd_rc.ch.r.x * 3000;
  
  /* 限制最大速度 */
  const float max_linear_speed = 5000.0f;
  const float max_angular_speed = 3000.0f;
  
  chassis_control_state_.command.vx = CONSTRAIN(chassis_control_state_.command.vx,
                                                 -max_linear_speed, max_linear_speed);
  chassis_control_state_.command.vy = CONSTRAIN(chassis_control_state_.command.vy,
                                                 -max_linear_speed, max_linear_speed);
  chassis_control_state_.command.wz = CONSTRAIN(chassis_control_state_.command.wz,
                                                 -max_angular_speed, max_angular_speed);
  
  /* 运动学解算 */
  // ...
}
```

## 更多信息

详细的适配说明和 API 对照请参考 [ADAPTATION.md](ADAPTATION.md)。
