# 底盘系统完整适配总结

## 概述

已完成将底盘控制系统从旧版 `dj_motor` API 迁移到新版模块化设计。新系统采用面向对象的电机管理、写齐自动发送机制和规范的初始化流程。

## 修改文件清单

### 1. 底盘控制模块 (calculate/chassis_control/)

| 文件 | 状态 | 主要变更 |
|------|------|---------|
| `chassis_control.h` | ✅ 已更新 | 新增初始化函数、电机索引枚举 |
| `chassis_control.c` | ✅ 重构 | 适配新版 dj_motor API |
| `ADAPTATION.md` | ✅ 新增 | 详细的适配说明和 API 对照表 |
| `README.md` | ✅ 新增 | 使用指南和故障排查 |

**关键改进：**
- 使用 `dj_motor_bus_t` 和 `dj_motor_t` 实例管理
- 采用 `dj_motor_get_feedback()` 获取一致性反馈快照
- 使用 `dj_motor_set_command()` 实现写齐自动发送
- 使用 `dj_motor_zero_and_flush()` 安全停机

### 2. 底盘任务模块 (task/chassis_task/)

| 文件 | 状态 | 主要变更 |
|------|------|---------|
| `chassis_task.h` | ✅ 已更新 | 规范函数名和注释 |
| `chassis_task.c` | ✅ 重构 | 适配新初始化流程 |
| `README.md` | ✅ 新增 | 完整的任务集成文档 |

**关键改进：**
- 任务内初始化：`chassis_control_init()` → `chassis_speed_pid_init()` → `STM32CAN_Start()`
- 错误处理：初始化失败时挂起任务并记录 `chassis_status`
- 简化控制循环：直接调用 `Chassis_Mode()`

### 3. BSP CAN 模块 (bsp/bsp_can/)

| 文件 | 状态 | 主要变更 |
|------|------|---------|
| `bsp_can.h` | ✅ 已更新 | 新增 `STM32CAN_GetInstance()` |
| `bsp_can.c` | ✅ 已更新 | 实现实例访问函数 |

**关键改进：**
- 提供公开 API 访问 CAN 实例，替代直接访问 static 数组

## 完整的初始化流程

### 主程序 (main.c)

```c
#include "bsp_can.h"
#include "can.h"
#include "chassis_task.h"

/* 全局 CAN2 实例 */
STM32CAN_t can2_instance;

int main(void) {
  /* 1. HAL 初始化 */
  HAL_Init();
  SystemClock_Config();
  
  /* 2. 外设初始化 */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  
  /* 3. 初始化 BSP CAN2 */
  err_t result = STM32CAN_Init(&can2_instance, &hcan2);
  if (result != OK) {
    Error_Handler();
  }
  
  /* 4. 配置 CAN2 过滤器 */
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
  
  /* 5. 创建底盘任务 */
  osThreadNew(start_chassis_task, NULL, &chassis_task_attributes);
  
  /* 6. 启动 RTOS */
  osKernelStart();
  
  while (1);
}
```

### 底盘任务 (chassis_task.c)

```c
void start_chassis_task(void *argument) {
  /* 1. 初始化底盘总线与电机 */
  chassis_status = chassis_control_init();
  if (chassis_status != OK) {
    vTaskSuspend(NULL);
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

  /* 4. 控制循环 */
  while (1) {
    Chassis_Mode();
    vTaskDelay(pdMS_TO_TICKS(2U));
  }
}
```

### 底盘控制 (chassis_control.c)

```c
err_t chassis_control_init(void) {
  /* 1. 获取 CAN2 实例 */
  BSP_CAN_t can_id = BSP_CAN_get_id(CAN2);
  if (can_id == BSP_CAN_ID_ERROR) {
    return NOT_FOUND;
  }

  STM32CAN_t *can2 = STM32CAN_GetInstance(can_id);
  if (can2 == NULL) {
    return PTR_NULL;
  }

  /* 2. 初始化底盘总线 */
  err_t result = dj_motor_bus_init(&chassis_bus, can2);
  if (result != OK) {
    return result;
  }

  /* 3. 注册四个 M3508 电机 */
  result = dj_motor_init(&chassis_motors[CHASSIS_MOTOR_FL], &chassis_bus,
                         DJ_MOTOR_M3508, 1, false);
  if (result != OK) return result;
  
  // ... 依次注册其他三个电机
  
  return OK;
}
```

## 初始化时序图

```
main.c                    chassis_task.c           chassis_control.c       dj_motor
  |                            |                          |                    |
  |-- HAL_Init() ------------>|                          |                    |
  |                            |                          |                    |
  |-- MX_CAN2_Init() -------->|                          |                    |
  |                            |                          |                    |
  |-- STM32CAN_Init() ------->|                          |                    |
  |                            |                          |                    |
  |-- osThreadNew() --------->|                          |                    |
  |                            |                          |                    |
  |-- osKernelStart() ------->|                          |                    |
  |                            |                          |                    |
  |                            |-- chassis_control_init()->|                  |
  |                            |                          |                    |
  |                            |                          |-- dj_motor_bus_init()->
  |                            |                          |                    |
  |                            |                          |-- dj_motor_init() ->
  |                            |                          |    (x4 电机)       |
  |                            |                          |                    |
  |                            |<-- OK -------------------|                    |
  |                            |                          |                    |
  |                            |-- chassis_speed_pid_init()->                 |
  |                            |                          |                    |
  |                            |-- STM32CAN_Start() ----->|                    |
  |                            |                          |                    |
  |                            |                          |                    |
  |                            |== 进入控制循环 ==========|                    |
  |                            |                          |                    |
  |                            |-- Chassis_Mode() ------->|                    |
  |                            |                          |                    |
  |                            |                          |-- dj_motor_set_command()
  |                            |                          |    (写齐自动发送)   |
  |                            |                          |                    |
  |                            |<-------------------------|                    |
  |                            |                          |                    |
  |                            |-- vTaskDelay(2ms) ------>|                    |
  |                            |                          |                    |
```

## API 对照表

### 初始化 API

| 功能 | 旧版 API | 新版 API |
|------|----------|----------|
| 初始化电机系统 | `dj_motor_system_init()` | `chassis_control_init()` + `STM32CAN_Start()` |
| 初始化 PID | `dj_motor_speed_pid_init()` | `chassis_speed_pid_init()` |

### 控制 API

| 功能 | 旧版 API | 新版 API |
|------|----------|----------|
| 获取反馈 | `dj_motor[id].feedback.rpm_speed` | `dj_motor_get_feedback(&motor, &feedback)` |
| 设置命令 | `dj_motor_set_current_by_id(id, current)` | `dj_motor_set_command(&motor, command)` |
| 发送控制帧 | `dj_motor_control_send(&hcan2)` | 自动发送（写齐触发） |
| 安全停机 | 手动置零 + 发送 | `dj_motor_zero_and_flush(&bus, group)` |

### 管理 API

| 功能 | 旧版 API | 新版 API |
|------|----------|----------|
| 总线初始化 | 无（隐式） | `dj_motor_bus_init(&bus, can)` |
| 电机注册 | 无（静态配置） | `dj_motor_init(&motor, &bus, type, id, reversed)` |
| 在线检测 | 无 | `dj_motor_is_online(&motor, tick, timeout)` |
| 获取 CAN 实例 | 直接访问 `stm32_can_map[]` | `STM32CAN_GetInstance(can_id)` |

## 主要改进

### 1. 面向对象设计

**旧版：** 全局数组 + ID 索引
```c
dj_motor_t dj_motor[DJ_MOTOR_MAX];
float speed = dj_motor[motor_id].feedback.rpm_speed;
```

**新版：** 对象实例 + 指针
```c
dj_motor_t chassis_motors[CHASSIS_MOTOR_COUNT];
dj_motor_feedback_t feedback;
dj_motor_get_feedback(&chassis_motors[index], &feedback);
```

### 2. 写齐自动发送

**旧版：** 手动发送
```c
for (uint8_t i = 0; i <= DJ_MOTOR4; i++) {
  dj_motor_pid_control_speed(i, motor_speed[i]);
}
dj_motor_control_send(&hcan2);  // 必须手动调用
```

**新版：** 自动发送
```c
for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
  chassis_motor_pid_control_speed(i, motor_target_speed[i]);
}
// 当所有 4 个电机都写入后自动发送
```

### 3. 安全停机

**旧版：**
```c
dj_motor_set_current_by_id(1, 0);
dj_motor_set_current_by_id(2, 0);
dj_motor_set_current_by_id(3, 0);
dj_motor_set_current_by_id(4, 0);
dj_motor_control_send(&hcan2);
```

**新版：**
```c
dj_motor_zero_and_flush(&chassis_bus, DJ_MOTOR_GROUP_200);
```

### 4. 一致性保证

**旧版：** 直接访问可能被中断修改的数据
```c
float speed = dj_motor[id].feedback.rpm_speed;
```

**新版：** 临界区保护的快照读取
```c
dj_motor_feedback_t feedback;
dj_motor_get_feedback(&motor, &feedback);  // 原子操作
float speed = feedback.speed_rpm;
```

## 配置说明

### 电机方向配置

在 `chassis_control_init()` 中配置 `reversed` 参数：

```c
/* 示例：左侧电机反向 */
dj_motor_init(&chassis_motors[CHASSIS_MOTOR_FL], &chassis_bus,
              DJ_MOTOR_M3508, 1, true);   // 左前轮反向

dj_motor_init(&chassis_motors[CHASSIS_MOTOR_FR], &chassis_bus,
              DJ_MOTOR_M3508, 2, false);  // 右前轮正向

dj_motor_init(&chassis_motors[CHASSIS_MOTOR_RL], &chassis_bus,
              DJ_MOTOR_M3508, 3, true);   // 左后轮反向

dj_motor_init(&chassis_motors[CHASSIS_MOTOR_RR], &chassis_bus,
              DJ_MOTOR_M3508, 4, false);  // 右后轮正向
```

### PID 参数调整

在 `chassis_speed_pid_init()` 中调整参数：

```c
void chassis_speed_pid_init(void) {
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_FL], 
                                12.0f, 0.0f, 0.0f, 12000.0f);
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_FR], 
                                8.0f, 0.0f, 0.0f, 12000.0f);
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_RL], 
                                8.0f, 0.0f, 0.0f, 12000.0f);
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_RR], 
                                14.0f, 2.0f, 0.0f, 12000.0f);
}
```

## 测试清单

### 初始化测试

- [ ] `chassis_control_init()` 返回 `OK`
- [ ] `chassis_status` 为 `OK`
- [ ] 任务未挂起
- [ ] 四个电机都成功注册

### 通信测试

- [ ] 能够接收电机反馈
- [ ] `dj_motor_get_feedback()` 返回 `OK`
- [ ] CAN 总线上能看到 0x200 控制帧
- [ ] 控制帧发送频率为 500Hz (2ms)

### 控制测试

- [ ] 遥控器中档时底盘响应
- [ ] 前后左右移动方向正确
- [ ] 旋转方向正确
- [ ] 模式开关上档或下档时底盘停止

### 安全测试

- [ ] 断开遥控器后底盘停止
- [ ] 模式切换时底盘立即响应
- [ ] 任务初始化失败时挂起
- [ ] 电机离线时能正确检测

## 故障排查

### 编译错误

**问题：** `STM32CAN_GetInstance` 未定义

**解决：** 确保使用更新后的 `bsp_can.h` 和 `bsp_can.c`

---

**问题：** `chassis_control_init` 未定义

**解决：** 确保包含了 `chassis_control.h` 并使用新版本

---

**问题：** `can2_instance` 未定义

**解决：** 在 `main.c` 中声明 `STM32CAN_t can2_instance;`

### 运行时错误

**问题：** 底盘任务启动后挂起

**排查步骤：**
1. 使用调试器查看 `chassis_status` 的值
2. 检查 `STM32CAN_Init()` 是否在任务创建前调用
3. 确认 CAN2 外设已正确初始化

---

**问题：** 底盘不响应遥控器

**排查步骤：**
1. 检查遥控器是否在线
2. 确认模式开关在中档
3. 检查 `joint_enable_single` 是否为 1
4. 使用调试器查看 `vt13_cmd_rc` 的值

---

**问题：** 电机方向错误

**解决：** 在 `chassis_control_init()` 中调整 `reversed` 参数

## 后续优化建议

### 1. 添加电机在线监控

```c
static bool check_motors_online(void) {
  uint32_t now = HAL_GetTick();
  for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
    if (!dj_motor_is_online(&chassis_motors[i], now, 20)) {
      return false;
    }
  }
  return true;
}
```

### 2. 添加遥控器超时保护

```c
static uint32_t last_rc_tick = 0;

void Chassis_Mode(void) {
  uint32_t now = HAL_GetTick();
  
  /* 遥控器超时检测（500ms） */
  if ((now - last_rc_tick) > 500) {
    chassis_stop();
    return;
  }
  
  /* 正常控制逻辑 */
  // ...
  
  last_rc_tick = now;
}
```

### 3. 添加速度斜坡

```c
static void apply_ramp(float *current, float target, float max_delta) {
  float delta = target - *current;
  if (delta > max_delta) {
    *current += max_delta;
  } else if (delta < -max_delta) {
    *current -= max_delta;
  } else {
    *current = target;
  }
}
```

## 参考文档

- [底盘控制模块](calculate/chassis_control/README.md)
- [底盘任务模块](task/chassis_task/README.md)
- [dj_motor 适配说明](calculate/chassis_control/ADAPTATION.md)

## 版本信息

- **适配日期：** 2026-09-10
- **适配版本：** v2.0
- **适配者：** Claude (Kiro AI Assistant)
- **参考项目：** E:\stm32cubemx exe\jie_max
