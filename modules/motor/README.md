# Motor 模块

`motor` 包含 DJI 与 DM 两类 CAN 电机协议。两个模块共享对象式 `STM32CAN_t`，分别注册 RX 订阅者，因此可以同时监听同一条 CAN 总线。

## 目录职责

```text
motor/
├── dj_motor/
│   ├── dj_motor_def.h      数据类型和设备参数
│   ├── dj_motor_codec.*    DJI 纯协议编解码
│   ├── dj_motor_drv.*      CAN 发送和单电机反馈处理
│   └── dj_motor_ctrl.*     八电机默认配置、限流和 RX 订阅
└── dm_motor/
    ├── dm_motor_def.h      数据类型和参数寄存器镜像
    ├── dm_motor_codec.*    DM 量化、控制帧和反馈编解码
    ├── dm_motor_drv.*      控制模式与寄存器命令发送
    ├── dm_motor_param.*    45 项寄存器读取状态机
    └── dm_motor_ctrl.*     六电机默认配置和 RX 订阅
```

codec 文件不依赖 HAL，可在主机测试中直接验证协议字节。drv 和 ctrl 仅通过 `bsp_can.h` 使用 CAN，不直接调用 HAL。

## 初始化顺序

```c
static STM32CAN_t can1;

void motor_can_init(const CAN_FilterTypeDef *filter)
{
  STM32CAN_Init(&can1, &hcan1);
  STM32CAN_ConfigFilter(&can1, filter);

  dj_motor_system_init();
  dm_motor_init();
  dj_motor_attach_can(&can1);
  dm_motor_attach_can(&can1);

  STM32CAN_Start(&can1);
}
```

两个 `attach` 函数必须在 `STM32CAN_Start()` 前调用。回调运行在 HAL 中断上下文，只解析并写入对应电机状态，不执行阻塞操作。

## DJI 发送

DJI 四路电流统一使用对象式接口：

```c
const int16_t current[4] = {1000, -1000, 0, 0};
err_t error = dj_CAN_Send_Data(&can1, DJ_MOTOR_GROUP_1, current);
```

电流按高字节在前编码，DLC 固定为 8。`dj_motor_control_send()` 发送默认数组中 1 到 4 号电机的设定电流。

## DM 发送

DM 所有控制和寄存器命令接收 `STM32CAN_t *` 并返回 `err_t`：

```c
motor_t *joint = &motor[Motor1];
joint->ctrl.mode = pos_mode;
joint->ctrl.pos_set = 1.0f;
joint->ctrl.vel_set = 2.0f;

err_t error = dm_motor_enable(&can1, joint);
if (error == OK)
{
  error = dm_motor_ctrl_send(&can1, joint);
}
```

MIT/POS/SPD/PSI 的 CAN ID、DLC、缩放和 payload 字节序保持原协议行为。寄存器读取由 `read_all_motor_data()` 按 `read_flag` 推进，收到 `0x33` 响应后由 `receive_motor_data()` 更新参数镜像。

## 验证

主机协议测试：

```powershell
cmake -S tests -B build/tests -G Ninja -DCMAKE_C_COMPILER=E:/vscode/mingw64/bin/gcc.exe
cmake --build build/tests
ctest --test-dir build/tests --output-on-failure
```

固件交叉构建：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```
