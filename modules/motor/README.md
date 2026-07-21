# Motor

本目录包含 DJI 与 DM 电机协议层。实现按子目录组织：

- `dj_motor/dj_motor_def.h`、`dj_motor/dj_motor_drv.h/.c`、
  `dj_motor/dj_motor_ctrl.h/.c`
- `dm_motor/dm_motor_def.h`、`dm_motor/dm_motor_drv.h/.c`、
  `dm_motor/dm_motor_ctrl.h/.c`

CAN BSP 只传输完整帧并广播订阅者；两类协议保持独立的注册表、编解码和状态机。

DM MIT 量化直接使用 `tool/convert` 的 `float_to_uint` / `uint_to_float`；
公共转换模块统一负责参数检查、非有限值回退、越界钳位和精确端点。

## DJ 实例化 API（写齐再发）

应用层只包含 `dj_motor_ctrl.h`。调用方静态持有：

- `dj_motor_bus_t`：每条 `STM32CAN_t` 一条总线，`bus_init` 注册 1 个 RX 订阅；
- `dj_motor_t`：每个电机一个实例，`dj_motor_init` 推导路由并挂入控制组。

| 电机 | 设备 ID | 反馈 ID | 控制组 | 帧槽位 | 指令限幅 |
| --- | ---: | ---: | ---: | ---: | ---: |
| M3508/C620 | 1–4 | `0x200 + ID` | `0x200` | `ID - 1` | ±16384 |
| M3508/C620 | 5–8 | `0x200 + ID` | `0x1FF` | `ID - 5` | ±16384 |
| M2006/C610 | 1–4 | `0x200 + ID` | `0x200` | `ID - 1` | ±10000 |
| M2006/C610 | 5–8 | `0x200 + ID` | `0x1FF` | `ID - 5` | ±10000 |
| GM6020 | 1–4 | `0x204 + ID` | `0x1FF` | `ID - 1` | ±30000 |
| GM6020 | 5–7 | `0x204 + ID` | `0x2FF` | `ID - 5` | ±30000 |

### 发送语义

- `dj_motor_set_command`：限幅、可选反向，写入组 `tx_buff` 与 `pending_mask`；
  当 `pending_mask == group_mask` 时自动打包 8 字节大端帧并 `STM32CAN_Send`。
- 半写不发：组内未全部写过本轮命令时不发送。
- 发送失败：`pending` 保留，返回 BSP 错误码。
- `dj_motor_force_flush_group`：不要求写齐，按当前缓存立即发送。
- `dj_motor_zero_and_flush`：**安全停机唯一推荐路径**，清零已注册槽并立即发零帧。

### 公共接口

- `dj_motor_bus_init(bus, can)`
- `dj_motor_init(motor, bus, type, device_id, reversed)`
- `dj_motor_set_command(motor, command)`
- `dj_motor_force_flush_group(bus, group)`
- `dj_motor_zero_and_flush(bus, group)`
- `dj_motor_get_feedback(motor, feedback)`
- `dj_motor_is_online(motor, now, timeout)`

同 bus 禁止重复反馈 ID 与 `(group, slot)`；跨 bus 可复用设备 ID。
ISR 只解码原始整数与时间戳；任务侧 `get_feedback` 做方向与物理量换算。

## DM

DM 当前生产实现由 `dm_motor_def`、`dm_motor_drv`、`dm_motor_ctrl` 五文件
组成，保留全局六路 `motor[]`，支持 MIT/POS/SPD/PSI 控制和寄存器读写。

## 底盘安全边界

底盘固定四个 M3508 实例挂 CAN2 的 `DJ_MOTOR_GROUP_200`。非 MID、非法拨杆、
遥控离线、任一轮超过 20ms 无反馈或初始化失败时，调用
`dj_motor_zero_and_flush(..., GROUP_200)`。`CHASSIS_ACTUATION_ENABLED=0`，
四路速度 PID 的 `kp/ki/kd` 全为 0，只允许安全收发验证。

## 初始化顺序

对象初始化 → 过滤器 → `dj_motor_bus_init` / `dj_motor_init` → DM 订阅 →
CAN 启动 → PID → `zero_and_flush` 零帧。

## 验证

```powershell
cmake -P modules/motor/dj_motor/tests/check_layout.cmake

cmake --preset Debug
cmake --build --preset Debug --clean-first --parallel 2
```

干净构建应生成 `dj_motor_drv.c.obj`、`dj_motor_ctrl.c.obj`，不生成旧
`dj_motor.c.obj`。
