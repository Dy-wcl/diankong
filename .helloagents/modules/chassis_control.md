# chassis_control

## 职责
固定拓扑麦克纳姆底盘控制：4 台 M3508（CAN2 / 0x200）、速度环、安全零帧。

## 接口
- `chassis_control_init(STM32CAN_t *can)` — 注册 M3508 ID1-4，要求 `BSP_CAN2`
- `chassis_control_step(const chassis_control_input_t *input, uint32_t now_tick)`
- `chassis_control_force_stop(void)`
- `chassis_control_get_status(chassis_control_status_t *status)`
- `chassis_kinematics_mecanum(...)` — 逆运动学纯函数

## 行为规范
- 默认 `CHASSIS_ACTUATION_ENABLED=0`，只允许零输出 / READY
- 反馈超时 20ms、源离线、未使能 → `dj_motor_zero_and_flush(GROUP_200)`
- 输入由 `chassis_task` 通过 DR16 快照映射，不直接读遥控器
- 左拨杆 `sw_l` 模式：`UP` 仅安全失能；`MID` 使能且 `yaw=0`；`DOWN` 使能且 `yaw=CHASSIS_SWITCH_DOWN_YAW(1.0f)`
- 电机设备 ID 固定 1–4 → 控制组固定 `0x200`；零帧路径与 init 注册一致

## 依赖
- `dj_motor_ctrl` / `pid_location` / `bsp_can` / `control_common`
