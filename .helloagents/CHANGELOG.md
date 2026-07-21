# CHANGELOG

## 快速修改 / 功能

- **[chassis_task / chassis_control]**: 左拨杆 `sw_l` 分档：仅 UP 安全失能；MID 使能且 yaw=0；DOWN 使能且 yaw=`CHASSIS_SWITCH_DOWN_YAW(1.0f)`；平移仍映射左摇杆；不再用 `sw_r` 作 enable。 类型: 行为修正 文件: task/chassis_task/chassis_task.c、calculate/chassis_control/chassis_control.h、calculate/chassis_control/chassis_control.c、.helloagents/modules/chassis_control.md
- **[chassis_control]**: 为底盘控制 Facade 与逆运动学补充详细中文注释（文件头职责、结构体字段、安全零帧/状态机/step 流程），不改动控制逻辑。 类型: 文档/注释 文件: calculate/chassis_control/chassis_control.c:1-360、calculate/chassis_control/chassis_control.h:1-120、calculate/chassis_control/chassis_kinematics.c:1-45
- **[chassis_control]**: 新增底盘 Facade 与麦克纳姆运动学，对齐 gimbal 安全骨架；默认 `CHASSIS_ACTUATION_ENABLED=0`；补齐 `control_common` 与最小 `control_platform` 以满足启动顺序。 文件: calculate/chassis_control/*、calculate/control_common/control_common.h、calculate/control_platform/*、calculate/CMakeLists.txt
