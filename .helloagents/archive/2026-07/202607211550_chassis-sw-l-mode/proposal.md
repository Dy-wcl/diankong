# 变更提案: chassis-sw-l-mode

## 元信息
```yaml
类型: 缺陷修复/行为修正
方案类型: implementation
优先级: P1
状态: 已确认
创建: 2026-07-21
复杂度: simple
路由: R2
```

---

## 1. 需求

### 背景
`chassis_task` 当前用右拨杆 `sw_r == MID` 作为 `enable`，其余档位全部安全失能。用户要求改为左拨杆 `sw_l` 分档：仅 UP 安全失能；MID/DOWN 走正常控制链；MID 时 kinematics yaw=0；DOWN 时 yaw 为归一化常量（选项1）。

### 目标
- 左拨杆 `sw_l` 决定底盘运行模式
- `CMD_SW_UP`：仅安全失能（`enable=false`）
- `CMD_SW_MID` / `CMD_SW_DOWN`：`enable=true`，保持正常 `chassis_control_step` 控制链
- MID：`input.yaw = 0`（逆解无旋转分量）
- DOWN：`input.yaw` 为归一化常量 `1.0f`（经 `CHASSIS_COMMAND_SCALE` 得约满量程旋转）
- 平移仍由左摇杆映射；源离线/非法拨杆走安全零输出
- 理清并保持：零帧固定 `DJ_MOTOR_GROUP_200`，与 init 注册 ID1–4 一致

### 约束条件
```yaml
性能约束: 底盘任务 2ms 周期，无动态分配
兼容性约束: 不改动 dj_motor 协议与 CAN 绑定
业务约束: CHASSIS_ACTUATION_ENABLED 默认仍为 0
安全约束: UP/ERR/掉线必须零输出；不得误使能
```

### 验收标准
- [ ] `sw_l==UP` 时 `enable=false`，step 进入 SAFE_DISABLED 零输出路径
- [ ] `sw_l==MID` 时 `enable=true` 且 `yaw==0`，forward/lateral 仍映射左摇杆
- [ ] `sw_l==DOWN` 时 `enable=true` 且 `yaw==CHASSIS_SWITCH_DOWN_YAW(1.0f)`
- [ ] 不再使用 `sw_r` 控制 enable
- [ ] 注释/知识库与代码一致；0x200 零帧说明保留

---

## 2. 方案

### 核心思路
模式映射放在任务层 `chassis_task.c`：根据 `command.sw_l` 填充 `chassis_control_input_t` 的 `enable`/`yaw`。控制层 `chassis_control_step` 的 `!enable → 零输出` 语义保留，仅更新过时注释。可选在 `chassis_control.h` 增加 `CHASSIS_SWITCH_DOWN_YAW` 宏，避免魔法数。

### 实现路径
1. 在 `chassis_control.h` 增加归一化 yaw 常量宏并更新 enable 语义注释
2. 重写 `chassis_task.c` 的拨杆映射：`sw_l` 分档
3. 修正 `chassis_control.c` 中 “拨杆非 MID” 等过时注释
4. 同步 `.helloagents/modules/chassis_control.md` 与 CHANGELOG

### 不改动范围
- `chassis_kinematics_mecanum` 公式本身
- `chassis_flush_zero_if_due` / 电机 ID 注册逻辑（已正确绑定 0x200）
- 右拨杆 `sw_r`（本轮不参与底盘模式）

### 风险评估
- 低：仅任务层映射与注释；安全路径仍由 `enable`/`source_online` 闸门保证
- 注意：DOWN 固定 yaw=1.0 为联调/模式量，真车启用闭环前需标定

---

## 3. 技术约束

- C11 / FreeRTOS / 现有 DR16 快照 API
- 不引入新依赖
