# 任务清单: chassis-control-facade

```yaml
@feature: chassis-control-facade
@created: 2026-07-20
@status: completed
@mode: R2
```

## 进度概览

| 完成 | 失败 | 跳过 | 总数 |
|------|------|------|------|
| 4 | 0 | 0 | 4 |

---

## 任务列表

### 1. 公共与运动学

- [√] 1.1 新增 `calculate/control_common/control_common.h` 共享控制状态 | depends_on: []
- [√] 1.2 实现 `calculate/chassis_control/chassis_kinematics.h/.c` 麦克纳姆混控 | depends_on: []

### 2. 底盘 Facade

- [√] 2.1 实现 `calculate/chassis_control/chassis_control.h/.c`（init/step/force_stop/get_status） | depends_on: [1.1, 1.2]
- [√] 2.2 调整 `calculate/CMakeLists.txt` include 并验证编译；补齐最小 `control_platform` 以满足 main 启动 | depends_on: [2.1]

---

## 执行日志

| 时间 | 事件 | 详情 |
|------|------|------|
| 2026-07-20 21:31 | 方案包 | 已创建 Facade 安全骨架方案 |
| 2026-07-20 21:40 | 开发 | 完成底盘/运动学/公共头/平台与 CMake |
| 2026-07-20 21:42 | 构建 | Debug 全量构建与链接通过 |

## 执行备注

> 主动输出宏默认 0；不自动提交。
> 为实现可链接固件，额外补齐最小 `control_platform`（CAN1/2 初始化、过滤器、三 Facade 启动顺序）。
> 遗留方案包 `202607202017_control-facade-refactor` 仍存在，未自动归档。
