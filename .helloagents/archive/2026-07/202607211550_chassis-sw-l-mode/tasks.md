# 任务清单: chassis-sw-l-mode

> **@status:** completed | 2026-07-21 15:54

```yaml
@feature: chassis-sw-l-mode
@created: 2026-07-21
@status: completed
@mode: R2
```

## 进度概览

| 完成 | 失败 | 跳过 | 总数 |
|------|------|------|------|
| 4 | 0 | 0 | 4 |

---

## 任务列表

### 1. 底盘左拨杆模式映射

- [√] 1.1 在 chassis_control.h 增加 CHASSIS_SWITCH_DOWN_YAW 并更新 enable 语义注释 | depends_on: []
- [√] 1.2 按 sw_l 分档重写 chassis_task 输入映射（UP 失能 / MID yaw=0 / DOWN yaw 常量） | depends_on: [1.1]
- [√] 1.3 修正 chassis_control.c 过时拨杆注释 | depends_on: [1.2]
- [√] 1.4 同步知识库 modules/chassis_control.md 与 CHANGELOG | depends_on: [1.3]
