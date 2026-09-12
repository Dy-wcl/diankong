# IMU CAN adaptation

## Goal
Adapt modules/dm_imu/imu_can and task/imu_can_task to bsp_can using the existing DR16/UART module and task conventions.

## Phases
1. Inspect CAN BSP, reference modules/tasks, IMU protocol and build integration — complete.
2. Implement CAN module and task integration — complete.
3. Build, verify receive/command behavior, and review scoped diff — complete.

## Constraints
- Preserve existing staged and unstaged changes, including the recent IMU RS485 work.
- Use the actual workspace E:/RM/libxr_exe/diankong.
- Do not flash hardware.

## Errors
- None.
