# UART hardware double-buffer TX

## Goal
Preserve bsp_uart style while replacing normal-mode software TX buffering with STM32 hardware DMA double buffering and adapting modules.

## Phases
1. Read BSP, DMA/HAL behavior, and module callers — complete.
2. Implement hardware double-buffer TX and update associated modules/documentation — complete.
3. Build and verify transfer state transitions; review diff — complete.

## Validation outcome
- ARM compilation with -Werror passed for the BSP, both modules, and both associated tasks.
- Host regression passed, including queued retry after failed continuation.
- Scoped diff checks passed; unrelated concurrent edits were preserved.
- Full firmware build remains blocked by unrelated baseline/link and concurrently removed calculation sources. No board timing test was performed.

## Constraints
- Preserve existing unrelated user changes in .settings.
- Validate variable-length sends and idle behavior against STM32F4 DBM semantics.
- Default design: hardware DBM is continuous and fixed-size; Vofa/LX824 retain finite frame semantics through STM32UARTFrameTx.

## Errors
- Baseline Debug build already fails to link: task/chassis_task/chassis_task.c references undefined DR16_GetSnapshot. Keep this unrelated defect separate from UART verification.
- A Python edit matched no section because PowerShell piped Chinese text using the default encoding. No write occurred; subsequent scripts explicitly use UTF-8.
- Concurrent work removed calculate/chassis_control/chassis_kinematics.c and other calculation sources. Full build now stops on missing sources; use existing compile commands for targeted ARM builds.
- Documentation patch rejected a delete/add of the same path; replaced the document using a UTF-8 write instead.
