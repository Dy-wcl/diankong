/**
 * @file dj_motor_def.h
 * @brief DJI / RoboMaster 电机模块共用数据类型定义。
 *
 * 本文件集中描述电机型号、控制组、CAN 路由、原始/换算反馈、控制组拼包状态，
 * 以及调用方静态分配的总线对象与电机实例布局。这里只保存协议数据与运行状态，
 * 不直接访问 CAN 外设。
 *
 * ## 地址模型（本仓库既有协议，与 C620 / GM6020 官方手册一致）
 *
 * | 型号 | 设备 ID | 反馈帧 ID | 控制组 ID | 帧内槽位 |
 * | --- | ---: | ---: | ---: | ---: |
 * | M3508 / M2006 | 1–4 | 0x200 + ID | 0x200 | ID - 1 |
 * | M3508 / M2006 | 5–8 | 0x200 + ID | 0x1FF | ID - 5 |
 * | GM6020 | 1–4 | 0x204 + ID | 0x1FF | ID - 1 |
 * | GM6020 | 5–7 | 0x204 + ID | 0x2FF | ID - 5 |
 *
 * GM6020 无设备 ID 8；其控制量为电压原始指令，限幅 ±30000，勿按“电流”理解。
 *
 * ## 发送语义（写齐再发）
 *
 * 控制组维护 group_mask（已注册成员）与 pending_mask（本轮已写命令成员）。
 * 当两者相等时，ctrl 层打包 8 字节大端帧并通过 bsp_can 发送。安全停机请使用
 * dj_motor_zero_and_flush()，不要依赖“碰巧写齐”。
 *
 * @note 位置在反馈快照中单位为 rad；输出轴角速度单位为 rad/s；命令为协议原始
 *       int16_t。逻辑方向与物理方向由实例的 reversed 标志区分。
 */
#ifndef DJ_MOTOR_DEF_H
#define DJ_MOTOR_DEF_H

#include "bsp_can.h"

#include <stdbool.h>
#include <stdint.h>

/** @name 容量与协议常量
 * @{
 */
/** 单条总线最多注册的电机数量（与电机指针表长度一致） */
#define DJ_MOTOR_BUS_CAPACITY (8U)
/** 每个控制帧容纳的电机槽位数（每槽 2 字节大端 int16） */
#define DJ_MOTOR_GROUP_SLOT_COUNT (4U)
/** 控制组数量：0x200、0x1FF、0x2FF 各一组 */
#define DJ_MOTOR_GROUP_COUNT (3U)
/** 编码器一圈分辨率（圈内原始值 0..8191） */
#define DJ_MOTOR_ENCODER_RESOLUTION (8192U)
/** M3508 / C620 电流指令绝对值限幅（协议原始值，非物理安培） */
#define DJ_MOTOR_M3508_LIMIT (16384)
/** M2006 / C610 电流指令绝对值限幅 */
#define DJ_MOTOR_M2006_LIMIT (10000)
/** GM6020 电压原始指令绝对值限幅 */
#define DJ_MOTOR_GM6020_LIMIT (30000)
/** M3508 减速比（转子转速 → 输出轴） */
#define DJ_MOTOR_M3508_REDUCTION_RATIO (19.0f)
/** M2006 减速比 */
#define DJ_MOTOR_M2006_REDUCTION_RATIO (36.0f)
/** GM6020 视为直驱，减速比为 1 */
#define DJ_MOTOR_GM6020_REDUCTION_RATIO (1.0f)
/** @} */

/**
 * @brief 支持的 DJI 电机型号。
 *
 * 型号决定：合法设备 ID 范围、反馈/控制帧路由、指令限幅，以及转速换算用的减速比。
 */
typedef enum {
  DJ_MOTOR_M3508 = 0, /**< 3508 电机 + C620 电调，电流控制 */
  DJ_MOTOR_M2006,     /**< 2006 电机 + C610 电调，电流控制 */
  DJ_MOTOR_GM6020     /**< 6020 电机，电压控制（无 ID8） */
} dj_motor_type_e;

/**
 * @brief 控制组标准帧 ID。
 *
 * 枚举值与实际 11 bit 标准帧 ID 数值一致，可直接作为 STM32CAN_Send 的 std_id。
 * 同一 CAN 上不同型号可能共享 0x1FF 控制组，但槽位不得冲突。
 */
typedef enum {
  DJ_MOTOR_GROUP_200 = 0x200, /**< M3508/M2006 设备 ID 1–4 */
  DJ_MOTOR_GROUP_1FF = 0x1FF, /**< M3508/M2006 ID 5–8，或 GM6020 ID 1–4 */
  DJ_MOTOR_GROUP_2FF = 0x2FF  /**< GM6020 设备 ID 5–7 */
} dj_motor_group_e;

/**
 * @brief 由型号与设备 ID 推导出的 CAN 路由。
 *
 * 在 dj_motor_init() 时写入实例，运行期只读。feedback_id 用于 RX 匹配，
 * group 与 slot 用于写入控制组拼包缓存。
 */
typedef struct {
  uint16_t feedback_id;   /**< 反馈标准帧 ID，如 0x201..0x20B */
  dj_motor_group_e group; /**< 所属控制组 ID */
  uint8_t slot;           /**< 控制帧内槽位编号 0..3 */
} dj_motor_route_t;

/**
 * @brief ISR 写入的原始反馈整数。
 *
 * 反馈帧布局（大端）：encoder[0:1]、rpm[2:3]、current[4:5]、temperature[6]，
 * 第 7 字节保留。ISR 路径只保存这些整数，不做浮点换算。
 */
typedef struct {
  uint16_t encoder;    /**< 编码器原始值，0..8191 */
  int16_t speed_rpm;   /**< 转子侧转速，单位接近 RPM */
  int16_t current;     /**< 电流/力矩相关原始量（型号相关） */
  uint8_t temperature; /**< 温度，单位 ℃ */
} dj_motor_raw_feedback_t;

/**
 * @brief 任务侧读取的一致反馈快照。
 *
 * 由 dj_motor_get_feedback() 在短临界区拷贝原始状态后，在任务上下文填充换算量。
 * reversed=true 时，speed_rpm、current、position_rad 为逻辑方向（已取反）；
 * encoder_raw 始终保持硬件编码器原始值。
 */
typedef struct {
  uint16_t encoder_raw;        /**< 硬件编码器原始值，不因 reverse 翻转 */
  int16_t speed_rpm;           /**< 逻辑方向下的转子 RPM */
  int16_t current;             /**< 逻辑方向下的电流原始量 */
  uint8_t temperature;         /**< 温度，单位 ℃ */
  float position_rad;          /**< 逻辑方向位置角，单位 rad，约 [0, 2π) 或取反 */
  float speed_rad_s;           /**< 输出轴角速度，单位 rad/s（已除减速比） */
  uint32_t last_feedback_tick; /**< 最近一次合法反馈的 HAL_GetTick */
  bool valid;                  /**< true 表示至少成功解析过一帧反馈 */
} dj_motor_feedback_t;

/** 电机实例前向声明，供控制组与总线结构引用。 */
typedef struct dj_motor dj_motor_t;

/**
 * @brief 单个控制组的拼包与成员状态。
 *
 * 对齐 RMMotor 的 MotorGroupState 思路：
 * - group_mask：bitN=1 表示槽 N 已有已注册电机；
 * - pending_mask：bitN=1 表示槽 N 在本轮已通过 set_command 写入；
 * - 当 pending_mask == group_mask 且 group_mask != 0 时触发自动发送；
 * - tx_buff 保存物理方向（已 reverse）的协议原始命令；未注册槽恒为 0。
 */
typedef struct {
  dj_motor_group_e id;                            /**< 本控制组的标准帧 ID */
  dj_motor_t *members[DJ_MOTOR_GROUP_SLOT_COUNT]; /**< 槽位 0..3 对应的电机指针 */
  uint8_t group_mask;   /**< 已注册成员位图，bit0 对应槽 0 */
  uint8_t pending_mask; /**< 本轮已写命令成员位图 */
  int16_t tx_buff[DJ_MOTOR_GROUP_SLOT_COUNT]; /**< 物理方向命令缓存，空槽为 0 */
} dj_motor_group_state_t;

/**
 * @brief 单条 CAN 总线上的 DJ 协议对象。
 *
 * 由调用方静态分配，生命周期须覆盖 CAN 运行期。dj_motor_bus_init() 会：
 * 清零对象、绑定 STM32CAN_t、初始化三组控制组 ID，并向该 CAN 注册一个
 * RX 订阅（context 指向本 bus）。同一 CAN 上不论有多少电机，只占用一个
 * bsp_can 订阅槽。
 */
typedef struct {
  STM32CAN_t *can;           /**< 绑定的 BSP CAN 设备，不可为 NULL */
  dj_motor_t *motors[DJ_MOTOR_BUS_CAPACITY]; /**< 注册顺序保存的电机指针表 */
  dj_motor_group_state_t groups[DJ_MOTOR_GROUP_COUNT]; /**< 0:0x200 1:0x1FF 2:0x2FF */
  uint8_t motor_count;       /**< 当前已注册电机数量，0..CAPACITY */
  bool initialized;          /**< true 表示 bus_init 已成功 */
} dj_motor_bus_t;

/**
 * @brief 单个 DJ 电机软件实例。
 *
 * 由调用方静态分配。初始化后不支持注销、换总线、换型号或换设备 ID。
 * command 保存逻辑方向最新目标；raw_feedback 由 RX 回调在中断上下文更新。
 */
struct dj_motor {
  dj_motor_bus_t *bus;                  /**< 所属总线，init 后只读 */
  dj_motor_type_e type;                 /**< 电机型号 */
  uint8_t device_id;                    /**< 设备 ID：M3508/M2006 为 1..8，GM6020 为 1..7 */
  bool reversed;                        /**< true 时命令与反馈速度/电流/角度取反 */
  bool initialized;                     /**< true 表示已成功注册到总线 */
  dj_motor_route_t route;               /**< 自动推导的反馈/控制路由 */
  int16_t command;                      /**< 逻辑方向最新命令（限幅后） */
  dj_motor_raw_feedback_t raw_feedback; /**< ISR 写入的最近原始反馈 */
  uint32_t last_feedback_tick;          /**< 最近反馈时间戳 */
  bool feedback_valid;                  /**< 是否至少收过一帧合法反馈 */
};

#endif /* DJ_MOTOR_DEF_H */
