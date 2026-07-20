#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"
#include "comp_cmd.h"
#include "comp_utils.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// ==================== CAN 常量与类型描述 ====================

/**
 * @brief Classical CAN 单帧数据区最大字节数
 *
 * 对应 STM32 bxCAN DLC 上限；本 BSP 仅支持 Classical CAN，不支持 FDCAN。
 */
#define BSP_CAN_DATA_SIZE (8U)

/**
 * @brief 每个 CAN 对象可注册的 RX 订阅者容量
 *
 * 启动前通过 STM32CAN_SubscribeRx 注册；满员后返回 FULL。
 * 不支持运行期退订，也不使用堆内存。
 */
#define STM32CAN_RX_SUBSCRIBER_CAPACITY (4U)

/**
 * @brief CAN BSP 逻辑设备编号
 *
 * 随 CubeMX 启用的 CAN 宏展开生成枚举项。
 * 用于索引对象表 stm32_can_map[]。
 * BSP_CAN_NUMBER 为合法 ID 数量；BSP_CAN_ID_ERROR 表示无效外设地址。
 */
typedef enum
{
#ifdef CAN1
  BSP_CAN1,
#endif
#ifdef CAN2
  BSP_CAN2,
#endif
  BSP_CAN_NUMBER,   //!< 合法 CAN 数量（作数组上界）
  BSP_CAN_ID_ERROR  //!< 无效 ID / 未识别外设
} BSP_CAN_t;

/**
 * @brief CAN 帧快照（RX 路径构造，也可供上层转发）
 *
 * 在 HAL_CAN_GetRxMessage 成功后由 ISR 填充；
 * data_ 仅包含 DLC 有效字节，剩余字节已清零。
 * 回调内指针仅本次有效，延后处理须先拷贝到自有缓冲区。
 */
typedef struct
{
  uint32_t id_;                      //!< 帧 ID：标准帧为 StdId，扩展帧为 ExtId
  uint32_t ide_;                     //!< 标识符类型：CAN_ID_STD / CAN_ID_EXT
  uint32_t rtr_;                     //!< 远程帧标志：CAN_RTR_DATA / CAN_RTR_REMOTE
  uint32_t fifo_;                    //!< 来源 FIFO：CAN_RX_FIFO0 / CAN_RX_FIFO1
  uint8_t size_;                     //!< 有效数据长度 DLC，范围 0..8
  uint8_t data_[BSP_CAN_DATA_SIZE];  //!< 数据区；无效尾部字节已清零
} BSP_CAN_Frame_t;

/** 前向声明：RX 回调参数中需要完整对象类型。 */
typedef struct STM32CAN STM32CAN_t;

/**
 * @brief RX 订阅回调类型
 *
 * 通常在 HAL/ISR 路径中调用（FIFO pending 中断）。
 * @param self    触发回调的 CAN 控制块
 * @param frame   指向栈上帧快照，仅本次回调有效；延后处理必须先拷贝
 * @param context 订阅时传入的用户上下文，可为 NULL
 * @note 回调必须短且非阻塞；禁止在回调内做协议状态机、日志或长时间处理。
 */
typedef void (*STM32CAN_RxCallback_t)(STM32CAN_t *self,
                                      const BSP_CAN_Frame_t *frame,
                                      void *context);

/**
 * @brief 单个 RX 订阅槽
 *
 * 保存回调函数与用户上下文；相同 callback+context 重复注册保持幂等。
 */
typedef struct
{
  STM32CAN_RxCallback_t callback_;  //!< 数据到达时的用户回调（不可为 NULL）
  void *context_;                   //!< 用户上下文，生命周期须覆盖 CAN 运行期
} STM32CAN_RxSubscriber_t;

/**
 * @brief CAN 控制块
 *
 * 绑定一个 HAL CAN 句柄，管理固定容量 RX 订阅表与启动状态。
 * 发送不排队：邮箱满立即返回 BUSY，由上层在下一周期用最新数据重试。
 * 协议打包、帧 ID 路由归业务模块，本结构不承载电机协议。
 */
struct STM32CAN
{
  BSP_CAN_t id_;  //!< 逻辑 CAN 编号，用于挂入对象表
  CAN_HandleTypeDef *can_handle_;  //!< 对应 HAL CAN 句柄
  STM32CAN_RxSubscriber_t
      subscribers_[STM32CAN_RX_SUBSCRIBER_CAPACITY];  //!< 固定容量订阅表
  uint8_t subscriber_count_;  //!< 已注册订阅者数量 [0, CAPACITY]
  bool started_;              //!< true：已成功 Start，此后禁止再 Subscribe
  err_t last_error_;          //!< 最近一次操作错误码（OK / PTR_NULL / ...）
};

#ifdef __cplusplus
extern "C" {
#endif

// ==================== 公开接口 ====================

/**
 * @brief 由硬件外设地址映射到逻辑 CAN ID
 * @param addr CAN 寄存器基址（如 CAN1 / CAN2）
 * @return 对应 BSP_CAN_t；未识别或 NULL 返回 BSP_CAN_ID_ERROR
 */
BSP_CAN_t BSP_CAN_get_id(CAN_TypeDef *addr);

/**
 * @brief 绑定 CAN 控制块到指定 HAL 句柄
 *
 * 仅完成对象初始化与对象表注册，不启动外设、不配置过滤器。
 * 真正启动由 STM32CAN_Start() 完成；过滤器由 STM32CAN_ConfigFilter() 配置。
 * @param self        CAN 控制块（调用方分配，生命周期须覆盖使用期）
 * @param can_handle  HAL CAN 句柄，Instance 须为已启用的 CAN 外设
 * @return OK 成功；PTR_NULL / NOT_FOUND / BUSY 等失败码
 */
err_t STM32CAN_Init(STM32CAN_t *self, CAN_HandleTypeDef *can_handle);

/**
 * @brief 在启动前注册 RX 订阅者
 *
 * 回调按注册顺序在每帧到达时依次调用。
 * 相同 callback + context 重复注册幂等返回 OK，不占用新槽位。
 * @param self     CAN 控制块
 * @param callback 数据到达回调，不可为 NULL
 * @param context  用户上下文，可为 NULL；生命周期须覆盖运行期
 * @return OK 成功；PTR_NULL / STATE_ERR（已 Start）/ FULL（槽位已满）
 */
err_t STM32CAN_SubscribeRx(STM32CAN_t *self,
                           STM32CAN_RxCallback_t callback,
                           void *context);

/**
 * @brief 启动 CAN 外设并开启 RX FIFO 中断通知
 *
 * 若尚未处于 LISTENING，则调用 HAL_CAN_Start；
 * 随后激活 FIFO0/FIFO1 消息 pending 中断。
 * 成功后 started_ 置 true，此后 SubscribeRx 将返回 STATE_ERR。
 * @param self CAN 控制块
 * @return OK 成功；PTR_NULL / INIT_ERR 等
 */
err_t STM32CAN_Start(STM32CAN_t *self);

/**
 * @brief 配置 HAL CAN 过滤器
 *
 * 透传 HAL_CAN_ConfigFilter；过滤器结构体由调用方按业务填充。
 * 可在 Start 前后调用（取决于 HAL 与硬件状态要求）。
 * @param self   CAN 控制块
 * @param filter HAL 过滤器配置，不可为 NULL
 * @return OK 成功；PTR_NULL / FAILED
 */
err_t STM32CAN_ConfigFilter(STM32CAN_t *self,
                            const CAN_FilterTypeDef *filter);

/**
 * @brief 通过控制块发送一帧标准数据帧
 *
 * 仅支持标准 ID（0x000..0x7FF）、数据帧（非远程帧）、1..8 字节。
 * 无空邮箱立即返回 BUSY，不排队、不覆盖、不自动重试。
 * @param self   CAN 控制块
 * @param std_id 标准帧 ID，须 ≤ 0x7FF
 * @param data   待发数据指针，不可为 NULL
 * @param size   字节数，须在 1..BSP_CAN_DATA_SIZE
 * @return OK / BUSY / PTR_NULL / SIZE_ERR / OUT_OF_RANGE / FAILED 等
 */
err_t STM32CAN_Send(STM32CAN_t *self,
                    uint32_t std_id,
                    const uint8_t *data,
                    size_t size);

/**
 * @brief 通过已注册的 HAL 句柄适配发送
 *
 * 先按句柄反查已 Init 的控制块，再复用 STM32CAN_Send()。
 * 句柄未注册时返回 NOT_FOUND。供仍只持有 HAL 句柄的边界代码使用。
 * @param can_handle HAL CAN 句柄
 * @param std_id     标准帧 ID
 * @param data       待发数据
 * @param size       字节数
 * @return 见 STM32CAN_Send；未注册时 NOT_FOUND
 */
err_t STM32CAN_SendByHandle(CAN_HandleTypeDef *can_handle,
                            uint32_t std_id,
                            const uint8_t *data,
                            size_t size);

/**
 * @brief 向全部订阅者广播一帧 RX 快照
 *
 * 通常由内部 ISR 路径在构帧后调用；也可供测试/上层注入使用。
 * 校验 frame 非空且 size_ ≤ BSP_CAN_DATA_SIZE 后，按注册顺序调用各订阅回调。
 * @param self  CAN 控制块
 * @param frame 帧快照，不可为 NULL
 */
void STM32CAN_HandleRxFrame(STM32CAN_t *self,
                            const BSP_CAN_Frame_t *frame);

/**
 * @brief 读取 CAN 控制块最近一次错误码
 * @param self CAN 控制块
 * @return last_error_；self 为 NULL 时返回 PTR_NULL
 */
err_t STM32CAN_GetLastError(const STM32CAN_t *self);

#ifdef __cplusplus
}
#endif

#endif
