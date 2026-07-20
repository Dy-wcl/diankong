/**
 * @file dm_motor_def.h
 * @brief 达妙电机模块共用数据类型定义。
 *
 * 本文件集中描述电机寄存器参数、实时反馈、控制目标以及单个电机对象的数据布局，
 * 供 dm_motor_drv 与 dm_motor_ctrl 共同使用。这里只保存协议数据和运行状态，不直接
 * 访问 CAN 外设。
 *
 * @note 位置、速度、扭矩等物理量在控制接口中分别按 rad、rad/s、N·m 使用；寄存器
 *       参数的实际量纲和有效范围以对应型号电机固件的寄存器手册为准。
 */
#ifndef __DM_MOTOR_DEF_H__
#define __DM_MOTOR_DEF_H__
#include "main.h"

/**
 * @brief 电机驱动器寄存器参数及参数读写状态。
 *
 * read_flag 用于控制参数轮询状态机；其余成员对应驱动器参数表中的寄存器值。
 * 参数读取完成后，dm_motor_ctrl 会把 CAN 返回值写入对应成员。
 */
typedef struct {
  uint8_t read_flag;  ///< 参数轮询步骤：0 表示停止，非 0 表示当前待读取项
  uint8_t write_flag; ///< 参数写入流程标志，供上层参数管理逻辑使用
  uint8_t save_flag;  ///< 参数保存流程标志，供上层参数管理逻辑使用

  float UV_Value;   ///< 欠压保护阈值
  float KT_Value;   ///< 电机扭矩系数
  float OT_Value;   ///< 过温保护阈值
  float OC_Value;   ///< 过流保护阈值
  float ACC;        ///< 加速过程的加速度参数
  float DEC;        ///< 减速过程的减速度参数
  float MAX_SPD;    ///< 驱动器允许的最大速度
  uint32_t MST_ID;  ///< 电机向主控发送反馈时使用的标准帧 ID
  uint32_t ESC_ID;  ///< 电机接收主控命令时使用的标准帧 ID
  uint32_t TIMEOUT; ///< 通信超时报警时间参数
  uint32_t cmode;   ///< 驱动器当前控制模式编号
  float Damp;       ///< 电机等效粘滞阻尼系数
  float Inertia;    ///< 电机等效转动惯量
  uint32_t hw_ver;  ///< 硬件版本或固件保留字段
  uint32_t sw_ver;  ///< 主软件版本号
  uint32_t SN;      ///< 设备序列号或固件保留字段
  uint32_t NPP;     ///< 电机极对数
  float Rs;         ///< 电机相电阻参数
  float Ls;         ///< 电机相电感参数
  float Flux;       ///< 电机磁链参数
  float Gr;         ///< 齿轮减速比
  float PMAX;       ///< MIT 协议位置量化映射的绝对范围
  float VMAX;       ///< MIT 协议速度量化映射的绝对范围
  float TMAX;       ///< MIT 协议扭矩量化映射的绝对范围
  float I_BW;       ///< 电流环控制带宽
  float KP_ASR;     ///< 速度环比例增益 Kp
  float KI_ASR;     ///< 速度环积分增益 Ki
  float KP_APR;     ///< 位置环比例增益 Kp
  float KI_APR;     ///< 位置环积分增益 Ki
  float OV_Value;   ///< 过压保护阈值
  float GREF;       ///< 齿轮力矩效率系数
  float Deta;       ///< 速度环阻尼系数
  float V_BW;       ///< 速度环滤波带宽
  float IQ_cl;      ///< 电流环增强系数
  float VL_cl;      ///< 速度环增强系数
  uint32_t can_br;  ///< CAN 波特率配置代码
  uint32_t sub_ver; ///< 软件子版本号
  float u_off;      ///< U 相采样偏置
  float v_off;      ///< V 相采样偏置
  float k1;         ///< 驱动器内部补偿因子 1
  float k2;         ///< 驱动器内部补偿因子 2
  float m_off;      ///< 电机机械角度偏移
  float dir;        ///< 电机旋转方向配置
  float p_m;        ///< 电机轴位置参数
  float x_out;      ///< 输出轴位置参数
} esc_inf_t;

/**
 * @brief 电机实时反馈解析结果。
 *
 * p_int、v_int、t_int、kp_int、kd_int 保存协议中的原始整数值，便于调试量化过程；
 * pos、vel、tor、Kp、Kd 是按当前电机映射范围还原后的浮点值。
 */
typedef struct {
  int id;      ///< 反馈帧携带的电机 CAN ID
  int state;   ///< 电机状态码或故障状态
  int p_int;   ///< 协议解包得到的位置原始整数值
  int v_int;   ///< 协议解包得到的速度原始整数值
  int t_int;   ///< 协议解包得到的扭矩原始整数值
  int kp_int;  ///< MIT 控制比例增益的原始整数值
  int kd_int;  ///< MIT 控制微分增益的原始整数值
  float pos;   ///< 解析后的电机位置，单位 rad
  float vel;   ///< 解析后的电机速度，单位 rad/s
  float tor;   ///< 解析后的电机扭矩，单位 N·m
  float Kp;    ///< 解析或记录的 MIT 控制比例增益
  float Kd;    ///< 解析或记录的 MIT 控制微分增益
  float Tmos;  ///< 驱动板 MOS 温度，单位 ℃
  float Tcoil; ///< 电机线圈温度，单位 ℃
} motor_fbpara_t;

/**
 * @brief 上层写入的电机控制目标。
 *
 * 不同控制模式只使用其中一部分字段；发送函数会根据 mode 选择相应协议进行编码。
 */
typedef struct {
  uint8_t mode;  ///< 控制模式，取值见 mode_e
  float pos_set; ///< 目标位置，单位 rad
  float vel_set; ///< 目标速度，单位 rad/s
  float tor_set; ///< 目标扭矩，单位 N·m
  float cur_set; ///< 目标电流，单位 A
  float kp_set;  ///< MIT 控制比例增益 Kp
  float kd_set;  ///< MIT 控制微分增益 Kd
} motor_ctrl_t;

/**
 * @brief 单个达妙电机的软件对象。
 *
 * id 用于主控向电机发送命令，mst_id 用于识别电机返回的反馈帧；para、ctrl 和 tmp
 * 分别保存实时反馈、待发送控制目标和异步读取到的驱动器寄存器参数。
 */
typedef struct {
  uint16_t id;         ///< 电机命令标准帧 ID（主控发送方向）
  uint16_t mst_id;     ///< 电机反馈标准帧 ID（主控接收方向）
  motor_fbpara_t para; ///< 最近一次实时反馈的解析结果
  motor_ctrl_t ctrl;   ///< 当前控制模式及目标值
  esc_inf_t tmp;       ///< 参数读取状态与驱动器寄存器镜像
} motor_t;

#endif
/* __DM_MOTOR_DEF_H__ */
