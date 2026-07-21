/**
 * @file dm_motor_ctrl.c
 * @brief 达妙电机对象初始化、CAN 反馈分发和参数轮询状态机实现。
 *
 * 本文件维护六个全局电机对象，并通过 bsp_can 的订阅式接收接口处理标准数据帧。
 * CAN 回调运行在接收中断上下文：实时反馈直接解析，寄存器反馈只保存为每电机一份
 * 快照；任务上下文随后消费快照并推进参数读取状态机。
 *
 * @note 参数快照是“最新一帧覆盖旧帧”的单槽结构，不是消息队列。若任务消费速度
 *       低于参数反馈到达速度，中间反馈可能被后到帧覆盖。
 */
#include "dm_motor_ctrl.h"

#include "comp_utils.h"
#include "convert.h"
#include "dm_motor_drv.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include <math.h>

/** @brief 六个预定义电机对象，索引由 motor_num 枚举给出。 */
motor_t motor[num];

/**
 * @brief 单个电机的寄存器反馈快照。
 *
 * data 保存完整 8 字节 CAN 数据区；pending 在中断写入新快照后置位，在任务上下文
 * 成功取走快照后清零。
 */
typedef struct {
  uint8_t data[BSP_CAN_DATA_SIZE]; ///< 最近一帧寄存器反馈数据
  volatile bool pending;           ///< true 表示存在尚未消费的新快照
} dm_motor_param_snapshot_t;

/** @brief 与全局 motor 数组一一对应的寄存器反馈快照。 */
static dm_motor_param_snapshot_t dm_motor_param_snapshots[num];

/**
 * @brief 在 CAN 回调中保存指定电机的最新寄存器反馈。
 * @param motor_index 全局 motor 数组中的电机索引。
 * @param rx_data 长度为 BSP_CAN_DATA_SIZE 的接收数据区。
 *
 * 本函数在接收中断上下文执行。先复制完整数据，再置 pending，确保任务侧看到
 * pending=true 时快照内容已经更新。
 */
static void dm_motor_store_param_snapshot(motor_num motor_index,
                                          const uint8_t *rx_data) {
  memcpy(dm_motor_param_snapshots[motor_index].data, rx_data,
         BSP_CAN_DATA_SIZE);
  dm_motor_param_snapshots[motor_index].pending = true;
}

/**
 * @brief 在任务上下文原子地取出一个寄存器反馈快照。
 * @param target 必须指向全局 motor 数组中的某个电机对象。
 * @param rx_data 接收快照的 BSP_CAN_DATA_SIZE 字节缓冲区。
 * @return 有待处理快照时返回 true，否则返回 false。
 *
 * 函数先把对象指针转换为数组索引，再短暂关闭中断，完成“检查 pending、复制数据、
 * 清除 pending”的不可分割操作。恢复时写回进入临界区前的 PRIMASK，因此不会错误
 * 打开调用前已经关闭的中断。
 */
static bool dm_motor_take_param_snapshot(motor_t *target, uint8_t *rx_data) {
  motor_num motor_index;
  for (motor_index = Motor1; motor_index < num;
       motor_index = (motor_num)(motor_index + 1)) {
    if (target == &motor[motor_index]) {
      break;
    }
  }
  if (motor_index == num) {
    return false;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const bool pending = dm_motor_param_snapshots[motor_index].pending;
  if (pending) {
    memcpy(rx_data, dm_motor_param_snapshots[motor_index].data,
           BSP_CAN_DATA_SIZE);
    dm_motor_param_snapshots[motor_index].pending = false;
  }
  __set_PRIMASK(primask);
  return pending;
}

/**
 * @brief 把一帧反馈分发给指定电机对象。
 * @param motor_index 目标电机在全局数组中的索引。
 * @param rx_data 长度为 BSP_CAN_DATA_SIZE 的接收数据区。
 *
 * 所有帧先经过实时反馈解码；当 byte2 为寄存器读取命令字 0x33 时，再保存一份参数
 * 快照，避免在中断中运行较长的参数状态机。
 */
static void dm_motor_dispatch_feedback(motor_num motor_index,
                                       const uint8_t *rx_data) {
  ASSERT(motor_index < num);
  ASSERT(rx_data != NULL);

  if (rx_data == NULL) {
    return;
  }

  dm_motor_fbdata(&motor[motor_index], rx_data);
  if (rx_data[2] == 0x33) {
    dm_motor_store_param_snapshot(motor_index, rx_data);
  }
}

/**
 * @brief 初始化六个电机对象和参数快照。
 *
 * 初始化结果：
 * - 命令 ID 依次为 0x10～0x15，反馈 ID 依次为 0x20～0x25；
 * - read_flag 置 1，使任务侧从寄存器 0 开始轮询；
 * - 默认控制模式为 pos_mode，所有控制目标和增益清零；
 * - PMAX 均为 π rad，VMAX/TMAX 按各关节电机能力分别设置；
 * - 清空全部参数反馈快照，避免复位后消费旧数据。
 */
void dm_motor_init(void) {
  // 先清空六个电机对象，保证未显式赋值的反馈和寄存器字段均从 0 开始。
  memset(&motor[Motor1], 0, sizeof(motor[Motor1]));
  memset(&motor[Motor2], 0, sizeof(motor[Motor2]));
  memset(&motor[Motor3], 0, sizeof(motor[Motor3]));
  memset(&motor[Motor4], 0, sizeof(motor[Motor4]));
  memset(&motor[Motor5], 0, sizeof(motor[Motor5]));
  memset(&motor[Motor6], 0, sizeof(motor[Motor6]));
  memset(dm_motor_param_snapshots, 0, sizeof(dm_motor_param_snapshots));

  /****************** 电机 1：命令 0x10，反馈 0x20，TMAX 20 N·m ******************/
  // 设置 Motor1 的通信标识、默认控制目标和 MIT 量化范围。
  motor[Motor1].id = 0x10;
  motor[Motor1].mst_id = 0x20;
  motor[Motor1].tmp.read_flag = 1;
  motor[Motor1].ctrl.mode = pos_mode;
  motor[Motor1].ctrl.vel_set = 0.0f;
  motor[Motor1].ctrl.pos_set = 0.0f;
  motor[Motor1].ctrl.cur_set = 0.0f;
  motor[Motor1].ctrl.kp_set = 0.0f;
  motor[Motor1].ctrl.kd_set = 0.0f;
  motor[Motor1].tmp.PMAX = 3.141592f;
  motor[Motor1].tmp.VMAX = 45.0f;
  motor[Motor1].tmp.TMAX = 20.0f;
  // 电机 2：命令 0x11，反馈 0x21，TMAX 40 N·m。
  motor[Motor2].id = 0x11;
  motor[Motor2].mst_id = 0x21;
  motor[Motor2].tmp.read_flag = 1;
  motor[Motor2].ctrl.mode = pos_mode;
  motor[Motor2].ctrl.vel_set = 0.0f;
  motor[Motor2].ctrl.pos_set = 0.0f;
  motor[Motor2].ctrl.cur_set = 0.0f;
  motor[Motor2].ctrl.kp_set = 0.0f;
  motor[Motor2].ctrl.kd_set = 0.0f;
  motor[Motor2].tmp.PMAX = 3.141592f;
  motor[Motor2].tmp.VMAX = 45.0f;
  motor[Motor2].tmp.TMAX = 40.0f;
  // 电机 3：命令 0x12，反馈 0x22，TMAX 40 N·m。
  motor[Motor3].id = 0x12;
  motor[Motor3].mst_id = 0x22;
  motor[Motor3].tmp.read_flag = 1;
  motor[Motor3].ctrl.mode = pos_mode;
  motor[Motor3].ctrl.vel_set = 0.0f;
  motor[Motor3].ctrl.pos_set = 0.0f;
  motor[Motor3].ctrl.cur_set = 0.0f;
  motor[Motor3].ctrl.kp_set = 0.0f;
  motor[Motor3].ctrl.kd_set = 0.0f;
  motor[Motor3].tmp.PMAX = 3.141592f;
  motor[Motor3].tmp.VMAX = 45.0f;
  motor[Motor3].tmp.TMAX = 40.0f;
  // 电机 4：命令 0x13，反馈 0x23，VMAX 30 rad/s，TMAX 10 N·m。
  motor[Motor4].id = 0x13;
  motor[Motor4].mst_id = 0x23;
  motor[Motor4].tmp.read_flag = 1;
  motor[Motor4].ctrl.mode = pos_mode;
  motor[Motor4].ctrl.vel_set = 0.0f;
  motor[Motor4].ctrl.pos_set = 0.0f;
  motor[Motor4].ctrl.cur_set = 0.0f;
  motor[Motor4].ctrl.kp_set = 0.0f;
  motor[Motor4].ctrl.kd_set = 0.0f;
  motor[Motor4].tmp.PMAX = 3.141592f;
  motor[Motor4].tmp.VMAX = 30.0f;
  motor[Motor4].tmp.TMAX = 10.0f;
  // 电机 5：命令 0x14，反馈 0x24，VMAX 30 rad/s，TMAX 10 N·m。
  motor[Motor5].id = 0x14;
  motor[Motor5].mst_id = 0x24;
  motor[Motor5].tmp.read_flag = 1;
  motor[Motor5].ctrl.mode = pos_mode;
  motor[Motor5].ctrl.vel_set = 0.0f;
  motor[Motor5].ctrl.pos_set = 0.0f;
  motor[Motor5].ctrl.cur_set = 0.0f;
  motor[Motor5].ctrl.kp_set = 0.0f;
  motor[Motor5].ctrl.kd_set = 0.0f;
  motor[Motor5].tmp.PMAX = 3.141592f;
  motor[Motor5].tmp.VMAX = 30.0f;
  motor[Motor5].tmp.TMAX = 10.0f;
  // 电机 6：命令 0x15，反馈 0x25，VMAX 30 rad/s，TMAX 10 N·m。
  motor[Motor6].id = 0x15;
  motor[Motor6].mst_id = 0x25;
  motor[Motor6].tmp.read_flag = 1;
  motor[Motor6].ctrl.mode = pos_mode;
  motor[Motor6].ctrl.vel_set = 0.0f;
  motor[Motor6].ctrl.pos_set = 0.0f;
  motor[Motor6].ctrl.cur_set = 0.0f;
  motor[Motor6].ctrl.kp_set = 0.0f;
  motor[Motor6].ctrl.kd_set = 0.0f;
  motor[Motor6].tmp.PMAX = 3.141592f;
  motor[Motor6].tmp.VMAX = 30.0f;
  motor[Motor6].tmp.TMAX = 10.0f;

}

/**
 * @brief 消费参数反馈并推进完整寄存器轮询。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor 待读取参数的全局电机对象。
 *
 * 每次调用先尝试原子取出该电机的最新参数快照；若存在，则由 receive_motor_data()
 * 写入寄存器镜像并推进 read_flag。随后根据新的 read_flag 发送下一项读取命令：
 * - 状态 1～37 对应寄存器 0～36；
 * - 状态 38～43 对应寄存器 50～55；
 * - 状态 44～45 对应寄存器 80～81；
 * - 状态 0 表示轮询完成，不再发送。
 *
 * 未收到反馈时 read_flag 保持不变，因此周期调用会重发当前寄存器请求。调用周期即
 * 重试间隔，应由任务调度控制，避免在高速循环中持续占用 CAN 总线。
 */
void read_all_motor_data(STM32CAN_t *hcan, motor_t *motor) {
  uint8_t rx_data[BSP_CAN_DATA_SIZE];
  if (dm_motor_take_param_snapshot(motor, rx_data)) {
    receive_motor_data(motor, rx_data);
  }

  switch (motor->tmp.read_flag) {
  case 1:
    read_motor_data(hcan, motor->id, 0);
    break; // UV_Value
  case 2:
    read_motor_data(hcan, motor->id, 1);
    break; // KT_Value
  case 3:
    read_motor_data(hcan, motor->id, 2);
    break; // OT_Value
  case 4:
    read_motor_data(hcan, motor->id, 3);
    break; // OC_Value
  case 5:
    read_motor_data(hcan, motor->id, 4);
    break; // ACC
  case 6:
    read_motor_data(hcan, motor->id, 5);
    break; // DEC
  case 7:
    read_motor_data(hcan, motor->id, 6);
    break; // MAX_SPD
  case 8:
    read_motor_data(hcan, motor->id, 7);
    break; // MST_ID
  case 9:
    read_motor_data(hcan, motor->id, 8);
    break; // ESC_ID
  case 10:
    read_motor_data(hcan, motor->id, 9);
    break; // TIMEOUT
  case 11:
    read_motor_data(hcan, motor->id, 10);
    break; // CTRL_MODE
  case 12:
    read_motor_data(hcan, motor->id, 11);
    break; // Damp
  case 13:
    read_motor_data(hcan, motor->id, 12);
    break; // Inertia
  case 14:
    read_motor_data(hcan, motor->id, 13);
    break; // Rsv1
  case 15:
    read_motor_data(hcan, motor->id, 14);
    break; // sw_ver
  case 16:
    read_motor_data(hcan, motor->id, 15);
    break; // Rsv2
  case 17:
    read_motor_data(hcan, motor->id, 16);
    break; // NPP
  case 18:
    read_motor_data(hcan, motor->id, 17);
    break; // Rs
  case 19:
    read_motor_data(hcan, motor->id, 18);
    break; // Ls
  case 20:
    read_motor_data(hcan, motor->id, 19);
    break; // Flux
  case 21:
    read_motor_data(hcan, motor->id, 20);
    break; // Gr
  case 22:
    read_motor_data(hcan, motor->id, 21);
    break; // PMAX
  case 23:
    read_motor_data(hcan, motor->id, 22);
    break; // VMAX
  case 24:
    read_motor_data(hcan, motor->id, 23);
    break; // TMAX
  case 25:
    read_motor_data(hcan, motor->id, 24);
    break; // I_BW
  case 26:
    read_motor_data(hcan, motor->id, 25);
    break; // KP_ASR
  case 27:
    read_motor_data(hcan, motor->id, 26);
    break; // KI_ASR
  case 28:
    read_motor_data(hcan, motor->id, 27);
    break; // KP_APR
  case 29:
    read_motor_data(hcan, motor->id, 28);
    break; // KI_APR
  case 30:
    read_motor_data(hcan, motor->id, 29);
    break; // OV_Value
  case 31:
    read_motor_data(hcan, motor->id, 30);
    break; // GREF
  case 32:
    read_motor_data(hcan, motor->id, 31);
    break; // Deta
  case 33:
    read_motor_data(hcan, motor->id, 32);
    break; // V_BW
  case 34:
    read_motor_data(hcan, motor->id, 33);
    break; // IQ_cl
  case 35:
    read_motor_data(hcan, motor->id, 34);
    break; // VL_cl
  case 36:
    read_motor_data(hcan, motor->id, 35);
    break; // can_br
  case 37:
    read_motor_data(hcan, motor->id, 36);
    break; // sub_ver
  case 38:
    read_motor_data(hcan, motor->id, 50);
    break; // u_off
  case 39:
    read_motor_data(hcan, motor->id, 51);
    break; // v_off
  case 40:
    read_motor_data(hcan, motor->id, 52);
    break; // k1
  case 41:
    read_motor_data(hcan, motor->id, 53);
    break; // k2
  case 42:
    read_motor_data(hcan, motor->id, 54);
    break; // m_off
  case 43:
    read_motor_data(hcan, motor->id, 55);
    break; // dir
  case 44:
    read_motor_data(hcan, motor->id, 80);
    break; // p_m
  case 45:
    read_motor_data(hcan, motor->id, 81);
    break; // x_out
  }
}

/**
 * @brief 解析寄存器反馈，更新参数镜像并推进读取状态。
 * @param motor 接收参数结果的电机对象。
 * @param data 长度为 BSP_CAN_DATA_SIZE 的寄存器反馈数据区。
 *
 * 仅在 read_flag 非 0 且 byte2 为读取命令字 0x33 时处理数据。byte3 是寄存器编号，
 * byte4～7 按小端顺序重解释为 float 或 uint32_t：
 * - ID、超时、模式、版本、极对数和波特率等离散字段使用 uint32_t；
 * - 阈值、环路参数、映射范围和校准量等连续字段使用 float。
 *
 * 每个已知寄存器处理完成后，把 read_flag 设置为下一轮询状态；寄存器 81 完成后
 * 清零 read_flag。未知寄存器编号不会修改参数，也不会推进状态。
 */
void receive_motor_data(motor_t *motor, const uint8_t *data) {
  if (motor->tmp.read_flag == 0)
    return;

  float_type_u y;

  if (data[2] == 0x33) {
    y.b_val[0] = data[4];
    y.b_val[1] = data[5];
    y.b_val[2] = data[6];
    y.b_val[3] = data[7];

    switch (data[3]) {
    case 0:
      motor->tmp.UV_Value = y.f_val;
      motor->tmp.read_flag = 2;
      break;
    case 1:
      motor->tmp.KT_Value = y.f_val;
      motor->tmp.read_flag = 3;
      break;
    case 2:
      motor->tmp.OT_Value = y.f_val;
      motor->tmp.read_flag = 4;
      break;
    case 3:
      motor->tmp.OC_Value = y.f_val;
      motor->tmp.read_flag = 5;
      break;
    case 4:
      motor->tmp.ACC = y.f_val;
      motor->tmp.read_flag = 6;
      break;
    case 5:
      motor->tmp.DEC = y.f_val;
      motor->tmp.read_flag = 7;
      break;
    case 6:
      motor->tmp.MAX_SPD = y.f_val;
      motor->tmp.read_flag = 8;
      break;
    case 7:
      motor->tmp.MST_ID = y.u_val;
      motor->tmp.read_flag = 9;
      break;
    case 8:
      motor->tmp.ESC_ID = y.u_val;
      motor->tmp.read_flag = 10;
      break;
    case 9:
      motor->tmp.TIMEOUT = y.u_val;
      motor->tmp.read_flag = 11;
      break;
    case 10:
      motor->tmp.cmode = y.u_val;
      motor->tmp.read_flag = 12;
      break;
    case 11:
      motor->tmp.Damp = y.f_val;
      motor->tmp.read_flag = 13;
      break;
    case 12:
      motor->tmp.Inertia = y.f_val;
      motor->tmp.read_flag = 14;
      break;
    case 13:
      motor->tmp.hw_ver = y.u_val;
      motor->tmp.read_flag = 15;
      break;
    case 14:
      motor->tmp.sw_ver = y.u_val;
      motor->tmp.read_flag = 16;
      break;
    case 15:
      motor->tmp.SN = y.u_val;
      motor->tmp.read_flag = 17;
      break;
    case 16:
      motor->tmp.NPP = y.u_val;
      motor->tmp.read_flag = 18;
      break;
    case 17:
      motor->tmp.Rs = y.f_val;
      motor->tmp.read_flag = 19;
      break;
    case 18:
      motor->tmp.Ls = y.f_val;
      motor->tmp.read_flag = 20;
      break;
    case 19:
      motor->tmp.Flux = y.f_val;
      motor->tmp.read_flag = 21;
      break;
    case 20:
      motor->tmp.Gr = y.f_val;
      motor->tmp.read_flag = 22;
      break;
    case 21:
      motor->tmp.PMAX = y.f_val;
      motor->tmp.read_flag = 23;
      break;
    case 22:
      motor->tmp.VMAX = y.f_val;
      motor->tmp.read_flag = 24;
      break;
    case 23:
      motor->tmp.TMAX = y.f_val;
      motor->tmp.read_flag = 25;
      break;
    case 24:
      motor->tmp.I_BW = y.f_val;
      motor->tmp.read_flag = 26;
      break;
    case 25:
      motor->tmp.KP_ASR = y.f_val;
      motor->tmp.read_flag = 27;
      break;
    case 26:
      motor->tmp.KI_ASR = y.f_val;
      motor->tmp.read_flag = 28;
      break;
    case 27:
      motor->tmp.KP_APR = y.f_val;
      motor->tmp.read_flag = 29;
      break;
    case 28:
      motor->tmp.KI_APR = y.f_val;
      motor->tmp.read_flag = 30;
      break;
    case 29:
      motor->tmp.OV_Value = y.f_val;
      motor->tmp.read_flag = 31;
      break;
    case 30:
      motor->tmp.GREF = y.f_val;
      motor->tmp.read_flag = 32;
      break;
    case 31:
      motor->tmp.Deta = y.f_val;
      motor->tmp.read_flag = 33;
      break;
    case 32:
      motor->tmp.V_BW = y.f_val;
      motor->tmp.read_flag = 34;
      break;
    case 33:
      motor->tmp.IQ_cl = y.f_val;
      motor->tmp.read_flag = 35;
      break;
    case 34:
      motor->tmp.VL_cl = y.f_val;
      motor->tmp.read_flag = 36;
      break;
    case 35:
      motor->tmp.can_br = y.u_val;
      motor->tmp.read_flag = 37;
      break;
    case 36:
      motor->tmp.sub_ver = y.u_val;
      motor->tmp.read_flag = 38;
      break;
    case 50:
      motor->tmp.u_off = y.f_val;
      motor->tmp.read_flag = 39;
      break;
    case 51:
      motor->tmp.v_off = y.f_val;
      motor->tmp.read_flag = 40;
      break;
    case 52:
      motor->tmp.k1 = y.f_val;
      motor->tmp.read_flag = 41;
      break;
    case 53:
      motor->tmp.k2 = y.f_val;
      motor->tmp.read_flag = 42;
      break;
    case 54:
      motor->tmp.m_off = y.f_val;
      motor->tmp.read_flag = 43;
      break;
    case 55:
      motor->tmp.dir = y.f_val;
      motor->tmp.read_flag = 44;
      break;
    case 80:
      motor->tmp.p_m = y.f_val;
      motor->tmp.read_flag = 45;
      break;
    case 81:
      motor->tmp.x_out = y.f_val;
      motor->tmp.read_flag = 0;
      break;
    }
  }
}

/**
************************************************************************
* @brief:      	read_dm_motor_pos:
*将电机位置浮点数转换为无符号整数（编码器格式）
* @param:      	pos_float: 位置浮点数值（单位：rad）
* @param:      	p_max: 位置最大值（单位：rad）
* @param:      	p_min: 位置最小值（单位：rad）
* @retval:     	16位无符号整数编码的位置值
* @details:    	用于将控制命令中的位置设定值转换为CAN通信所需的整数格式。
*               采用16位编码，可表示65536个位置离散值。
************************************************************************
**/
int read_dm_motor_pos(float pos_float, float p_max, float p_min) {
  return float_to_uint(pos_float, p_min, p_max, 16);
}

/**
************************************************************************
* @brief:      	read_dm_motor_vel: 将电机速度浮点数转换为无符号整数
* @param:      	vel_float: 速度浮点数值（单位：rad/s）
* @param:      	v_max: 速度最大值（单位：rad/s）
* @param:      	v_min: 速度最小值（单位：rad/s）
* @retval:     	12位无符号整数编码的速度值
* @details:    	用于将控制命令中的速度设定值转换为CAN通信所需的整数格式。
*               采用12位编码，可表示4096个速度离散值。
************************************************************************
**/
int read_dm_motor_vel(float vel_float, float v_max, float v_min) {
  return float_to_uint(vel_float, v_min, v_max, 12);
}

/**
************************************************************************
* @brief:      	read_dm_motor_tor: 将电机扭矩浮点数转换为无符号整数
* @param:      	tor_float: 扭矩浮点数值（单位：N·m）
* @param:      	t_max: 扭矩最大值（单位：N·m）
* @param:      	t_min: 扭矩最小值（单位：N·m）
* @retval:     	12位无符号整数编码的扭矩值
* @details:    	用于将控制命令中的扭矩设定值转换为CAN通信所需的整数格式。
*               采用12位编码，可表示4096个扭矩离散值。
************************************************************************
**/
int read_dm_motor_tor(float tor_float, float t_max, float t_min) {
  return float_to_uint(tor_float, t_min, t_max, 12);
}

/**
************************************************************************
* @brief:      	set_dm_motor_pos: 将无符号整数位置值转换为浮点数
* @param:      	pos_int: 16位无符号整数编码的位置值
* @param:      	p_max: 位置最大值（单位：rad）
* @param:      	p_min: 位置最小值（单位：rad）
* @retval:     	位置浮点数值（单位：rad）
* @details:    	用于解析电机反馈的位置数据，将CAN通信接收的整数格式
*               转换为实际物理量。采用16位解码。
************************************************************************
**/
float set_dm_motor_pos(int pos_int, float p_max, float p_min) {
  return uint_to_float(pos_int, p_min, p_max, 16);
}

/**
************************************************************************
* @brief:      	set_dm_motor_vel: 将无符号整数速度值转换为浮点数
* @param:      	vel_int: 12位无符号整数编码的速度值
* @param:      	v_max: 速度最大值（单位：rad/s）
* @param:      	v_min: 速度最小值（单位：rad/s）
* @retval:     	速度浮点数值（单位：rad/s）
* @details:    	用于解析电机反馈的速度数据，将CAN通信接收的整数格式
*               转换为实际物理量。采用12位解码。
************************************************************************
**/
float set_dm_motor_vel(int vel_int, float v_max, float v_min) {
  return uint_to_float(vel_int, v_min, v_max, 12);
}

/**
************************************************************************
* @brief:      	set_dm_motor_tor: 将无符号整数扭矩值转换为浮点数
* @param:      	tor_int: 12位无符号整数编码的扭矩值
* @param:      	t_max: 扭矩最大值（单位：N·m）
* @param:      	t_min: 扭矩最小值（单位：N·m）
* @retval:     	扭矩浮点数值（单位：N·m）
* @details:    	用于解析电机反馈的扭矩数据，将CAN通信接收的整数格式
*               转换为实际物理量。采用12位解码。
************************************************************************
**/
float set_dm_motor_tor(int tor_int, float t_max, float t_min) {
  return uint_to_float(tor_int, t_min, t_max, 12);
}

/**
 * @brief BSP CAN 接收订阅回调，校验并路由六个电机反馈 ID。
 * @param hcan 触发本次回调的 BSP CAN 设备。
 * @param frame BSP 层已经复制完成的接收帧。
 * @param context 订阅者上下文；本模块订阅时传入 NULL，因此此处不使用。
 *
 * 只接受 11 bit 标准数据帧且 DLC 必须为 8。反馈 ID 0x20～0x25 分别路由到
 * Motor1～Motor6；其他 ID 由本模块忽略，以便同一 CAN 总线上的其他订阅者处理。
 *
 * @note 此函数运行在 BSP CAN 接收回调上下文中，应保持短小且不得执行阻塞操作。
 */
void dm_can1_rx_callback(STM32CAN_t *hcan,
                         const BSP_CAN_Frame_t *frame,
                         void *context) {
  (void)context;

  ASSERT(hcan != NULL);
  ASSERT(frame != NULL);
  if ((hcan == NULL) || (frame == NULL) ||
      (frame->ide_ != CAN_ID_STD) ||
      (frame->rtr_ != CAN_RTR_DATA) ||
      (frame->size_ != BSP_CAN_DATA_SIZE)) {
    return;
  }

  switch (frame->id_) {
  case 0x20:
    dm_motor_dispatch_feedback(Motor1, frame->data_);
    break;
  case 0x21:
    dm_motor_dispatch_feedback(Motor2, frame->data_);
    break;
  case 0x22:
    dm_motor_dispatch_feedback(Motor3, frame->data_);
    break;
  case 0x23:
    dm_motor_dispatch_feedback(Motor4, frame->data_);
    break;
  case 0x24:
    dm_motor_dispatch_feedback(Motor5, frame->data_);
    break;
  case 0x25:
    dm_motor_dispatch_feedback(Motor6, frame->data_);
    break;
  default:
    break;
  }
}

/**
 * @brief 预留的第二路 CAN 接收回调。
 * @param hcan 触发回调的 BSP CAN 设备。
 * @param frame 接收到的 CAN 帧。
 * @param context 订阅者上下文。
 *
 * 当前六个电机全部由 dm_can1_rx_callback() 处理，本占位函数不解析任何帧，便于后续
 * 扩展到第二路 CAN 时保持接口形式一致。
 */
void dm_can2_rx_callback(STM32CAN_t *hcan,
                         const BSP_CAN_Frame_t *frame,
                         void *context) {
  (void)hcan;
  (void)frame;
  (void)context;
}

/**
 * @brief 把达妙电机反馈回调订阅到一个 BSP CAN 设备。
 * @param can 已完成 STM32CAN_Init() 的 CAN 设备对象。
 * @return can 为 NULL 时返回 PTR_NULL；否则返回 STM32CAN_SubscribeRx() 的结果。
 *
 * 本函数只注册订阅关系，不负责初始化或启动 CAN 外设。应用层应在 CAN 设备初始化后、
 * 接收中断开始工作前调用一次，避免重复占用订阅槽位。
 */
err_t dm_motor_attach_can(STM32CAN_t *can) {
  if (can == NULL) {
    return PTR_NULL;
  }
  return STM32CAN_SubscribeRx(can, dm_can1_rx_callback, NULL);
}
