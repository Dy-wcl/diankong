/**
 * @file dm_motor_drv.c
 * @brief 达妙电机 CAN 协议的帧编码、发送与实时反馈解码实现。
 *
 * 所有发送均通过 bsp_can 的 STM32CAN_Send() 完成，使用 11 bit 标准数据帧。
 * 控制帧 ID 为电机基础 ID 与控制模式偏移量之和；寄存器命令统一发送到 0x7FF。
 *
 * @note 本文件公开接口保持 void 形式，因此调用 STM32CAN_Send() 后不向上传递返回值。
 *       如需分析发送失败，应由应用层查询 BSP CAN 错误状态或增加独立诊断。
 */
#include "dm_motor_drv.h"
#include "can.h"
#include <math.h>

/**
 * @brief 按电机对象当前选择的模式发送使能命令。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor 待使能的电机对象。
 *
 * 函数把 mit_mode、pos_mode、spd_mode、psi_mode 分别映射到对应的 CAN ID
 * 偏移量，再发送末字节为 0xFC 的八字节特殊帧。未知模式不会发送帧。
 */
void dm_motor_enable(STM32CAN_t *hcan, motor_t *motor) {
  switch (motor->ctrl.mode) {
  case mit_mode:
    enable_motor_mode(hcan, motor->id, MIT_MODE);
    break;
  case pos_mode:
    enable_motor_mode(hcan, motor->id, POS_MODE);
    break;
  case spd_mode:
    enable_motor_mode(hcan, motor->id, SPD_MODE);
    break;
  case psi_mode:
    enable_motor_mode(hcan, motor->id, PSI_MODE);
    break;
  default:
    break;
  }
}

/**
 * @brief 按电机对象当前选择的模式发送失能命令。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor 待失能的电机对象。
 *
 * 发送末字节为 0xFD 的八字节特殊帧后，清零 RAM 中的位置、速度、扭矩、电流以及
 * MIT 增益目标，避免再次使能时沿用旧命令。
 */
void dm_motor_disable(STM32CAN_t *hcan, motor_t *motor) {
  switch (motor->ctrl.mode) {
  case mit_mode:
    disable_motor_mode(hcan, motor->id, MIT_MODE);
    break;
  case pos_mode:
    disable_motor_mode(hcan, motor->id, POS_MODE);
    break;
  case spd_mode:
    disable_motor_mode(hcan, motor->id, SPD_MODE);
    break;
  case psi_mode:
    disable_motor_mode(hcan, motor->id, PSI_MODE);
    break;
  default:
    break;
  }
  dm_motor_clear_para(motor);
}

/**
 * @brief 根据控制模式编码并发送当前控制目标。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor 提供命令 ID、控制模式、目标值和 MIT 映射范围的电机对象。
 *
 * 四种模式分别调用 mit_ctrl()、pos_ctrl()、spd_ctrl() 和 psi_ctrl()。调用方应在
 * 进入本函数前完成目标限幅；本函数只按协议量化或重排字节。
 */
void dm_motor_ctrl_send(STM32CAN_t *hcan, motor_t *motor) {
  switch (motor->ctrl.mode) {
  case mit_mode:

    mit_ctrl(hcan, motor, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set,
             motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
    break;
  case pos_mode:
    pos_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
    break;
  case spd_mode:
    spd_ctrl(hcan, motor->id, motor->ctrl.vel_set);
    break;
  case psi_mode:
    psi_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set,
             motor->ctrl.cur_set);
    break;
  }
}

/**
 * @brief 清零电机对象中的全部控制目标。
 * @param motor 待清理的电机对象。
 *
 * 仅修改本地 motor->ctrl，不改变控制模式，也不会自动发送清零后的控制帧。
 */
void dm_motor_clear_para(motor_t *motor) {
  motor->ctrl.kd_set = 0;
  motor->ctrl.kp_set = 0;
  motor->ctrl.pos_set = 0;
  motor->ctrl.vel_set = 0;
  motor->ctrl.tor_set = 0;
  motor->ctrl.cur_set = 0;
}

/**
 * @brief 按电机对象当前选择的模式发送清除错误命令。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor 待清除错误的电机对象。
 *
 * 特殊帧末字节为 0xFB；未知控制模式不会发送帧。
 */
void dm_motor_clear_err(STM32CAN_t *hcan, motor_t *motor) {
  switch (motor->ctrl.mode) {
  case mit_mode:
    clear_err(hcan, motor->id, MIT_MODE);
    break;
  case pos_mode:
    clear_err(hcan, motor->id, POS_MODE);
    break;
  case spd_mode:
    clear_err(hcan, motor->id, SPD_MODE);
    break;
  case psi_mode:
    clear_err(hcan, motor->id, PSI_MODE);
    break;
  default:
    break;
  }
}

/**
 * @brief 解码八字节实时反馈并更新电机反馈结构。
 * @param motor 接收解析结果且提供 PMAX、VMAX、TMAX 映射范围的电机对象。
 * @param rx_data 长度至少为 8 字节的反馈数据区。
 *
 * 数据布局：
 * - byte0：高 4 bit 为状态，低 4 bit 为电机 ID；
 * - byte1～2：16 bit 位置；
 * - byte3～5：两个交错排列的 12 bit 速度和扭矩；
 * - byte6、byte7：MOS 温度与线圈温度。
 *
 * 三个量化值按 motor->tmp 中配置的对称范围还原为浮点物理量。
 */
void dm_motor_fbdata(motor_t *motor, const uint8_t *rx_data) {
  motor->para.id = (rx_data[0]) & 0x0F;
  motor->para.state = (rx_data[0]) >> 4;
  motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
  motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
  motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
  motor->para.pos = uint_to_float(motor->para.p_int, -motor->tmp.PMAX,
                                  motor->tmp.PMAX, 16); // 16 bit 位置映射
  motor->para.vel = uint_to_float(motor->para.v_int, -motor->tmp.VMAX,
                                  motor->tmp.VMAX, 12); // 12 bit 速度映射
  motor->para.tor = uint_to_float(motor->para.t_int, -motor->tmp.TMAX,
                                  motor->tmp.TMAX, 12); // 12 bit 扭矩映射
  motor->para.Tmos = (float)(rx_data[6]);
  motor->para.Tcoil = (float)(rx_data[7]);
}

/**
 * @brief 向指定模式的 CAN ID 发送使能特殊帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor_id 电机基础标准帧 ID。
 * @param mode_id 控制模式 ID 偏移量。
 *
 * 发送 ID 为 motor_id + mode_id；数据固定为 FF FF FF FF FF FF FF FC。
 */
void enable_motor_mode(STM32CAN_t *hcan, uint16_t motor_id, uint16_t mode_id) {
  uint8_t data[8];
  uint16_t id = motor_id + mode_id;

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFC;

  (void)STM32CAN_Send(hcan, id, data, 8U);
}

/**
 * @brief 向指定模式的 CAN ID 发送失能特殊帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor_id 电机基础标准帧 ID。
 * @param mode_id 控制模式 ID 偏移量。
 *
 * 数据固定为 FF FF FF FF FF FF FF FD。
 */
void disable_motor_mode(STM32CAN_t *hcan, uint16_t motor_id,
                        uint16_t mode_id) {
  uint8_t data[8];
  uint16_t id = motor_id + mode_id;

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFD;

  (void)STM32CAN_Send(hcan, id, data, 8U);
}

/**
 * @brief 把当前位置保存为指定模式的机械零点。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor_id 电机基础标准帧 ID。
 * @param mode_id 控制模式 ID 偏移量。
 *
 * 数据固定为 FF FF FF FF FF FF FF FE。执行后零点会发生持久化变化，调用方应确保
 * 电机已处于期望的机械位置。
 */
void save_pos_zero(STM32CAN_t *hcan, uint16_t motor_id, uint16_t mode_id) {
  uint8_t data[8];
  uint16_t id = motor_id + mode_id;

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFE;

  (void)STM32CAN_Send(hcan, id, data, 8U);
}

/**
 * @brief 向指定模式的 CAN ID 发送清除错误特殊帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor_id 电机基础标准帧 ID。
 * @param mode_id 控制模式 ID 偏移量。
 *
 * 数据固定为 FF FF FF FF FF FF FF FB。本命令只请求驱动器清除可恢复错误，不负责
 * 判断故障原因或确认错误是否已消失。
 */
void clear_err(STM32CAN_t *hcan, uint16_t motor_id, uint16_t mode_id) {
  uint8_t data[8];
  uint16_t id = motor_id + mode_id;

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFB;

  (void)STM32CAN_Send(hcan, id, data, 8U);
}

/**
 * @brief 量化并发送 MIT 力矩/阻抗控制帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor 提供 PMAX、VMAX、TMAX 映射范围的电机对象。
 * @param motor_id 电机基础标准帧 ID。
 * @param pos 目标位置，单位 rad，量化为 16 bit。
 * @param vel 目标速度，单位 rad/s，量化为 12 bit。
 * @param kp 位置比例增益，按 KP_MIN～KP_MAX 量化为 12 bit。
 * @param kd 速度微分增益，按 KD_MIN～KD_MAX 量化为 12 bit。
 * @param tor 前馈扭矩，单位 N·m，量化为 12 bit。
 *
 * 八字节布局依次为 P[15:8]、P[7:0]、V[11:4]、V[3:0]|Kp[11:8]、
 * Kp[7:0]、Kd[11:4]、Kd[3:0]|T[11:8]、T[7:0]。
 */
void mit_ctrl(STM32CAN_t *hcan, motor_t *motor, uint16_t motor_id, float pos,
              float vel, float kp, float kd, float tor) {
  uint8_t data[8];
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  pos_tmp = float_to_uint(pos, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
  vel_tmp = float_to_uint(vel, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
  tor_tmp = float_to_uint(tor, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
  kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;

  (void)STM32CAN_Send(hcan, id, data, 8U);
}

/**
 * @brief 发送位置-速度控制帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor_id 电机基础标准帧 ID。
 * @param pos 目标位置，单位 rad。
 * @param vel 目标速度，单位 rad/s。
 *
 * pos 和 vel 均按 STM32 小端内存中的 32 bit IEEE 754 原始字节发送：byte0～3 为
 * 位置，byte4～7 为速度。该模式不使用 MIT 定点量化。
 */
void pos_ctrl(STM32CAN_t *hcan, uint16_t motor_id, float pos, float vel) {
  uint16_t id;
  uint8_t *pbuf, *vbuf;
  uint8_t data[8];

  id = motor_id + POS_MODE;
  pbuf = (uint8_t *)&pos;
  vbuf = (uint8_t *)&vel;

  data[0] = *pbuf;
  data[1] = *(pbuf + 1);
  data[2] = *(pbuf + 2);
  data[3] = *(pbuf + 3);

  data[4] = *vbuf;
  data[5] = *(vbuf + 1);
  data[6] = *(vbuf + 2);
  data[7] = *(vbuf + 3);

  (void)STM32CAN_Send(hcan, id, data, 8U);
}

/**
 * @brief 发送纯速度控制帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor_id 电机基础标准帧 ID。
 * @param vel 目标速度，单位 rad/s。
 *
 * 四字节数据区直接承载 STM32 小端内存中的 32 bit IEEE 754 速度值。
 */
void spd_ctrl(STM32CAN_t *hcan, uint16_t motor_id, float vel) {
  uint16_t id;
  uint8_t *vbuf;
  uint8_t data[4];

  id = motor_id + SPD_MODE;
  vbuf = (uint8_t *)&vel;

  data[0] = *vbuf;
  data[1] = *(vbuf + 1);
  data[2] = *(vbuf + 2);
  data[3] = *(vbuf + 3);

  (void)STM32CAN_Send(hcan, id, data, 4U);
}

/**
 * @brief 发送位置-速度-电流混合控制帧。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param motor_id 电机基础标准帧 ID。
 * @param pos 目标位置，单位 rad，使用 32 bit IEEE 754 原始字节。
 * @param vel 目标速度，单位 rad/s，乘以 100 后转换为 uint16_t。
 * @param cur 目标电流，单位 A，乘以 10000 后转换为 uint16_t。
 *
 * 数据布局为：byte0～3 位置浮点原始字节，byte4～5 速度定点值，byte6～7 电流
 * 定点值；多字节字段均按 STM32 小端顺序排列。
 *
 * @note 当前转换直接使用 uint16_t，不在本函数内做负值编码或越界保护。调用方应按
 *       电机协议允许范围提供 vel 与 cur。
 */

void psi_ctrl(STM32CAN_t *hcan, uint16_t motor_id, float pos, float vel,
              float cur) {
  uint16_t id;
  uint8_t *pbuf, *vbuf, *ibuf;
  uint8_t data[8];

  uint16_t u16_vel = vel * 100;
  uint16_t u16_cur = cur * 10000;

  id = motor_id + PSI_MODE;
  pbuf = (uint8_t *)&pos;
  vbuf = (uint8_t *)&u16_vel;
  ibuf = (uint8_t *)&u16_cur;

  data[0] = *pbuf;
  data[1] = *(pbuf + 1);
  data[2] = *(pbuf + 2);
  data[3] = *(pbuf + 3);

  data[4] = *vbuf;
  data[5] = *(vbuf + 1);

  data[6] = *ibuf;
  data[7] = *(ibuf + 1);

  (void)STM32CAN_Send(hcan, id, data, 8U);
}

/**
 * @brief 发送单个寄存器读取命令。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param id 目标电机的 11 bit 标准帧 ID。
 * @param rid 待读取的寄存器编号。
 *
 * 命令统一发送到标准帧 ID 0x7FF，四字节数据依次为电机 ID 低 8 bit、电机 ID
 * 高 3 bit、读取命令字 0x33 和寄存器编号。寄存器结果由控制层的参数快照流程接收。
 */
void read_motor_data(STM32CAN_t *hcan, uint16_t id, uint8_t rid) {
  uint8_t can_id_l = id & 0xFF;        // 标准帧 ID 的低 8 bit
  uint8_t can_id_h = (id >> 8) & 0x07; // 标准帧 ID 的高 3 bit

  uint8_t data[4] = {can_id_l, can_id_h, 0x33, rid};
  (void)STM32CAN_Send(hcan, 0x7FFU, data, 4U);
}

/**
 * @brief 请求电机回传当前控制状态。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param id 目标电机的 11 bit 标准帧 ID。
 *
 * 命令统一发送到 0x7FF，数据区为 {ID低8位, ID高3位, 0xCC, 0x00}。
 */
void read_motor_ctrl_fbdata(STM32CAN_t *hcan, uint16_t id) {
  uint8_t can_id_l = id & 0xFF;        // 标准帧 ID 的低 8 bit
  uint8_t can_id_h = (id >> 8) & 0x07; // 标准帧 ID 的高 3 bit

  uint8_t data[4] = {can_id_l, can_id_h, 0xCC, 0x00};
  (void)STM32CAN_Send(hcan, 0x7FFU, data, 4U);
}

/**
 * @brief 发送 32 bit 寄存器写入命令。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param id 目标电机 ID。
 * @param rid 待写入的寄存器编号。
 * @param d0 写入值的最低有效字节。
 * @param d1 写入值的次低字节。
 * @param d2 写入值的次高字节。
 * @param d3 写入值的最高有效字节。
 *
 * 命令统一发送到 0x7FF，数据区为 {ID低4位, ID次低4位, 0x55, rid, d0～d3}。
 * 这里的 ID 拆分方式是当前写寄存器协议实现，与读取命令使用的低8位/高3位形式不同。
 */
void write_motor_data(STM32CAN_t *hcan, uint16_t id, uint8_t rid, uint8_t d0,
                      uint8_t d1, uint8_t d2, uint8_t d3) {
  uint8_t can_id_l = id & 0x0F;
  uint8_t can_id_h = (id >> 4) & 0x0F;

  uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3};
  (void)STM32CAN_Send(hcan, 0x7FFU, data, 8U);
}

/**
 * @brief 请求电机把已写入参数保存到非易失存储器。
 * @param hcan 已初始化的 BSP CAN 设备。
 * @param id 目标电机的 11 bit 标准帧 ID。
 * @param rid 为兼容现有调用签名而保留，当前帧不会使用该参数。
 *
 * 命令统一发送到 0x7FF，数据区固定为 {ID低8位, ID高3位, 0xAA, 0x01}。
 */
void save_motor_data(STM32CAN_t *hcan, uint16_t id, uint8_t rid) {
  uint8_t can_id_l = id & 0xFF;        // 标准帧 ID 的低 8 bit
  uint8_t can_id_h = (id >> 8) & 0x07; // 标准帧 ID 的高 3 bit

  uint8_t data[4] = {can_id_l, can_id_h, 0xAA, 0x01};
  (void)STM32CAN_Send(hcan, 0x7FFU, data, 4U);
}
