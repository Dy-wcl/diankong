#include "convert.h"
#include "dm_imu.h"

imu_t imu;

void imu_init(uint8_t can_id, uint8_t mst_id, CAN_HandleTypeDef *hfdcan) {
  imu.can_id = can_id;
  imu.mst_id = mst_id;
  imu.can_handle = hfdcan;
}

/**
 * @brief 向IMU设备发送命令
 * @param reg_id 要访问的寄存器ID
 * @param ac 访问控制(读/写)
 * @param data 要发送的32位数据
 * @note 通过CAN总线发送8字节的命令帧
 */
static void imu_send_cmd(uint8_t reg_id, uint8_t ac, uint32_t data) {
  // 检查CAN句柄是否已初始化，如果为空则直接返回
  if (imu.can_handle == NULL)
    return;

  // 定义CAN发送消息头结构体
  CAN_TxHeaderTypeDef tx_header;

  // 构建CAN消息数据缓冲区(8字节)
  // 格式: [起始标志(0xCC)][寄存器ID][访问控制(0xDD)][32位数据(小端序)]
  uint8_t buf[8] = {0xCC, reg_id, ac, 0xDD, 0, 0, 0, 0};
  // 将32位数据拷贝到缓冲区的第4-7字节位置
  memcpy(buf + 4, &data, 4);

  // 配置CAN消息头参数
  tx_header.DLC = 8;            // 数据长度码(8字节)
  tx_header.IDE = CAN_ID_STD;   // 使用标准ID格式
  tx_header.StdId = imu.can_id; // 设置标准ID为IMU的CAN ID
  tx_header.ExtId = 0;          // 扩展ID(不使用)
  tx_header.IDE = 0;            // IDE标志(0表示标准帧)
  tx_header.RTR = 0;            // 远程传输请求(0表示数据帧)
  tx_header.DLC = 8;            // 重复设置数据长度码(8字节)

  // 以下是FDCAN相关的配置(当前被注释)
  // tx_header.IdType = FDCAN_STANDARD_ID;
  // tx_header.TxFrameType = FDCAN_DATA_FRAME;
  // tx_header.Identifier = imu.can_id;
  // tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  // tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  // tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  // tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  // tx_header.MessageMarker = 0x00;

  // 尝试使用CAN邮箱0发送消息
  if (HAL_CAN_AddTxMessage(imu.can_handle, &tx_header, buf,
                           (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) {
    // 如果邮箱0忙碌，尝试使用邮箱1发送
    if (HAL_CAN_AddTxMessage(imu.can_handle, &tx_header, buf,
                             (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK) {
      // 如果邮箱1也忙碌，使用邮箱2发送(不检查返回值)
      HAL_CAN_AddTxMessage(imu.can_handle, &tx_header, buf,
                           (uint32_t *)CAN_TX_MAILBOX2);
    }
  }

  // 以下是FDCAN发送方式(当前被注释)
  // if(HAL_FDCAN_GetTxFifoFreeLevel(imu.can_handle) > 2) {
  //     HAL_FDCAN_AddMessageToTxFifoQ(imu.can_handle, &tx_header, buf);
  // }
}

void imu_write_reg(uint8_t reg_id, uint32_t data) {
  imu_send_cmd(reg_id, CMD_WRITE, data);
}

void imu_read_reg(uint8_t reg_id) { imu_send_cmd(reg_id, CMD_READ, 0); }

void imu_reboot() { imu_write_reg(REBOOT_IMU, 0); }

void imu_accel_calibration() { imu_write_reg(ACCEL_CALI, 0); }

void imu_gyro_calibration() { imu_write_reg(GYRO_CALI, 0); }

void imu_change_com_port(imu_com_port_e port) {
  imu_write_reg(CHANGE_COM, (uint8_t)port);
}

void imu_set_active_mode_delay(uint32_t delay) {
  imu_write_reg(SET_DELAY, delay);
}

// 设置成主动模式
void imu_change_to_active() { imu_write_reg(CHANGE_ACTIVE, 1); }

void imu_change_to_request() { imu_write_reg(CHANGE_ACTIVE, 0); }

void imu_set_baud(imu_baudrate_e baud) {
  imu_write_reg(SET_BAUD, (uint8_t)baud);
}

void imu_set_can_id(uint8_t can_id) { imu_write_reg(SET_CAN_ID, can_id); }

void imu_set_mst_id(uint8_t mst_id) { imu_write_reg(SET_MST_ID, mst_id); }

void imu_save_parameters() { imu_write_reg(SAVE_PARAM, 0); }

void imu_restore_settings() { imu_write_reg(RESTORE_SETTING, 0); }

void imu_request_accel() { imu_read_reg(ACCEL_DATA); }

void imu_request_gyro() { imu_read_reg(GYRO_DATA); }

void imu_request_euler() { imu_read_reg(EULER_DATA); }

void imu_request_quat() { imu_read_reg(QUAT_DATA); }

void IMU_UpdateAccel(uint8_t *pData) {
  uint16_t accel[3];

  accel[0] = pData[3] << 8 | pData[2];
  accel[1] = pData[5] << 8 | pData[4];
  accel[2] = pData[7] << 8 | pData[6];

  imu.accel[0] = uint_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
  imu.accel[1] = uint_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
  imu.accel[2] = uint_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
}

void IMU_UpdateGyro(uint8_t *pData) {
  uint16_t gyro[3];

  gyro[0] = pData[3] << 8 | pData[2];
  gyro[1] = pData[5] << 8 | pData[4];
  gyro[2] = pData[7] << 8 | pData[6];

  imu.gyro[0] = uint_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
  imu.gyro[1] = uint_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
  imu.gyro[2] = uint_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
}

void IMU_UpdateEuler(uint8_t *pData) {
  int euler[3];

  euler[0] = pData[3] << 8 | pData[2];
  euler[1] = pData[5] << 8 | pData[4];
  euler[2] = pData[7] << 8 | pData[6];

  imu.pitch = uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
  imu.yaw = uint_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);
  imu.roll = uint_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
}

void IMU_UpdateQuaternion(uint8_t *pData) {
  int w = pData[1] << 6 | ((pData[2] & 0xF8) >> 2);
  int x = (pData[2] & 0x03) << 12 | (pData[3] << 4) | ((pData[4] & 0xF0) >> 4);
  int y = (pData[4] & 0x0F) << 10 | (pData[5] << 2) | (pData[6] & 0xC0) >> 6;
  int z = (pData[6] & 0x3F) << 8 | pData[7];

  imu.q[0] = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
  imu.q[1] = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
  imu.q[2] = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
  imu.q[3] = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);
}

void IMU_UpdateData(uint8_t *pData) {

  switch (pData[0]) {
  case 1:
    IMU_UpdateAccel(pData);
    break;
  case 2:
    IMU_UpdateGyro(pData);
    break;
  case 3:
    IMU_UpdateEuler(pData);
    break;
  case 4:
    IMU_UpdateQuaternion(pData);
    break;
  }
}
