#include "imu.h"
#include <string.h>

dm_imu_t_t imu_485;

void imu_485_data_unpack(uint8_t *pData) {
  normal_packet_t normal_packet;
  normal_ext_packet_t ext_packet;

  memcpy(&normal_packet, pData + 0, 19);

  if (normal_packet.header != 0x55 || normal_packet.tail != 0x0A)
    return;

  if (normal_packet.reg == 0x01) {
    imu_485.accel[0] = normal_packet.data[0];
    imu_485.accel[1] = normal_packet.data[1];
    imu_485.accel[2] = normal_packet.data[2];
  }

  memcpy(&normal_packet, pData + 19, 19);
  if (normal_packet.header != 0x55 || normal_packet.tail != 0x0A)
    return;

  if (normal_packet.reg == 0x02) {
    imu_485.gyro[0] = normal_packet.data[0];
    imu_485.gyro[1] = normal_packet.data[1];
    imu_485.gyro[2] = normal_packet.data[2];
  }

  memcpy(&normal_packet, pData + 38, 19);
  if (normal_packet.header != 0x55 || normal_packet.tail != 0x0A)
    return;

  if (normal_packet.reg == 0x03) {
    imu_485.roll = normal_packet.data[0];
    imu_485.pitch = normal_packet.data[1];
    imu_485.yaw = normal_packet.data[2];
  }

  memcpy(&ext_packet, pData + 57, 23);
  if (ext_packet.header != 0x55 || ext_packet.tail != 0x0A)
    return;

  if (ext_packet.reg == 0x04) {
    imu_485.quaternion[0] = ext_packet.data[0];
    imu_485.quaternion[1] = ext_packet.data[1];
    imu_485.quaternion[2] = ext_packet.data[2];
    imu_485.quaternion[3] = ext_packet.data[3];
  }
}