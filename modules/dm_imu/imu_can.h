#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "can.h"
#include "main.h"


#define ACCEL_CAN_MAX (235.2f)
#define ACCEL_CAN_MIN (-235.2f)
#define GYRO_CAN_MAX (34.88f)
#define GYRO_CAN_MIN (-34.88f)
#define PITCH_CAN_MAX (90.0f)
#define PITCH_CAN_MIN (-90.0f)
#define ROLL_CAN_MAX (180.0f)
#define ROLL_CAN_MIN (-180.0f)
#define YAW_CAN_MAX (180.0f)
#define YAW_CAN_MIN (-180.0f)
#define TEMP_MIN (0.0f)
#define TEMP_MAX (60.0f)
#define Quaternion_MIN (-1.0f)
#define Quaternion_MAX (1.0f)

#define CMD_READ 0  // 读
#define CMD_WRITE 1 // 写
#define imu__can_id 0x10
#define imu__mst_id 0x20
typedef enum {
  COM_USB = 0,
  COM_RS485,
  COM_CAN,
  COM_VOFA

} imu_com_port_e;

typedef enum {
  CAN_BAUD_1M = 0, // 1M
  CAN_BAUD_500K,   // 500K
  CAN_BAUD_400K,   // 400K
  CAN_BAUD_250K,   // 250K
  CAN_BAUD_200K,   // 200K
  CAN_BAUD_100K,   // 100K
  CAN_BAUD_50K,    // 50K
  CAN_BAUD_25K     // 25K

} imu_baudrate_e;

typedef enum {
  REBOOT_IMU = 0,        // 重启 IMU
  ACCEL_DATA,            // 加速度数据
  GYRO_DATA,             // 角速度数据
  EULER_DATA,            // 欧拉角数据
  QUAT_DATA,             // 四元数数据
  SET_ZERO,              // 角度置零
  ACCEL_CALI,            // 加计六面校准
  GYRO_CALI,             // 陀螺静态校准
  MAG_CALI,              // 磁计椭球校准
  CHANGE_COM,            // 切换通信模式
  SET_DELAY,             // 设置主动发送间隔
  CHANGE_ACTIVE,         // 切换主被动模式
  SET_BAUD,              // 修改波特率
  SET_CAN_ID,            // 修改CAN ID
  SET_MST_ID,            // MST_ID
  DATA_OUTPUT_SELECTION, // 输出数据选择
  SAVE_PARAM = 254,
  RESTORE_SETTING = 255
} reg_id_e;

typedef struct {
  uint8_t can_id;
  uint8_t mst_id;

  CAN_HandleTypeDef *can_handle;

  float pitch;
  float roll;
  float yaw;

  float gyro[3];
  float accel[3];

  float q[4];

  float cur_temp;

} imu_t;

void imu_init(uint8_t can_id, uint8_t mst_id, CAN_HandleTypeDef *hfdcan);
void imu_write_reg(uint8_t reg_id, uint32_t data);
void imu_read_reg(uint8_t reg_id);
void imu_reboot();
void imu_accel_calibration();
void imu_gyro_calibration();
void imu_change_com_port(imu_com_port_e port);
void imu_set_active_mode_delay(uint32_t delay);
void imu_change_to_active();
void imu_change_to_request();
void imu_set_baud(imu_baudrate_e baud);
void imu_set_can_id(uint8_t can_id);
void imu_set_mst_id(uint8_t mst_id);
void imu_save_parameters();
void imu_restore_settings();
void imu_request_accel();
void imu_request_gyro();
void imu_request_euler();
void imu_request_quat();
void IMU_UpdateData(uint8_t *pData);
#endif
