#ifndef DM_IMU_RS485_H
#define DM_IMU_RS485_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "bsp_uart.h"
#include "comp_def.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU485_NORMAL_PACKET_SIZE (19u)
#define IMU485_EXT_PACKET_SIZE (23u)
#define IMU485_FRAME_SIZE (80u)
#define IMU485_DMA_BUFFER_SIZE (IMU485_FRAME_SIZE * 2u)
#define IMU485_OFFLINE_TIMEOUT_MS (20u)
#define IMU485_HEADER (0x55u)
#define IMU485_TAIL (0x0Au)

typedef struct {
  float accel[3];
  float gyro[3];
  float roll;
  float pitch;
  float yaw;
  float quaternion[4];
} dm_imu_t_t;

/* Wire layouts are packed: 4-byte floats follow the four protocol bytes. */
typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t tag;
  uint8_t slave_id;
  uint8_t reg;
  float data[3];
  uint16_t crc;
  uint8_t tail;
} normal_packet_t;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t tag;
  uint8_t slave_id;
  uint8_t reg;
  float data[4];
  uint16_t crc;
  uint8_t tail;
} normal_ext_packet_t;

_Static_assert(sizeof(normal_packet_t) == IMU485_NORMAL_PACKET_SIZE,
               "normal IMU packet layout must match the wire format");
_Static_assert(sizeof(normal_ext_packet_t) == IMU485_EXT_PACKET_SIZE,
               "extended IMU packet layout must match the wire format");

typedef struct {
  TaskHandle_t thread_alert;
  dm_imu_t_t data;
  bool online_;
  STM32UART_t uart_;
  err_t init_error_;
  uint8_t raw_frame[IMU485_FRAME_SIZE];
} IMU485_t;

/* Compatibility global used by existing control code. */
extern dm_imu_t_t imu_485;

err_t imu_485_init(IMU485_t *self, UART_HandleTypeDef *uart_handle);
err_t imu_485_start(IMU485_t *self);
void imu_485_update(IMU485_t *self, uint32_t timeout_ms);

/* Decode one complete 80-byte aggregate into the compatibility global. */
void imu_485_data_unpack(const uint8_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* DM_IMU_RS485_H */
