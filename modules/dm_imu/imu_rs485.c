#include "imu_rs485.h"

#include <string.h>

#include "comp_utils.h"

static uint8_t dma_buffer_[IMU485_DMA_BUFFER_SIZE];
static uint8_t imu_rx_buffer_[IMU485_FRAME_SIZE];
static uint8_t frame_assembler_[IMU485_FRAME_SIZE];
static size_t frame_index_ = 0u;
static volatile bool frame_ready_ = false;
static IMU485_t *instance_ = NULL;

dm_imu_t_t imu_485;

static bool IMU485_CheckPacket(const uint8_t *packet, size_t size,
                               uint8_t expected_reg)
{
  if ((packet == NULL) || (size < 1u) || (packet[0] != IMU485_HEADER) ||
      (packet[size - 1u] != IMU485_TAIL)) {
    return false;
  }
  return (size < 4u) || (packet[3] == expected_reg);
}

static bool IMU485_DecodeFrame(const uint8_t *data, dm_imu_t_t *decoded)
{
  normal_packet_t normal;
  normal_ext_packet_t extended;

  if ((data == NULL) || (decoded == NULL) ||
      !IMU485_CheckPacket(data, IMU485_NORMAL_PACKET_SIZE, 1u) ||
      !IMU485_CheckPacket(data + 19u, IMU485_NORMAL_PACKET_SIZE, 2u) ||
      !IMU485_CheckPacket(data + 38u, IMU485_NORMAL_PACKET_SIZE, 3u) ||
      !IMU485_CheckPacket(data + 57u, IMU485_EXT_PACKET_SIZE, 4u)) {
    return false;
  }

  memcpy(&normal, data, sizeof(normal));
  decoded->accel[0] = normal.data[0];
  decoded->accel[1] = normal.data[1];
  decoded->accel[2] = normal.data[2];

  memcpy(&normal, data + 19u, sizeof(normal));
  decoded->gyro[0] = normal.data[0];
  decoded->gyro[1] = normal.data[1];
  decoded->gyro[2] = normal.data[2];

  memcpy(&normal, data + 38u, sizeof(normal));
  decoded->roll = normal.data[0];
  decoded->pitch = normal.data[1];
  decoded->yaw = normal.data[2];

  memcpy(&extended, data + 57u, sizeof(extended));
  decoded->quaternion[0] = extended.data[0];
  decoded->quaternion[1] = extended.data[1];
  decoded->quaternion[2] = extended.data[2];
  decoded->quaternion[3] = extended.data[3];
  return true;
}

void imu_485_data_unpack(const uint8_t *pData)
{
  dm_imu_t_t decoded;

  if (IMU485_DecodeFrame(pData, &decoded)) {
    imu_485 = decoded;
  }
}

static void IMU485_ResetAssembler(uint8_t byte)
{
  frame_index_ = (byte == IMU485_HEADER) ? 1u : 0u;
  if (frame_index_ == 1u) {
    frame_assembler_[0] = byte;
  }
}

static void IMU485_RxCallback(uint8_t *data, size_t size)
{
  if (data == NULL) {
    return;
  }

  for (size_t i = 0u; i < size; ++i) {
    const uint8_t byte = data[i];
    if ((frame_index_ == 0u) && (byte != IMU485_HEADER)) {
      continue;
    }

    if (frame_index_ >= sizeof(frame_assembler_)) {
      IMU485_ResetAssembler(byte);
      continue;
    }
    frame_assembler_[frame_index_++] = byte;

    const size_t packet_end = (frame_index_ == 19u) ? 19u :
                              (frame_index_ == 38u) ? 38u :
                              (frame_index_ == 57u) ? 57u :
                              (frame_index_ == IMU485_FRAME_SIZE) ? IMU485_FRAME_SIZE : 0u;
    if (packet_end != 0u) {
      const uint8_t expected_reg = (packet_end == 19u) ? 1u :
                                   (packet_end == 38u) ? 2u :
                                   (packet_end == 57u) ? 3u : 4u;
      const size_t packet_start = packet_end - ((packet_end == IMU485_FRAME_SIZE) ?
                                                IMU485_EXT_PACKET_SIZE :
                                                IMU485_NORMAL_PACKET_SIZE);
      if (!IMU485_CheckPacket(frame_assembler_ + packet_start,
                              packet_end - packet_start, expected_reg)) {
        IMU485_ResetAssembler(byte);
        continue;
      }
    }

    if (frame_index_ == IMU485_FRAME_SIZE) {
      memcpy(imu_rx_buffer_, frame_assembler_, IMU485_FRAME_SIZE);
      frame_ready_ = true;
      frame_index_ = 0u;
      if ((instance_ != NULL) && (instance_->thread_alert != NULL)) {
        BaseType_t woken = pdFALSE;
        xTaskNotifyFromISR(instance_->thread_alert, SIGNAL_IMU485_RAW_READY,
                           eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
      }
    }
  }
}

err_t imu_485_init(IMU485_t *self, UART_HandleTypeDef *uart_handle)
{
  if (self == NULL) {
    return PTR_NULL;
  }

  memset(self, 0, sizeof(*self));
  frame_index_ = 0u;
  frame_ready_ = false;
  self->init_error_ = STM32UART_Init(
      &self->uart_, uart_handle,
      (BSP_UART_RawData_t){dma_buffer_, sizeof(dma_buffer_)}, IMU485_RxCallback);
  instance_ = self;
  return self->init_error_;
}

err_t imu_485_start(IMU485_t *self)
{
  if (self == NULL) {
    return PTR_NULL;
  }
  if (self->init_error_ != OK) {
    return self->init_error_;
  }
  return STM32UART_SetRxDMA(&self->uart_);
}

void imu_485_update(IMU485_t *self, uint32_t timeout_ms)
{
  if (self == NULL) {
    return;
  }

  uint32_t notify_value = 0u;
  const BaseType_t result = xTaskNotifyWait(
      SIGNAL_IMU485_RAW_READY, UINT32_MAX, &notify_value,
      pdMS_TO_TICKS(timeout_ms));
  if ((result == pdTRUE) && frame_ready_) {
    frame_ready_ = false;
    memcpy(self->raw_frame, imu_rx_buffer_, IMU485_FRAME_SIZE);
    dm_imu_t_t decoded;
    if (IMU485_DecodeFrame(self->raw_frame, &decoded)) {
      self->data = decoded;
      imu_485 = decoded;
      self->online_ = true;
      return;
    }
  }

  self->online_ = false;
}
