/*
  VT13自定义遥控器接收模块
*/

#include "vt13.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "crc_ref.h"

static uint8_t dma_buffer_[sizeof(vt13_data_t) * 2u];
static uint8_t vt13_rx_buffer_[sizeof(vt13_data_t)];
static volatile bool frame_ready_ = false;
static vt13_t *instance_ = NULL;

static void VT13_RxCallback(uint8_t *data, size_t size) {
  if ((size != sizeof(vt13_data_t)) || (instance_ == NULL))
    return;
  memcpy(vt13_rx_buffer_, data, sizeof(vt13_data_t));
  frame_ready_ = true;
  if (instance_->thread_alert != NULL) {
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(instance_->thread_alert, SIGNAL_VT13_RAW_REDY, eSetBits,
                       &woken);
    portYIELD_FROM_ISR(woken);
  }
}

static float VT13_NormalizeStick(uint16_t raw) {
  const float full_range = (float)(VT13_CH_VALUE_MAX - VT13_CH_VALUE_MIN);
  float out = 2.0f * ((float)raw - VT13_CH_VALUE_MID) / full_range;
  clampf(&out, -1.0f, 1.0f);
  return out;
}

/**
 * @brief �?1位滚轮通道归一化到[-0.5, 0.5]，与dr16第五通道保持一致量纲�? */
static float VT13_NormalizeWheel(uint16_t raw) {
  const float full_range = (float)(VT13_CH_VALUE_MAX - VT13_CH_VALUE_MIN);
  float out = ((float)raw - VT13_CH_VALUE_MID) / full_range;
  clampf(&out, -0.5f, 0.5f);
  return out;
}

/**
 * @brief 校验VT13原始数据CRC16�? * @note
 * 使用裁判系统CRC16实现，与VT13协议使用相同的CRC16多项式�? */
static bool VT13_VerifyCrc16CheckSum(const vt13_data_t *data) {
  ASSERT(data);

  return Verify_CRC16_Check_Sum((uint8_t *)data, (uint32_t)sizeof(*data)) ==
         TRUE;
}

/**
 * @brief 检查VT13原始数据是否损坏�? */
static bool VT13_DataCorrupted(const vt13_t *vt13) {
  ASSERT(vt13);
  if (vt13->data.sof_1 != 0xA9u)
    return true;
  if (vt13->data.sof_2 != 0x53u)
    return true;
  /* 通道范围检查：11bit摇杆/滚轮均应位于有效区间�?*/
  if ((vt13->data.ch_0 < VT13_CH_VALUE_MIN) ||
      (vt13->data.ch_0 > VT13_CH_VALUE_MAX))
    return true;

  if ((vt13->data.ch_1 < VT13_CH_VALUE_MIN) ||
      (vt13->data.ch_1 > VT13_CH_VALUE_MAX))
    return true;

  if ((vt13->data.ch_2 < VT13_CH_VALUE_MIN) ||
      (vt13->data.ch_2 > VT13_CH_VALUE_MAX))
    return true;

  if ((vt13->data.ch_3 < VT13_CH_VALUE_MIN) ||
      (vt13->data.ch_3 > VT13_CH_VALUE_MAX))
    return true;

  if ((vt13->data.wheel < VT13_CH_VALUE_MIN) ||
      (vt13->data.wheel > VT13_CH_VALUE_MAX))
    return true;

  /* 开�?保留为错误态，0.1.2才是有效位置�?*/
  if (vt13->data.mode_sw > vt13_CMD_SW_DOWN)
    return true;

  /* 按键位虽然定义为2bit，但协议上只允许0/1�?*/
  if (vt13->data.mouse_left > 1u)
    return true;
  if (vt13->data.mouse_right > 1u)
    return true;
  if (vt13->data.mouse_middle > 1u)
    return true;

  /* 双帧头同时为0通常是DMA未更新或总线异常�?*/
  if ((vt13->data.sof_1 == 0u) && (vt13->data.sof_2 == 0u))
    return true;

  if (!VT13_VerifyCrc16CheckSum(&vt13->data))
    return true;

  return false;
}

err_t vt13_init(vt13_t *vt13, UART_HandleTypeDef *uart_handle) {
  if (vt13 == NULL)
    return PTR_NULL;
  memset(vt13, 0, sizeof(*vt13));
  vt13->thread_alert = xTaskGetCurrentTaskHandle();
  vt13->init_error_ = STM32UART_Init(
      &vt13->uart_, uart_handle,
      (BSP_UART_RawData_t){dma_buffer_, sizeof(dma_buffer_)}, VT13_RxCallback);
  instance_ = vt13;
  return vt13->init_error_;
}

err_t vt13_start(vt13_t *vt13) {
  if (vt13 == NULL)
    return PTR_NULL;
  if (vt13->init_error_ != OK)
    return vt13->init_error_;
  return STM32UART_SetRxDMA(&vt13->uart_);
}

void vt13_update(vt13_t *vt13, uint32_t timeout_ms) {
  if (vt13 == NULL)
    return;
  uint32_t notify = 0;
  BaseType_t result = xTaskNotifyWait(SIGNAL_VT13_RAW_REDY, UINT32_MAX, &notify,
                                      pdMS_TO_TICKS(timeout_ms));
  if ((result == pdTRUE) && frame_ready_) {
    frame_ready_ = false;
    memcpy(&vt13->data, vt13_rx_buffer_, sizeof(vt13->data));
    if (vt13_parse_rc(vt13, &vt13->cmd) == OK)
      vt13->online_ = true;
    else {
      vt13->online_ = false;
      memset(&vt13->cmd, 0, sizeof(vt13->cmd));
    }
  } else {
    vt13->online_ = false;
    memset(&vt13->cmd, 0, sizeof(vt13->cmd));
  }
}

err_t vt13_restart(vt13_t *vt13) {
  if ((vt13 == NULL) || (vt13->uart_.uart_handle_ == NULL))
    return PTR_NULL;
  __HAL_UART_DISABLE(vt13->uart_.uart_handle_);
  __HAL_UART_ENABLE(vt13->uart_.uart_handle_);
  return OK;
}

err_t vt13_start_dma_recv(vt13_t *vt13) { return vt13_start(vt13); }

bool vt13_wait_dma_cplt(uint32_t timeout) {
  uint32_t notify = 0;
  return xTaskNotifyWait(SIGNAL_VT13_RAW_REDY, 0, &notify,
                         pdMS_TO_TICKS(timeout)) == pdTRUE;
}

static void VT13_ParseKeyMask(uint16_t key_mask, vt13_cmd_rc_t *rc) {
  ASSERT(rc);

  rc->W = (uint16_t)((key_mask >> CMD_KEY_W) & 0x0001u);
  rc->S = (uint16_t)((key_mask >> CMD_KEY_S) & 0x0001u);
  rc->A = (uint16_t)((key_mask >> CMD_KEY_A) & 0x0001u);
  rc->D = (uint16_t)((key_mask >> CMD_KEY_D) & 0x0001u);
  rc->shift = (uint16_t)((key_mask >> CMD_KEY_SHIFT) & 0x0001u);
  rc->ctrl = (uint16_t)((key_mask >> CMD_KEY_CTRL) & 0x0001u);
  rc->Q = (uint16_t)((key_mask >> CMD_KEY_Q) & 0x0001u);
  rc->E = (uint16_t)((key_mask >> CMD_KEY_E) & 0x0001u);
  rc->R = (uint16_t)((key_mask >> CMD_KEY_R) & 0x0001u);
  rc->F = (uint16_t)((key_mask >> CMD_KEY_F) & 0x0001u);
  rc->G = (uint16_t)((key_mask >> CMD_KEY_G) & 0x0001u);
  rc->Z = (uint16_t)((key_mask >> CMD_KEY_Z) & 0x0001u);
  rc->X = (uint16_t)((key_mask >> CMD_KEY_X) & 0x0001u);
  rc->C = (uint16_t)((key_mask >> CMD_KEY_C) & 0x0001u);
  rc->V = (uint16_t)((key_mask >> CMD_KEY_V) & 0x0001u);
  rc->B = (uint16_t)((key_mask >> CMD_KEY_B) & 0x0001u);
}

static cmd_switch_pos_t VT13_ToCmdSwitch(vt13_cmd_switch_pos_t sw) {
  switch (sw) {
  case vt13_CMD_SW_UP:
    return CMD_SW_UP;
  case vt13_CMD_SW_MID:
    return CMD_SW_MID;
  case vt13_CMD_SW_DOWN:
    return CMD_SW_DOWN;
  default:
    return CMD_SW_ERR;
  }
}

static uint16_t VT13_BuildCompatKeyMask(const vt13_cmd_rc_t *vt13_rc) {
  uint16_t key_mask = 0u;

  ASSERT(vt13_rc != NULL);
  if (vt13_rc == NULL) {
    return 0u;
  }

  key_mask |= (uint16_t)((vt13_rc->W != 0u) << CMD_KEY_W);
  key_mask |= (uint16_t)((vt13_rc->S != 0u) << CMD_KEY_S);
  key_mask |= (uint16_t)((vt13_rc->A != 0u) << CMD_KEY_A);
  key_mask |= (uint16_t)((vt13_rc->D != 0u) << CMD_KEY_D);
  key_mask |= (uint16_t)((vt13_rc->shift != 0u) << CMD_KEY_SHIFT);
  key_mask |= (uint16_t)((vt13_rc->ctrl != 0u) << CMD_KEY_CTRL);
  key_mask |= (uint16_t)((vt13_rc->Q != 0u) << CMD_KEY_Q);
  key_mask |= (uint16_t)((vt13_rc->E != 0u) << CMD_KEY_E);
  key_mask |= (uint16_t)((vt13_rc->R != 0u) << CMD_KEY_R);
  key_mask |= (uint16_t)((vt13_rc->F != 0u) << CMD_KEY_F);
  key_mask |= (uint16_t)((vt13_rc->G != 0u) << CMD_KEY_G);
  key_mask |= (uint16_t)((vt13_rc->Z != 0u) << CMD_KEY_Z);
  key_mask |= (uint16_t)((vt13_rc->X != 0u) << CMD_KEY_X);
  key_mask |= (uint16_t)((vt13_rc->C != 0u) << CMD_KEY_C);
  key_mask |= (uint16_t)((vt13_rc->V != 0u) << CMD_KEY_V);
  key_mask |= (uint16_t)((vt13_rc->B != 0u) << CMD_KEY_B);
  return key_mask;
}

static uint16_t VT13_BuildCompatRes(const vt13_cmd_rc_t *vt13_rc) {
  uint16_t res = 0u;

  ASSERT(vt13_rc != NULL);
  if (vt13_rc == NULL) {
    return 0u;
  }

  if (vt13_rc->func.pause) {
    res |= VT13_COMPAT_RES_PAUSE;
  }
  if (vt13_rc->func.fn_1) {
    res |= VT13_COMPAT_RES_FN_1;
  }
  if (vt13_rc->func.fn_2) {
    res |= VT13_COMPAT_RES_FN_2;
  }
  if (vt13_rc->func.trigger) {
    res |= VT13_COMPAT_RES_TRIGGER;
  }
  if (vt13_rc->mouse_middle) {
    res |= VT13_COMPAT_RES_MOUSE_M;
  }

  return res;
}

err_t vt13_parse_rc(const vt13_t *vt13, vt13_cmd_rc_t *rc) {
  ASSERT(vt13);
  ASSERT(rc);

  if (VT13_DataCorrupted(vt13)) {
    return FAILED;
  }
  memset(rc, 0, sizeof(*rc));

  const uint16_t key_mask = vt13->data.ket;

  rc->ch.r.x = VT13_NormalizeStick((uint16_t)vt13->data.ch_0);
  rc->ch.r.y = VT13_NormalizeStick((uint16_t)vt13->data.ch_1);
  rc->ch.l.x = VT13_NormalizeStick((uint16_t)vt13->data.ch_3);
  rc->ch.l.y = VT13_NormalizeStick((uint16_t)vt13->data.ch_2);

  rc->mode_sw = (vt13_cmd_switch_pos_t)(vt13->data.mode_sw + 1);

  rc->mouse.x = vt13->data.mouse_x;
  rc->mouse.y = vt13->data.mouse_y;
  rc->mouse.z = vt13->data.mouse_z;
  rc->mouse.click.l = vt13->data.mouse_left;
  rc->mouse.click.r = vt13->data.mouse_right;
  rc->mouse_middle = vt13->data.mouse_middle;

  VT13_ParseKeyMask(key_mask, rc);

  rc->wheel = VT13_NormalizeWheel((uint16_t)vt13->data.wheel);
  rc->crc16 = vt13->data.crc16;

  rc->func.pause = vt13->data.pause;
  rc->func.fn_1 = vt13->data.fn_1;
  rc->func.fn_2 = vt13->data.fn_2;
  rc->func.trigger = vt13->data.trigger;

  rc->frame.sof_1 = vt13->data.sof_1;
  rc->frame.sof_2 = vt13->data.sof_2;

  return OK;
}

err_t vt13_cmd_rc_to_cmd_rc(const vt13_cmd_rc_t *vt13_rc, cmd_rc_t *rc) {
  cmd_switch_pos_t sw_l = CMD_SW_ERR;
  cmd_switch_pos_t sw_r = CMD_SW_UP;

  ASSERT(vt13_rc != NULL);
  ASSERT(rc != NULL);
  if ((vt13_rc == NULL) || (rc == NULL)) {
    return PTR_NULL;
  }

  memset(rc, 0, sizeof(*rc));
  rc->res = VT13_BuildCompatRes(vt13_rc);

  /*
   * pause 被定义为最高优先级安全动作�?   *
   * 一旦按下，直接把兼容遥控视图压成“停止挡”，这样即使上层没有识别 VT13
   * 的扩展位，也会进入安全态�?   */
  if (vt13_rc->func.pause) {
    rc->sw_l = CMD_SW_UP;
    rc->sw_r = CMD_SW_UP;
    return OK;
  }

  sw_l = VT13_ToCmdSwitch(vt13_rc->mode_sw);
  if (sw_l == CMD_SW_ERR) {
    return FAILED;
  }

  /*
   * VT13 只有一个三挡模式拨杆，所以这里把独有功能位映射成右拨杆子模式�?   * -
   * 默认：UP
   * - fn_1：MID
   * - fn_2 �?trigger：DOWN
   *
   * 这样 joint 现有�?sw_l / sw_r 状态机就能直接复用，不需要把整个关节控制
   * 逻辑重写一遍�?   */
  if (vt13_rc->func.fn_1) {
    sw_r = CMD_SW_MID;
  }
  if (vt13_rc->func.fn_2 || vt13_rc->func.trigger) {
    sw_r = CMD_SW_DOWN;
  }

  rc->ch.l.x = vt13_rc->ch.l.x;
  rc->ch.l.y = vt13_rc->ch.l.y;
  rc->ch.r.x = vt13_rc->ch.r.x;
  rc->ch.r.y = vt13_rc->ch.r.y;
  rc->ch_res = vt13_rc->wheel;
  rc->sw_l = sw_l;
  rc->sw_r = sw_r;
  rc->mouse = vt13_rc->mouse;
  rc->key = VT13_BuildCompatKeyMask(vt13_rc);
  return OK;
}

err_t vt13_handle_offline(const vt13_t *vt13, vt13_cmd_rc_t *rc) {
  ASSERT(vt13);
  ASSERT(rc);

  RM_UNUSED(vt13);
  memset(rc, 0, sizeof(*rc));
  return OK;
}
