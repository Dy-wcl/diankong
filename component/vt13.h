#pragma once

#include <stdint.h>
#include "comp_cmd.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief VT13 遥控器命令结构
 * @note 临时定义，需要根据实际 VT13 协议调整
 */
typedef struct {
  struct {
    vector2_t l;
    vector2_t r;
  } ch;

  vt13_cmd_switch_pos_t sw_l;
  vt13_cmd_switch_pos_t sw_r;
  vt13_cmd_switch_pos_t mode_sw;  // 模式开关

  uint8_t A;
  uint8_t B;
  uint8_t C;
  uint8_t D;
  uint8_t X;
  uint8_t Y;

  struct {
    uint8_t fn_1;
    uint8_t fn_2;
  } func;

  uint16_t res;
} vt13_cmd_rc_t;

#ifdef __cplusplus
}
#endif
