#ifndef _vt13_h
#define _vt13_h

#include "comp_cmd.h"
#include "comp_utils.h"

/*
  SIGNAL是用bit位置区分的，一个SIGNAL只能包含一个高bit位。
  不同的SIGNAL不能有相同的高bit位。
*/
#define SIGNAL_VT13_RAW_REDY (1u << 6)

/* VT13 11位通道原始值范围 */
#define VT13_CH_VALUE_MIN (364u)
#define VT13_CH_VALUE_MID (1024u)
#define VT13_CH_VALUE_MAX (1684u)

/* 将 VT13 独有功能位压缩到 cmd_rc.res 中，方便双板链路继续保留这些信息。 */
#define VT13_COMPAT_RES_PAUSE (1u << 0)
#define VT13_COMPAT_RES_FN_1 (1u << 1)
#define VT13_COMPAT_RES_FN_2 (1u << 2)
#define VT13_COMPAT_RES_TRIGGER (1u << 3)
#define VT13_COMPAT_RES_MOUSE_M (1u << 4)

typedef struct __attribute__((packed)) {
  uint8_t sof_1;
  uint8_t sof_2;
  uint64_t ch_0 : 11;
  uint64_t ch_1 : 11;
  uint64_t ch_2 : 11;
  uint64_t ch_3 : 11;
  uint64_t mode_sw : 2;
  uint64_t pause : 1;
  uint64_t fn_1 : 1;
  uint64_t fn_2 : 1;
  uint64_t wheel : 11;
  uint64_t trigger : 1;

  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  uint8_t mouse_left : 2;
  uint8_t mouse_right : 2;
  uint8_t mouse_middle : 2;
  uint16_t ket;
  uint16_t crc16;
} vt13_data_t;

typedef struct __attribute__((packed)) {
  bool pause;
  bool fn_1;
  bool fn_2;
  bool trigger;
} vt13_func_t;

typedef struct __attribute__((packed)) {
  struct {
    vector2_t l; /* 遥控器左侧摇杆横轴值(x)，上为正；纵轴值(y)，右为正 */
    vector2_t r; /* 遥控器右侧摇杆横轴值(x)，上为正；纵轴值(y)，右为正 */
  } ch;

  float wheel;                   /* VT13滚轮值，归一化到[-0.5, 0.5] */
  vt13_cmd_switch_pos_t mode_sw; /* VT13模式拨杆位置 */

  cmd_mouse_t mouse; /* 鼠标值，保留原始左右键语义 */
  bool mouse_middle; /* 鼠标中键 */
  uint16_t W;
  uint16_t S;
  uint16_t A;
  uint16_t D;
  uint16_t shift;
  uint16_t ctrl;
  uint16_t Q;
  uint16_t E;
  uint16_t R;
  uint16_t F;
  uint16_t G;
  uint16_t Z;
  uint16_t X;
  uint16_t C;
  uint16_t V;
  uint16_t B;
  // uint16_t key;   /* 按键值，位序与CMD_KEY_*保持一致 */
  uint16_t crc16; /* 原始帧CRC16 */

  vt13_func_t func; /* VT13专有功能位 */

  struct {
    uint8_t sof_1;
    uint8_t sof_2;
  } frame; /* 原始帧头 */
} vt13_cmd_rc_t;

typedef struct __attribute__((packed)) {
  vt13_data_t data;
} vt13_t;

err_t vt13_init(vt13_t *vt13);
err_t vt13_restart(void);
err_t vt13_start_dma_recv(vt13_t *vt13);
bool vt13_wait_dma_cplt(uint32_t timeout);
err_t vt13_parse_rc(const vt13_t *vt13, vt13_cmd_rc_t *rc);
err_t vt13_cmd_rc_to_cmd_rc(const vt13_cmd_rc_t *vt13_rc, cmd_rc_t *rc);
err_t vt13_handle_offline(const vt13_t *vt13, vt13_cmd_rc_t *rc);

#endif
