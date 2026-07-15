#ifndef DM_MOTOR_DEF_H
#define DM_MOTOR_DEF_H

#include <stdint.h>

typedef struct
{
  uint8_t read_flag;
  uint8_t write_flag;
  uint8_t save_flag;
  float UV_Value;
  float KT_Value;
  float OT_Value;
  float OC_Value;
  float ACC;
  float DEC;
  float MAX_SPD;
  uint32_t MST_ID;
  uint32_t ESC_ID;
  uint32_t TIMEOUT;
  uint32_t cmode;
  float Damp;
  float Inertia;
  uint32_t hw_ver;
  uint32_t sw_ver;
  uint32_t SN;
  uint32_t NPP;
  float Rs;
  float Ls;
  float Flux;
  float Gr;
  float PMAX;
  float VMAX;
  float TMAX;
  float I_BW;
  float KP_ASR;
  float KI_ASR;
  float KP_APR;
  float KI_APR;
  float OV_Value;
  float GREF;
  float Deta;
  float V_BW;
  float IQ_cl;
  float VL_cl;
  uint32_t can_br;
  uint32_t sub_ver;
  float u_off;
  float v_off;
  float k1;
  float k2;
  float m_off;
  float dir;
  float p_m;
  float x_out;
} esc_inf_t;

typedef struct
{
  int id;
  int state;
  int p_int;
  int v_int;
  int t_int;
  int kp_int;
  int kd_int;
  float pos;
  float vel;
  float tor;
  float Kp;
  float Kd;
  float Tmos;
  float Tcoil;
} motor_fbpara_t;

typedef struct
{
  uint8_t mode;
  float pos_set;
  float vel_set;
  float tor_set;
  float cur_set;
  float kp_set;
  float kd_set;
} motor_ctrl_t;

typedef struct
{
  uint16_t id;
  uint16_t mst_id;
  motor_fbpara_t para;
  motor_ctrl_t ctrl;
  esc_inf_t tmp;
} motor_t;

#endif // DM_MOTOR_DEF_H
