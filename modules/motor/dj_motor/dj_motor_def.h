#ifndef DJ_MOTOR_DEF_H
#define DJ_MOTOR_DEF_H

#include <stdint.h>

typedef enum
{
  DJ_MOTOR_M3508,
  DJ_MOTOR_M6020,
  DJ_MOTOR_M2006
} dj_motor_type_e;

typedef enum
{
  DJ_MOTOR_1 = 0x201,
  DJ_MOTOR_2 = 0x202,
  DJ_MOTOR_3 = 0x203,
  DJ_MOTOR_4 = 0x204,
  DJ_MOTOR_5 = 0x205,
  DJ_MOTOR_6 = 0x206,
  DJ_MOTOR_7 = 0x207,
  DJ_MOTOR_8 = 0x208
} dj_motor_id_e;

typedef enum
{
  DJ_MOTOR_GROUP_1 = 0x200,
  DJ_MOTOR_GROUP_2 = 0x1FF,
  DJ_MOTOR_GROUP_3 = 0x2FF,
  DJ_MOTOR_GROUP_4 = 0x1FE,
  DJ_MOTOR_GROUP_5 = 0x2FE
} dj_motor_group_e;

typedef struct
{
  uint16_t encoder;
  int16_t rpm_speed;
  int16_t current;
  uint8_t temp;
  uint8_t last_temp;
  float angle;
  float actual_angle_deg;
  float speed_rads;
} dj_motor_feedback_t;

typedef struct
{
  int16_t current_set;
  float speed_set;
  float angle_set;
  uint16_t motor_id;
  dj_motor_type_e type;
} dj_motor_control_t;

typedef struct
{
  dj_motor_id_e motor_id;
  dj_motor_type_e type;
  dj_motor_feedback_t feedback;
  dj_motor_control_t control;
} dj_motor_t;

#define M3508_MAX_CURRENT (16384)
#define M3508_MAX_RPM (8000)
#define M3508_ENCODER_RES (8192U)
#define M3508_REDUCTION_RATIO (19.0f)

#define M6020_MAX_CURRENT (30000)
#define M6020_MAX_RPM (6000)
#define M6020_ENCODER_RES (8192U)
#define M6020_REDUCTION_RATIO (1.0f)

#define M2006_MAX_CURRENT (10000)
#define M2006_MAX_RPM (8000)
#define M2006_ENCODER_RES (8192U)
#define M2006_REDUCTION_RATIO (36.0f)

#endif // DJ_MOTOR_DEF_H
