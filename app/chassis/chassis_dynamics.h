#ifndef APP_CHASSIS_DYNAMICS_H
#define APP_CHASSIS_DYNAMICS_H
#include <stdint.h>
void chassis_dynamics_set_attitude(float yaw, float pitch, float roll,
                                   float motor_yaw, float motor_pitch);
void chassis_dynamics_inverse(float vx, float vy, float wz, float out[4]);
void chassis_dynamics_feedforward(float out_current[4]);
#endif
