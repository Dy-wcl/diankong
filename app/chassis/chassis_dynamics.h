#ifndef APP_CHASSIS_DYNAMICS_H
#define APP_CHASSIS_DYNAMICS_H
#define CHASSIS_DYNAMICS_WHEEL_COUNT (4U)

/* 输出顺序固定为 [FL, FR, RL, RR] = [4, 3, 2, 1]。 */
void chassis_dynamics_set_attitude(float yaw, float pitch, float roll,
                                   float motor_yaw, float motor_pitch);
void chassis_dynamics_inverse(float vx, float vy, float wz,
                              float out[CHASSIS_DYNAMICS_WHEEL_COUNT]);
void chassis_dynamics_feedforward(
    float out_current[CHASSIS_DYNAMICS_WHEEL_COUNT]);
#endif
