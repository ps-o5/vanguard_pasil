#ifndef __PASIL_ESKF_H
#define __PASIL_ESKF_H

#include <stdint.h>

/* The 7-Dimensional State Vector */
typedef struct {
    /* Unit Quaternion (Attitude) */
    float q0; /* w */
    float q1; /* x */
    float q2; /* y */
    float q3; /* z */
    
    /* Gyroscope Bias Estimate (rad/s) */
    float bg_x;
    float bg_y;
    float bg_z;
} eskf_state_t;

/* Human-Readable Output for the Outer Control Loop */
typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
} euler_angles_t;

/* --- ESKF API --- */
void eskf_init(void);

/* Predict Step: Integrates raw gyro, applies current bias estimate */
void eskf_predict(float gyro_x, float gyro_y, float gyro_z, float dt);

/* Update Step: Corrects Roll/Pitch using gravity vector */
void eskf_update_accel(float ax, float ay, float az);

/* Update Step: Corrects Yaw using tilt-compensated magnetic vector */
void eskf_update_mag(float mx, float my, float mz);

/* Accessors */
eskf_state_t eskf_get_state(void);
euler_angles_t eskf_get_euler(void);

#endif /* __PASIL_ESKF_H */
