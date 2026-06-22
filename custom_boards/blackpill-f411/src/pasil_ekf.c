#include "pasil_ekf.h"
#define ARM_MATH_CM4
#include <arm_math.h>
#include <math.h>

/* --- ESKF Tuning Parameters --- */
#define Q_ANGLE_VAR 0.0001f    /* Gyro noise variance */
#define Q_BIAS_VAR  0.000001f  /* Gyro bias random walk variance */
#define R_ACCEL_VAR 0.01f      /* Accelerometer measurement noise */
#define R_MAG_VAR   0.05f      /* Magnetometer measurement noise */

/* --- State Variables --- */
static eskf_state_t nominal_state;

/* --- CMSIS-DSP Matrix Memory (6x6 Error State) --- */
static float32_t P_data[36];
static float32_t F_data[36], F_T_data[36], Q_data[36];
static float32_t temp6x6_A[36], temp6x6_B[36];

static arm_matrix_instance_f32 P     = {6, 6, P_data};
static arm_matrix_instance_f32 F     = {6, 6, F_data};
static arm_matrix_instance_f32 F_T   = {6, 6, F_T_data};
static arm_matrix_instance_f32 Q     = {6, 6, Q_data};
static arm_matrix_instance_f32 T_6x6A= {6, 6, temp6x6_A};
static arm_matrix_instance_f32 T_6x6B= {6, 6, temp6x6_B};


/* --- Helper: Fast 3x3 Matrix Inverse (Bypasses CMSIS-DSP Linker issues) --- */
static int invert_3x3_f32(float32_t* m, float32_t* invOut) {
    float det = m[0] * (m[4] * m[8] - m[5] * m[7]) -
                m[1] * (m[3] * m[8] - m[5] * m[6]) +
                m[2] * (m[3] * m[7] - m[4] * m[6]);

    if (fabsf(det) < 1e-6f) return -1; /* Singular matrix */

    float invdet = 1.0f / det;

    invOut[0] = (m[4] * m[8] - m[5] * m[7]) * invdet;
    invOut[1] = (m[2] * m[7] - m[1] * m[8]) * invdet;
    invOut[2] = (m[1] * m[5] - m[2] * m[4]) * invdet;
    invOut[3] = (m[5] * m[6] - m[3] * m[8]) * invdet;
    invOut[4] = (m[0] * m[8] - m[2] * m[6]) * invdet;
    invOut[5] = (m[2] * m[3] - m[0] * m[5]) * invdet;
    invOut[6] = (m[3] * m[7] - m[4] * m[6]) * invdet;
    invOut[7] = (m[1] * m[6] - m[0] * m[7]) * invdet;
    invOut[8] = (m[0] * m[4] - m[1] * m[3]) * invdet;

    return 0; /* Success */
}


/* --- Helper: Normalize Quaternion --- */
static void normalize_quaternion(void) {
    float norm = sqrtf(nominal_state.q0 * nominal_state.q0 + 
                       nominal_state.q1 * nominal_state.q1 + 
                       nominal_state.q2 * nominal_state.q2 + 
                       nominal_state.q3 * nominal_state.q3);
    if (norm > 0.0f) {
        nominal_state.q0 /= norm;
        nominal_state.q1 /= norm;
        nominal_state.q2 /= norm;
        nominal_state.q3 /= norm;
    } else {
        nominal_state.q0 = 1.0f; nominal_state.q1 = 0.0f; 
        nominal_state.q2 = 0.0f; nominal_state.q3 = 0.0f;
    }
}

/* --- Helper: Inject Error State into Nominal State --- */
static void inject_error(float32_t* err_state) {
    /* err_state = [dTheta_x, dTheta_y, dTheta_z, db_x, db_y, db_z] */
    float dq0 = 1.0f;
    float dq1 = 0.5f * err_state[0];
    float dq2 = 0.5f * err_state[1];
    float dq3 = 0.5f * err_state[2];

    /* Quaternion Multiplication: q_new = q_old * dq */
    float q0_new = nominal_state.q0*dq0 - nominal_state.q1*dq1 - nominal_state.q2*dq2 - nominal_state.q3*dq3;
    float q1_new = nominal_state.q0*dq1 + nominal_state.q1*dq0 + nominal_state.q2*dq3 - nominal_state.q3*dq2;
    float q2_new = nominal_state.q0*dq2 - nominal_state.q1*dq3 + nominal_state.q2*dq0 + nominal_state.q3*dq1;
    float q3_new = nominal_state.q0*dq3 + nominal_state.q1*dq2 - nominal_state.q2*dq1 + nominal_state.q3*dq0;

    nominal_state.q0 = q0_new;
    nominal_state.q1 = q1_new;
    nominal_state.q2 = q2_new;
    nominal_state.q3 = q3_new;
    normalize_quaternion();

    /* Inject Bias Error */
    nominal_state.bg_x += err_state[3];
    nominal_state.bg_y += err_state[4];
    nominal_state.bg_z += err_state[5];
}

/* =====================================================================
 * PUBLIC API IMPLEMENTATION
 * ===================================================================== */

void eskf_init(void) {
    /* Init Nominal State */
    nominal_state.q0 = 1.0f; nominal_state.q1 = 0.0f; 
    nominal_state.q2 = 0.0f; nominal_state.q3 = 0.0f;
    nominal_state.bg_x = 0.0f; nominal_state.bg_y = 0.0f; nominal_state.bg_z = 0.0f;

    /* Init Matrices */
    for (int i = 0; i < 36; i++) { P_data[i] = 0.0f; Q_data[i] = 0.0f; }
    
    /* P: Initial Covariance (Uncertainty) */
    for (int i = 0; i < 3; i++) P_data[i * 7] = 0.1f;       /* Angle uncertainty */
    for (int i = 3; i < 6; i++) P_data[i * 7] = 0.001f;     /* Bias uncertainty */

    /* Q: Process Noise */
    for (int i = 0; i < 3; i++) Q_data[i * 7] = Q_ANGLE_VAR; 
    for (int i = 3; i < 6; i++) Q_data[i * 7] = Q_BIAS_VAR;  
}

/* --- Helper: Quaternion Derivative for RK4 --- */
static inline void quat_derivative(float q0, float q1, float q2, float q3, 
                                   float wx, float wy, float wz, float* dq) {
    dq[0] = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
    dq[1] = 0.5f * ( q0 * wx - q3 * wy + q2 * wz);
    dq[2] = 0.5f * ( q3 * wx + q0 * wy - q1 * wz);
    dq[3] = 0.5f * (-q2 * wx + q1 * wy + q0 * wz);
}


void eskf_predict(float gyro_x, float gyro_y, float gyro_z, float dt) {
    /* 1. Correct raw gyro with estimated bias */
    float wx = gyro_x - nominal_state.bg_x;
    float wy = gyro_y - nominal_state.bg_y;
    float wz = gyro_z - nominal_state.bg_z;

    /* 2. Nominal State Kinematic Integration (4th-Order Runge-Kutta) */
    float q0 = nominal_state.q0;
    float q1 = nominal_state.q1;
    float q2 = nominal_state.q2;
    float q3 = nominal_state.q3;
    float k1[4], k2[4], k3[4], k4[4];

    /* k1 = f(q, w) */
    quat_derivative(q0, q1, q2, q3, wx, wy, wz, k1);

    /* k2 = f(q + 0.5*dt*k1, w) */
    quat_derivative(q0 + 0.5f * dt * k1[0], 
                    q1 + 0.5f * dt * k1[1], 
                    q2 + 0.5f * dt * k1[2], 
                    q3 + 0.5f * dt * k1[3], wx, wy, wz, k2);

    /* k3 = f(q + 0.5*dt*k2, w) */
    quat_derivative(q0 + 0.5f * dt * k2[0], 
                    q1 + 0.5f * dt * k2[1], 
                    q2 + 0.5f * dt * k2[2], 
                    q3 + 0.5f * dt * k2[3], wx, wy, wz, k3);

    /* k4 = f(q + dt*k3, w) */
    quat_derivative(q0 + dt * k3[0], 
                    q1 + dt * k3[1], 
                    q2 + dt * k3[2], 
                    q3 + dt * k3[3], wx, wy, wz, k4);

    /* Combine and integrate */
    float dt_over_6 = dt / 6.0f;
    nominal_state.q0 += dt_over_6 * (k1[0] + 2.0f * k2[0] + 2.0f * k3[0] + k4[0]);
    nominal_state.q1 += dt_over_6 * (k1[1] + 2.0f * k2[1] + 2.0f * k3[1] + k4[1]);
    nominal_state.q2 += dt_over_6 * (k1[2] + 2.0f * k2[2] + 2.0f * k3[2] + k4[2]);
    nominal_state.q3 += dt_over_6 * (k1[3] + 2.0f * k2[3] + 2.0f * k3[3] + k4[3]);

    normalize_quaternion();

    /* 3. Error State Covariance Predict: P = F * P * F^T + Q */
    for (int i = 0; i < 36; i++) F_data[i] = 0.0f;
    
    /* F_11 = I - [wx]dt (Skew Symmetric) */
    F_data[0] = 1.0f; F_data[1] = wz*dt;  F_data[2] = -wy*dt;
    F_data[6] = -wz*dt; F_data[7] = 1.0f; F_data[8] = wx*dt;
    F_data[12]= wy*dt;  F_data[13]= -wx*dt; F_data[14]= 1.0f;

    /* F_12 = -I * dt (Mapping bias error to angle error) */
    F_data[3] = -dt; F_data[10]= -dt; F_data[17]= -dt;

    /* F_22 = I (Bias is modeled as random walk) */
    F_data[21]= 1.0f; F_data[28]= 1.0f; F_data[35]= 1.0f;

    /* Math: P = F * P * F^T + Q */
    arm_mat_trans_f32(&F, &F_T);
    arm_mat_mult_f32(&F, &P, &T_6x6A);      /* T_6x6A = F * P */
    arm_mat_mult_f32(&T_6x6A, &F_T, &T_6x6B);/* T_6x6B = F * P * F^T */
    arm_mat_add_f32(&T_6x6B, &Q, &P);       /* P = T_6x6B + Q */
}

void eskf_update_accel(float ax, float ay, float az) {
    /* 1. Normalize Measurement */
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if(norm < 0.1f) return; /* Freefall rejection */
    ax /= norm; ay /= norm; az /= norm;

    /* 2. Predicted Gravity from current Quaternion */
    float gx = 2.0f * (nominal_state.q1 * nominal_state.q3 - nominal_state.q0 * nominal_state.q2);
    float gy = 2.0f * (nominal_state.q2 * nominal_state.q3 + nominal_state.q0 * nominal_state.q1);
    float gz = nominal_state.q0*nominal_state.q0 - nominal_state.q1*nominal_state.q1 - nominal_state.q2*nominal_state.q2 + nominal_state.q3*nominal_state.q3;

    /* 3. Innovation (Measurement Residual) */
    float32_t y_data[3] = {ax - gx, ay - gy, az - gz};
    arm_matrix_instance_f32 Y = {3, 1, y_data};

    /* 4. Jacobian H (3x6) for gravity */
    float32_t H_data[18] = {0};
    H_data[0] = 0.0f; H_data[1] = -gz;  H_data[2] = gy;
    H_data[6] = gz;   H_data[7] = 0.0f; H_data[8] = -gx;
    H_data[12]= -gy;  H_data[13]= gx;   H_data[14]= 0.0f;
    
    arm_matrix_instance_f32 H = {3, 6, H_data};
    
    /* 5. Kalman Gain K = P * H^T * (H * P * H^T + R)^-1 */
    float32_t HT_data[18], PHT_data[18], S_data[9], Sinv_data[9], K_data[18];
    arm_matrix_instance_f32 H_T = {6, 3, HT_data};
    arm_matrix_instance_f32 PHT = {6, 3, PHT_data};
    arm_matrix_instance_f32 S   = {3, 3, S_data};
    arm_matrix_instance_f32 Sinv= {3, 3, Sinv_data};
    arm_matrix_instance_f32 K   = {6, 3, K_data};

    arm_mat_trans_f32(&H, &H_T);
    arm_mat_mult_f32(&P, &H_T, &PHT);
    arm_mat_mult_f32(&H, &PHT, &S);

    /* Add R (Measurement Noise) */
    S_data[0] += R_ACCEL_VAR; S_data[4] += R_ACCEL_VAR; S_data[8] += R_ACCEL_VAR;
    
    /* Redundant
    if (arm_mat_inverse_f32(&S, &Sinv) != ARM_MATH_SUCCESS) return;
    arm_mat_mult_f32(&PHT, &Sinv, &K);
    */

    if (invert_3x3_f32(S_data, Sinv_data) != 0) return; /* Matrix singular check */
    arm_mat_mult_f32(&PHT, &Sinv, &K);

    /* 6. Compute Error State: dx = K * y */
    float32_t dx_data[6];
    arm_matrix_instance_f32 dX = {6, 1, dx_data};
    arm_mat_mult_f32(&K, &Y, &dX);

    /* 7. Inject Error and Update Covariance: P = (I - K*H) * P */
    inject_error(dx_data);
    
    float32_t KH_data[36]; arm_matrix_instance_f32 KH = {6, 6, KH_data};
    arm_mat_mult_f32(&K, &H, &KH);
    for(int i=0; i<36; i++) { T_6x6A.pData[i] = -KH_data[i]; }
    T_6x6A.pData[0]+=1.0f; T_6x6A.pData[7]+=1.0f; T_6x6A.pData[14]+=1.0f; 
    T_6x6A.pData[21]+=1.0f; T_6x6A.pData[28]+=1.0f; T_6x6A.pData[35]+=1.0f;
    arm_mat_mult_f32(&T_6x6A, &P, &T_6x6B);
    for(int i=0; i<36; i++) { P.pData[i] = T_6x6B.pData[i]; }
}

void eskf_update_mag(float mx, float my, float mz) {
    /* 1D Yaw Update for Computational Efficiency */
    euler_angles_t current = eskf_get_euler();
    
    /* Tilt Compensation */
    float cos_r = cosf(current.roll_rad);  float sin_r = sinf(current.roll_rad);
    float cos_p = cosf(current.pitch_rad); float sin_p = sinf(current.pitch_rad);
    
    float Xh = mx * cos_p + mz * sin_p;
    float Yh = mx * sin_r * sin_p + my * cos_r - mz * sin_r * cos_p;
    
    float measured_yaw = atan2f(Yh, Xh);
    
    /* Innovation */
    float y = measured_yaw - current.yaw_rad;
    while(y > M_PI) y -= 2.0f * M_PI;
    while(y < -M_PI) y += 2.0f * M_PI;

    /* Jacobian for 1D Yaw */
    //float32_t H_data[6] = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
    //arm_matrix_instance_f32 H = {1, 6, H_data};
    
    /* K = P * H^T / (H * P * H^T + R) */
    float S = P_data[14] + R_MAG_VAR; /* P_data[14] is P(2,2), the Z-axis angle variance */
    
    float32_t dx_data[6];
    for(int i=0; i<6; i++) {
        float K_i = P_data[i*6 + 2] / S; 
        dx_data[i] = K_i * y;
        /* Update P */
        for(int j=0; j<6; j++) {
            T_6x6B.pData[i*6 + j] = P_data[i*6 + j] - K_i * P_data[2*6 + j];
        }
    }
    
    for(int i=0; i<36; i++) P_data[i] = T_6x6B.pData[i];
    inject_error(dx_data);
}

eskf_state_t eskf_get_state(void) {
    return nominal_state;
}

euler_angles_t eskf_get_euler(void) {
    euler_angles_t euler;
    
    /* Quaternion to Euler Angle Conversion */
    //float sqw = nominal_state.q0 * nominal_state.q0;
    float sqx = nominal_state.q1 * nominal_state.q1;
    float sqy = nominal_state.q2 * nominal_state.q2;
    float sqz = nominal_state.q3 * nominal_state.q3;

    /* Roll */
    euler.roll_rad = atan2f(2.0f * (nominal_state.q0 * nominal_state.q1 + nominal_state.q2 * nominal_state.q3), 1.0f - 2.0f * (sqx + sqy));
    
    /* Pitch (Protected against Gimbal Lock NaN) */
    float sinp = 2.0f * (nominal_state.q0 * nominal_state.q2 - nominal_state.q3 * nominal_state.q1);
    if (fabsf(sinp) >= 1.0f) euler.pitch_rad = copysignf(M_PI / 2.0f, sinp); 
    else euler.pitch_rad = asinf(sinp);
    
    /* Yaw */
    euler.yaw_rad = atan2f(2.0f * (nominal_state.q0 * nominal_state.q3 + nominal_state.q1 * nominal_state.q2), 1.0f - 2.0f * (sqy + sqz));

    return euler;
}
