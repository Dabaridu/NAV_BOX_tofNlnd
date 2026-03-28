#ifndef SENS_FUSION_H
#define SENS_FUSION_H

#include "main.h"
#include <math.h>

#define RUNING_AVG_BUFF 100
#define GIMBAL_LOCK_THRESHOLD 1e-6

typedef struct {
    double w;
    double x;
    double y;
    double z;
} vector_t;

typedef struct {
    double roll;   // rotation about X
    double pitch;  // rotation about Y
    double yaw;    // rotation about Z
    uint8_t gimbal_locked; // flag
} euler_t;

/* ── Kalman state vector ──
 *  Full 9-state model:
 *    [px, py, pz,    position     (meters)
 *     vx, vy, vz,    velocity     (m/s)
 *     ax, ay, az]    acceleration (m/s²) — modeled as slowly changing
 */
#define KF_STATES 9
#define KF_POS_MEAS 3  // GPS measures px, py, pz

typedef struct {
    // State vector [9x1]
    double x[KF_STATES];

    // Covariance matrix [9x9]
    double P[KF_STATES][KF_STATES];

    // Process noise covariance [9x9]
    double Q[KF_STATES][KF_STATES];

    // GPS measurement noise covariance [3x3]
    double R_gps[KF_POS_MEAS][KF_POS_MEAS];

    // IMU accelerometer noise (σ² per axis)
    double imu_acc_variance;

    // Timestamp tracking
    double last_time_s;
    uint8_t initialized;
} kalman_state_t;

/* ── Global variables ── */
extern vector_t acc;
extern vector_t acc_prec;
extern vector_t abs_q;
extern vector_t pos;
extern vector_t pos_prec;

extern vector_t X_global_translation;
extern vector_t X_global_translation_precision;
extern euler_t  O_global_orientation_euler;

extern kalman_state_t kf;

extern vector_t acc_f;
extern int ts;
extern int lt;

extern double starting_position_offset[3];
extern double starting_orientation_offset[3][3];

/* ── API ── */

// Kalman filter
void kalman_init(kalman_state_t *kf,
                 double imu_acc_var,
                 double gps_pos_var,
                 double initial_time_s);
void kalman_predict_imu(kalman_state_t *kf,
                        vector_t *acc_global,
                        double dt);
void kalman_update_gps(kalman_state_t *kf,
                       double gps_x, double gps_y, double gps_z);
void kalman_get_position(kalman_state_t *kf, vector_t *out);
void kalman_get_velocity(kalman_state_t *kf, vector_t *out);

// IMU orientation + position (non-KF path kept for reference)
void update_IMU_global_position(
        double sp[3], double so[3][3],
        vector_t *acc, vector_t *abs_q,
        int time_step,
        vector_t *X_global_translation,
        euler_t  *O_global_orientation_euler);

// Quaternion to rotation matrix (fills 3x3 inside 4x4, row-major)
void quaternion_to_rotation_3x3(vector_t *q, double R[3][3]);

// Euler extraction with gimbal-lock guard
euler_t rotation_to_euler_safe(double R[3][3]);

// Utility
double update_ts(int *last_time, int current_time);
void   matrix_multiply(double A[4][4], double B[4][4], double result[4][4]);
void   vector_t_moving_average_filter(vector_t *input, vector_t *output);

#endif /* SENS_FUSION_H */