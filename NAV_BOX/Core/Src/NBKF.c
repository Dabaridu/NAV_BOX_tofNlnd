#include "NBKF.h"
#include <math.h>
#include <string.h>

#define EARTH_RADIUS 6378137.0f
#define DEG_TO_RAD (3.14159265359f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.14159265359f)

// Matrix operations helpers
static void matrix_multiply(float *A, float *B, float *C, int m, int n, int p);
static void matrix_add(float *A, float *B, float *C, int rows, int cols);
static void matrix_transpose(float *A, float *AT, int rows, int cols);
static void matrix_identity(float *A, int size);
static int matrix_inverse(float *A, float *Ainv, int n);

void NBKF_Init(NBKF_t *kf, float dt) {
    memset(kf, 0, sizeof(NBKF_t));
    kf->dt = dt;
    
    // Initialize state covariance (P) with high uncertainty
    for (int i = 0; i < STATE_SIZE; i++) {
        kf->P[i][i] = 1000.0f;
    }
    
    // Set default process noise
    NBKF_SetProcessNoise(kf, 0.1f, 0.5f, 0.01f);
    
    // Set default measurement noise
    NBKF_SetMeasurementNoise(kf, 5.0f, 0.1f);
}

void NBKF_SetProcessNoise(NBKF_t *kf, float pos_noise, float vel_noise, float att_noise) {
    memset(kf->Q, 0, sizeof(kf->Q));
    
    // Position noise (x, y, z)
    for (int i = 0; i < 3; i++) {
        kf->Q[i][i] = pos_noise * pos_noise;
    }
    
    // Velocity noise (vx, vy, vz)
    for (int i = 3; i < 6; i++) {
        kf->Q[i][i] = vel_noise * vel_noise;
    }
    
    // Attitude noise (roll, pitch, yaw)
    for (int i = 6; i < 9; i++) {
        kf->Q[i][i] = att_noise * att_noise;
    }
}

void NBKF_SetMeasurementNoise(NBKF_t *kf, float gps_noise, float imu_noise) {
    memset(kf->R, 0, sizeof(kf->R));
    
    // GPS noise (lat, lon, alt)
    for (int i = 0; i < 3; i++) {
        kf->R[i][i] = gps_noise * gps_noise;
    }
    
    // IMU noise (accel, gyro, mag)
    for (int i = 3; i < MEAS_SIZE; i++) {
        kf->R[i][i] = imu_noise * imu_noise;
    }
}

void NBKF_Predict(NBKF_t *kf, IMU_Data_t *imu) {
    float dt = kf->dt;
    
    // State transition: predict position from velocity
    kf->state[0] += kf->state[3] * dt; // x
    kf->state[1] += kf->state[4] * dt; // y
    kf->state[2] += kf->state[5] * dt; // z
    
    // Predict velocity from accelerometer (compensate for orientation)
    float roll = kf->state[6];
    float pitch = kf->state[7];
    float yaw = kf->state[8];
    
    // Rotation matrix to convert body frame to earth frame
    float cr = cosf(roll), sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw), sy = sinf(yaw);
    
    float ax_earth = imu->ax * (cy*cp) + imu->ay * (cy*sp*sr - sy*cr) + imu->az * (cy*sp*cr + sy*sr);
    float ay_earth = imu->ax * (sy*cp) + imu->ay * (sy*sp*sr + cy*cr) + imu->az * (sy*sp*cr - cy*sr);
    float az_earth = imu->ax * (-sp) + imu->ay * (cp*sr) + imu->az * (cp*cr) - 9.81f;
    
    kf->state[3] += ax_earth * dt; // vx
    kf->state[4] += ay_earth * dt; // vy
    kf->state[5] += az_earth * dt; // vz
    
    // Predict attitude from gyroscope
    kf->state[6] += imu->gx * dt; // roll
    kf->state[7] += imu->gy * dt; // pitch
    kf->state[8] += imu->gz * dt; // yaw
    
    // Normalize angles to [-pi, pi]
    for (int i = 6; i < 9; i++) {
        while (kf->state[i] > 3.14159f) kf->state[i] -= 6.28318f;
        while (kf->state[i] < -3.14159f) kf->state[i] += 6.28318f;
    }
    
    // Jacobian (simplified linear approximation)
    float F[STATE_SIZE][STATE_SIZE];
    matrix_identity((float*)F, STATE_SIZE);
    F[0][3] = dt;
    F[1][4] = dt;
    F[2][5] = dt;
    
    // P = F * P * F^T + Q
    float FP[STATE_SIZE][STATE_SIZE];
    float FT[STATE_SIZE][STATE_SIZE];
    matrix_transpose((float*)F, (float*)FT, STATE_SIZE, STATE_SIZE);
    matrix_multiply((float*)F, (float*)kf->P, (float*)FP, STATE_SIZE, STATE_SIZE, STATE_SIZE);
    matrix_multiply((float*)FP, (float*)FT, (float*)kf->P, STATE_SIZE, STATE_SIZE, STATE_SIZE);
    matrix_add((float*)kf->P, (float*)kf->Q, (float*)kf->P, STATE_SIZE, STATE_SIZE);
}

void NBKF_Update(NBKF_t *kf, GPS_Data_t *gps, IMU_Data_t *imu) {
    if (!gps->valid) return;
    
    // Convert GPS coordinates to local ENU coordinates
    float x_gps = (gps->longitude - kf->state[0] * RAD_TO_DEG) * DEG_TO_RAD * EARTH_RADIUS * cosf(gps->latitude * DEG_TO_RAD);
    float y_gps = (gps->latitude - kf->state[1] * RAD_TO_DEG) * DEG_TO_RAD * EARTH_RADIUS;
    float z_gps = gps->altitude;
    
    // Measurement vector
    float z[MEAS_SIZE] = {
        x_gps, y_gps, z_gps,
        imu->ax, imu->ay, imu->az,
        imu->gx, imu->gy, imu->gz,
        imu->mx, imu->my, imu->mz
    };
    
    // Predicted measurement (simplified)
    float h[MEAS_SIZE] = {
        kf->state[0], kf->state[1], kf->state[2], // Position
        0, 0, 9.81f, // Expected accel in body frame (simplified)
        0, 0, 0,     // Expected gyro
        0, 0, 0      // Expected mag (simplified)
    };
    
    // Innovation: y = z - h
    float y[MEAS_SIZE];
    for (int i = 0; i < MEAS_SIZE; i++) {
        y[i] = z[i] - h[i];
    }
    
    // Simplified Kalman gain (K = P * H^T * inv(H * P * H^T + R))
    // Update state: x = x + K * y
    float K_simple[3] = {0.3f, 0.3f, 0.3f}; // Simplified gains for GPS
    kf->state[0] += K_simple[0] * y[0];
    kf->state[1] += K_simple[1] * y[1];
    kf->state[2] += K_simple[2] * y[2];
    
    // Update covariance: P = (I - K*H) * P (simplified)
    for (int i = 0; i < 3; i++) {
        kf->P[i][i] *= (1.0f - K_simple[i]);
    }
}

void NBKF_GetOutput(NBKF_t *kf, NBKF_Output_t *output) {
    // Convert local ENU to lat/lon
    output->latitude = kf->state[1] * RAD_TO_DEG;
    output->longitude = kf->state[0] * RAD_TO_DEG;
    output->altitude = kf->state[2];
    
    output->vx = kf->state[3];
    output->vy = kf->state[4];
    output->vz = kf->state[5];
}

// Matrix operation implementations (simplified)
static void matrix_multiply(float *A, float *B, float *C, int m, int n, int p) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            C[i*p + j] = 0;
            for (int k = 0; k < n; k++) {
                C[i*p + j] += A[i*n + k] * B[k*p + j];
            }
        }
    }
}

static void matrix_add(float *A, float *B, float *C, int rows, int cols) {
    for (int i = 0; i < rows * cols; i++) {
        C[i] = A[i] + B[i];
    }
}

static void matrix_transpose(float *A, float *AT, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            AT[j*rows + i] = A[i*cols + j];
        }
    }
}

static void matrix_identity(float *A, int size) {
    memset(A, 0, size * size * sizeof(float));
    for (int i = 0; i < size; i++) {
        A[i*size + i] = 1.0f;
    }
}

static int matrix_inverse(float *A, float *Ainv, int n) {
    // Simplified - not implemented for full matrix operations
    // In production, use proper numerical library
    return 0;
}