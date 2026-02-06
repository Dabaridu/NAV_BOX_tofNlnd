#ifndef NBKF_H
#define NBKF_H

#include <stdint.h>

// State vector size: [x, y, z, vx, vy, vz, roll, pitch, yaw]
#define STATE_SIZE 9
// Measurement vector size: [lat, lon, alt, ax, ay, az, gx, gy, gz, mx, my, mz]
#define MEAS_SIZE 12

typedef struct {
    float state[STATE_SIZE];           // State vector
    float P[STATE_SIZE][STATE_SIZE];   // Error covariance matrix
    float Q[STATE_SIZE][STATE_SIZE];   // Process noise covariance
    float R[MEAS_SIZE][MEAS_SIZE];     // Measurement noise covariance
    float dt;                           // Time step
} NBKF_t;

// Position and velocity output structure
typedef struct {
    double latitude;
    double longitude;
    float altitude;
    float vx;
    float vy;
    float vz;
} NBKF_Output_t;

// IMU data structure
typedef struct {
    float ax, ay, az;      // Accelerometer (m/s^2)
    float gx, gy, gz;      // Gyroscope (rad/s)
    float mx, my, mz;      // Magnetometer (uT)
} IMU_Data_t;

// GPS data structure
typedef struct {
    double latitude;
    double longitude;
    float altitude;
    uint8_t valid;
} GPS_Data_t;

// Function prototypes
void NBKF_Init(NBKF_t *kf, float dt);
void NBKF_Predict(NBKF_t *kf, IMU_Data_t *imu);
void NBKF_Update(NBKF_t *kf, GPS_Data_t *gps, IMU_Data_t *imu);
void NBKF_GetOutput(NBKF_t *kf, NBKF_Output_t *output);
void NBKF_SetProcessNoise(NBKF_t *kf, float pos_noise, float vel_noise, float att_noise);
void NBKF_SetMeasurementNoise(NBKF_t *kf, float gps_noise, float imu_noise);

#endif // NBKF_H