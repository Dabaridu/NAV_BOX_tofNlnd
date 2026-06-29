#include "sens_fusion.h"
/* sens_fusion.c
 * Source file for sensor fusion algorithms and related functions
 */

 /*
    1. Prediction Step (Time Update)
    x_pred = A * x + B * u  // Predict next state
    P_pred = A * P * A^T + Q  // Predict covariance

    2. Update Step (Measurement Update)
    y = z - H * x_pred  // Innovation (measurement residual)
    S = H * P_pred * H^T + R  // Innovation covariance
    K = P_pred * H^T * S^-1  // Kalman gain
    x = x_pred + K * y  // Updated state
    P = (I - K * H) * P_pred  // Updated covariance

    3. Key Parameters to Tune
    -Q (Process Noise)          : How much you trust your system model. Higher = expect more unpredictable changes
    -R (Measurement Noise)      : How much you trust your sensors. Higher = measurements are noisier
    -Initial P (Covariance)     : Initial uncertainty in your state estimate
*/

#include <math.h>

typedef struct {
    float *x;           // State vector (n x 1)
    float *P;           // Covariance matrix (n x n)
    float *A;           // State transition (n x n)
    float *H;           // Measurement matrix (m x n)
    float *Q;           // Process noise (n x n)
    float *R;           // Measurement noise (m x m)
    float *K;           // Kalman gain (n x m)
    int n;              // State dimension
    int m;              // Measurement dimension
} KalmanFilter;





