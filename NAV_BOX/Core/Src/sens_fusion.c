#include "sens_fusion.h"

//complie size aprox 4KB

//DEBUGGING VARIABLES
extern double Hd[4][4];

//IMU data
vector_t acc;
vector_t acc_prec = {1e-5, 1e-5, 1e-5, 1e-5};
vector_t abs_q;
vector_t abs_q_prec;
//GPS data
vector_t pos;
vector_t pos_prec = {1e-3, 1e-3, 1e-3, 1e-3};

//imu integration data
vector_t X_global_translation;
vector_t X_global_translation_precision;
vector_t O_global_orientation_euller;
vector_t O_global_orientation_euller_precision;

//Fusion data
vector_t X_hat_estimation = {0,0,0,0};
vector_t X_hat_precision = {0,0,0,0};
vector_t O_hat_estimation = {0,0,0,0};
vector_t O_hat_precision = {0,0,0,0};

int ts = 0;
int lt = 0;

double starting_position_offset[3] = {0.0, 0.0, 0.0};
double starting_orientation_offset[3][3] = {{1.0, 0.0, 0.0},
											{0.0, 1.0, 0.0},
											{0.0, 0.0, 1.0}};

void low_pass_filter(vector_t *input){
    
}


void matrix_multiply(double A[4][4], double B[4][4], double result[4][4]){
    //result = A * B
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            result[i][j] = 0.0;
            for(int k = 0; k < 4; k++){
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void update_IMU_global_position(double sp[3],double so[3][3],vector_t *acc, vector_t *abs_q, int time_step, vector_t *X_global_translation, vector_t *O_global_orientation_euller){
    double t_s = time_step / 1000000.0; //convert to seconds
    double L[4][4] = {{so[0][0],so[0][1],so[0][2],sp[0]},
                      {so[1][0],so[1][1],so[1][2],sp[1]},
                      {so[2][0],so[2][1],so[2][2],sp[2]},
                      {0.0, 0.0, 0.0, 1.0}};

    //rotation matrix from quaternion (q = [w, x, y, z])
    double R[4][4] = {{(abs_q->w*abs_q->w+abs_q->x*abs_q->x-abs_q->y*abs_q->y-abs_q->z*abs_q->z), 2.0 * (abs_q->x * abs_q->y - abs_q->w * abs_q->z), 2.0 * (abs_q->x * abs_q->z + abs_q->w * abs_q->y), 0.0},
                     {2.0 * (abs_q->x * abs_q->y + abs_q->w * abs_q->z), (abs_q->w*abs_q->w-abs_q->x*abs_q->x+abs_q->y*abs_q->y-abs_q->z*abs_q->z), 2.0 * (abs_q->y * abs_q->z - abs_q->w * abs_q->x), 0.0},
                     {2.0 * (abs_q->x * abs_q->z - abs_q->w * abs_q->y), 2.0 * (abs_q->y * abs_q->z + abs_q->w * abs_q->x), (abs_q->w*abs_q->w-abs_q->x*abs_q->x-abs_q->y*abs_q->y+abs_q->z*abs_q->z), 0.0},
                     {0.0, 0.0, 0.0, 1.0}};
    
    double T[4][4] = {{1.0, 0.0, 0.0,X_global_translation->x + acc->x * t_s * t_s / 2.0},
                     {0.0, 1.0, 0.0,X_global_translation->y + acc->y * t_s * t_s / 2.0},
                     {0.0, 0.0, 1.0,X_global_translation->z + acc->z * t_s * t_s / 2.0},
                     {0.0, 0.0, 0.0, 1.0}};

    double L1[4][4];
    double L2[4][4];

    matrix_multiply(R, L, L1); //global rotation

    matrix_multiply(L1, T, L2); //local translation

    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            Hd[i][j] = L2[i][j]; // debug matrix Hd
        }
    } //for debugging

    X_global_translation->x = L2[0][3];
    X_global_translation->y = L2[1][3];
    X_global_translation->z = L2[2][3];

    // rotation from starting orientation in pitch, yaw, roll (euler angles)
    O_global_orientation_euller->x = atan2f(R[2][1], R[2][2]); // pitch
    O_global_orientation_euller->y = atan2f(-R[2][0], sqrtf(R[2][1] * R[2][1] + R[2][2] * R[2][2])); // yaw
    O_global_orientation_euller->z = atan2f(R[1][0], R[0][0]); // roll

}

void update_position_precision_calculation(vector_t *X_global_translation_precision, vector_t *acc_prec,int time_step){
	double t_s = time_step / 1000000.0; //convert to seconds
    // Simple model: precision degrades over time due to drift
	X_global_translation_precision->x += acc_prec->x * 1/sqrt(3) * sqrt(t_s*t_s*t_s); // Increase variance in x
	X_global_translation_precision->y += acc_prec->y * 1/sqrt(3) * sqrt(t_s*t_s*t_s); // Increase variance in y
	X_global_translation_precision->z += acc_prec->z * 1/sqrt(3) * sqrt(t_s*t_s*t_s); // Increase variance in z
}
void update_orientation_precision_calculation(vector_t *O_global_orientation_euller_precision, vector_t *abs_q_prec,int time_step){
	double t_s = (double) time_step / 1000000.0; //convert to seconds
	// Simple model: precision degrades over time due to drift
	O_global_orientation_euller_precision->x += abs_q_prec->x * 1/sqrt(3) * sqrt(t_s*t_s*t_s); // Increase variance in x
	O_global_orientation_euller_precision->y += abs_q_prec->y * 1/sqrt(3) * sqrt(t_s*t_s*t_s); // Increase variance in y
	O_global_orientation_euller_precision->z += abs_q_prec->z * 1/sqrt(3) * sqrt(t_s*t_s*t_s); // Increase variance in z
}


void joined_position_estimation(vector_t *X_hat_estimation, vector_t *pos, vector_t *imu_X, vector_t *pos_prec, vector_t *acc_prec){
    double gps_weight = 1.0 / (pos_prec->x + pos_prec->y + pos_prec->z);
    double imu_weight = 1.0 / (acc_prec->x + acc_prec->y + acc_prec->z);

    double total_weight = gps_weight + imu_weight;

    double estimated_latitude = (pos->x * gps_weight + imu_X->x * imu_weight) / total_weight;
    double estimated_longitude = (pos->y * gps_weight + imu_X->y * imu_weight) / total_weight;
    double estimated_altitude = (pos->z * gps_weight + imu_X->z * imu_weight) / total_weight;

    X_hat_estimation->x = estimated_latitude;
    X_hat_estimation->y = estimated_longitude;
    X_hat_estimation->z = estimated_altitude;
}

void joined_precision_calculation(vector_t *X_hat_precision, vector_t *pos_prec, vector_t *acc_prec){
    double gps_variance = pos_prec->x + pos_prec->y + pos_prec->z;
    double imu_variance = acc_prec->x + acc_prec->y + acc_prec->z;

    double combined_variance = 1.0 / (1.0 / gps_variance + 1.0 / imu_variance);
    X_hat_precision->x = combined_variance;
    X_hat_precision->y = combined_variance;
    X_hat_precision->z = combined_variance;
}

void reset_IMU_precision(vector_t *X_hat_precision, vector_t *X_global_translation_precision, vector_t *X_hat_estimation, vector_t *X_global_translation){
    // When we get a new GPS measurement, we can reset the IMU precision to the GPS precision
	*X_global_translation_precision = *X_hat_precision;
    *X_global_translation = *X_hat_estimation;
}

//-----------------------------------------------------Support functions------------------------------------------------------------------------------------------------------

double update_ts(int *last_time, int current_time){
    int time_step = current_time - *last_time;
    *last_time = current_time;
    return (double) time_step;
}
