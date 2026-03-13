#ifndef SENS_FUSION_H
#define SENS_FUSION_H

#include "main.h"

//standart BNO055 quaternion & vector struct
//(for ease of use)
typedef struct {
  double w;
  double x;
  double y;
  double z;
} vector_t;

//IMU data
extern vector_t acc; //LINEAR acceleration
extern vector_t acc_prec;
extern vector_t abs_q; //absolute orientation as quaternion
//GPS data
extern vector_t pos;
extern vector_t pos_prec;

//Fusion data
extern vector_t X_global_translation;
extern vector_t X_global_translation_precision;
extern vector_t O_global_orientation_euller;
extern vector_t O_global_orientation_euller_precision;

//estimation data
extern vector_t X_hat_estimation;
extern vector_t X_hat_precision;
extern vector_t O_hat_estimation;
extern vector_t O_hat_precision;

extern int ts;
extern int lt;

extern double starting_position_offset[3];
extern double starting_orientation_offset[3][3];

//-----------------------------------------------------------------------------
//update as often as posible
void update_IMU_global_position(double starting_position_offset[3],double starting_orientation_offset[3][3],vector_t *acc, vector_t *abs_q, int time_step, vector_t *X_global_translation, vector_t *O_global_orientation_euller);
void update_position_precision_calculation(vector_t *X_global_translation_precision, vector_t *acc_prec,int time_step);
//void update_orientation_precision_calculation(vector_t *abs_q_prec, int time_step);

//update on GPS recieved
void joined_position_estimation(vector_t *X_hat_estimation, vector_t *pos, vector_t *imu_X, vector_t *pos_prec, vector_t *acc_prec);
void joined_precision_calculation(vector_t *X_hat_precision, vector_t *pos_prec, vector_t *acc_prec);
void reset_IMU_precision(vector_t *X_hat_precision, vector_t *X_global_translation_precision, vector_t *X_hat_estimation, vector_t *X_global_translation);

//support functions
double update_ts(int *last_time, int current_time);

#endif /* SENS_FUSION_H */
