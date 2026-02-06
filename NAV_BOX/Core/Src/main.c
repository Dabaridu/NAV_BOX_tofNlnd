/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdbool.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

//include the library
//#include "nmea_parse.h"

#include "bno055_stm32.h"
#include "bmp180_for_stm32_hal.h"
#include "ubx_nav_sol.h"
#include "nslp_dma.h"

#include "ekfNavINS.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define type_GPS 0x0a 		//10
#define type_BNO055 0x0b	//11
#define type_BMP 0x0c		//12

#define rx_buffer_size 60 //GPS uart 1 recieve NAV-SOL packed size

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

float temperature;
int32_t pressure;
bno055_vector_t v;
bno055_vector_t ac;
bno055_vector_t mag;
bno055_calibration_state_t cal;

//dirty flags
bool recieved_GPS = false;
bool parsed_GPS = false;

//---------------REVIEVING PACKET FROM GPS VIA UBX raw bytes---------------
/*
 *
 * Match this to recieving end of NSLP debugger
 * Software: -->NSLP Trace<--
 *
 *It this struct define the data you want to send from the STM machine
 *It
 * */
struct Position {
	float latitude;
	float longitude;
	float altitude;
	uint32_t fixType;
	uint32_t numSV;

	uint32_t time;
};

struct Position pos = {
		.latitude = 1.0,
		.longitude = 0.0,
		.altitude = 0.0,
		.fixType = 0,
		.numSV = 0
};

//structure of BNO055 payload (variables of type DOUBLE)
struct BNO055_payload {

	//uint32_t calib; //8 --> 32 bit

	double vx;
	double vy;
	double vz;

	double ax;
	double ay;
	double az;

	double mx;
	double my;
	double mz;

	uint32_t time;
};
struct BNO055_payload BNO055_data;

struct BMP_payload {
	float temp;
	uint32_t press;

	uint32_t time;
};

struct BMP_payload BMP_data;


//----------------------------------------------------------------------------
UBX_NavSol rawData;
UBX_NavSolData solData;

uint8_t rx_buffer[rx_buffer_size]; //size of data recieved from GPS
uint8_t tx_buffer[sizeof(struct Position)]; //recieving size of DMA message

nslp_instance_t nslp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	nslp_uart_error_handler(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

	if (huart->Instance == USART1)//Check which interupt was triggered
	{
		recieved_GPS = true;

		/* restart the DMA  */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) rx_buffer, sizeof(rx_buffer)); //Restart interupt
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); //Dont interupt on half buffer


	}

	nslp_uart_rx_event_handler(huart, Size);

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	nslp_uart_tx_cplt_handler(huart);
}

void ekfNavINS::ekf_init(uint64_t time, double vn,double ve,double vd,double lat,double lon,double alt,float p,float q,float r,float ax,float ay,float az,float hx,float hy, float hz) {
  // grab initial gyro values for biases
  gbx = p;
  gby = q;
  gbz = r;
  std::tie(theta,phi,psi) = getPitchRollYaw(ax, ay, az, hx, hy, hz);
  // euler to quaternion
  quat = toQuaternion(phi, theta, psi);
  // Assemble the matrices
  // ... gravity
  grav(2,0) = G;
  // ... H
  H.block(0,0,5,5) = Eigen::Matrix<float,5,5>::Identity();
  // ... Rw
  Rw.block(0,0,3,3) = powf(SIG_W_A,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  Rw.block(3,3,3,3) = powf(SIG_W_G,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  Rw.block(6,6,3,3) = 2.0f * powf(SIG_A_D,2.0f) / TAU_A*Eigen::Matrix<float,3,3>::Identity();
  Rw.block(9,9,3,3) = 2.0f * powf(SIG_G_D,2.0f) / TAU_G*Eigen::Matrix<float,3,3>::Identity();
  // ... P
  P.block(0,0,3,3) = powf(P_P_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  P.block(3,3,3,3) = powf(P_V_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  P.block(6,6,2,2) = powf(P_A_INIT,2.0f) * Eigen::Matrix<float,2,2>::Identity();
  P(8,8) = powf(P_HDG_INIT,2.0f);
  P.block(9,9,3,3) = powf(P_AB_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  P.block(12,12,3,3) = powf(P_GB_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  // ... R
  R.block(0,0,2,2) = powf(SIG_GPS_P_NE,2.0f) * Eigen::Matrix<float,2,2>::Identity();
  R(2,2) = powf(SIG_GPS_P_D,2.0f);
  R.block(3,3,2,2) = powf(SIG_GPS_V_NE,2.0f) * Eigen::Matrix<float,2,2>::Identity();
  R(5,5) = powf(SIG_GPS_V_D,2.0f);
  // .. then initialize states with GPS Data
  lat_ins = lat;
  lon_ins = lon;
  alt_ins = alt;
  vn_ins = vn;
  ve_ins = ve;
  vd_ins = vd;
  // specific force
  f_b(0,0) = ax;
  f_b(1,0) = ay;
  f_b(2,0) = az;
  /* initialize the time */
  _tprev = time;
}

void ekfNavINS::ekf_update( uint64_t time/*, unsigned long TOW*/, double vn,double ve,double vd,double lat,double lon,double alt,
                          float p,float q,float r,float ax,float ay,float az,float hx,float hy, float hz ) {
  if (!initialized_) {
    ekf_init(time, vn, ve, vd, lat, lon, alt, p, q, r, ax, ay, az, hx, hy, hz);
    // initialized flag
    initialized_ = true;
  } else {
    // get the change in time
    float _dt = ((float)(time - _tprev)) / 1e6;
    // Update Gyro and Accelerometer biases
    updateBias(ax, ay, az, p, q, r);
    // Update INS values
    updateINS();
    // Attitude Update
    dq(0) = 1.0f;
    dq(1) = 0.5f*om_ib(0,0)*_dt;
    dq(2) = 0.5f*om_ib(1,0)*_dt;
    dq(3) = 0.5f*om_ib(2,0)*_dt;
    quat = qmult(quat,dq);
    quat.normalize();
    // Avoid quaternion flips sign
    if (quat(0) < 0) {
      quat = -1.0f*quat;
    }
    // AHRS Transformations
    C_N2B = quat2dcm(quat);
    C_B2N = C_N2B.transpose();
    // obtain euler angles from quaternion
    std::tie(phi, theta, psi) = toEulerAngles(quat);
    // Velocity Update
    dx = C_B2N*f_b + grav;
    vn_ins += _dt*dx(0,0);
    ve_ins += _dt*dx(1,0);
    vd_ins += _dt*dx(2,0);
    // Position Update
    dxd = llarate(V_ins,lla_ins);
    lat_ins += _dt*dxd(0,0);
    lon_ins += _dt*dxd(1,0);
    alt_ins += _dt*dxd(2,0);
    // Jacobian update
    updateJacobianMatrix();
    // Update process noise and covariance time
    updateProcessNoiseCovarianceTime(_dt);
    // Gps measurement update
    //if ((TOW - previousTOW) > 0) {
    if ((time - _tprev) > 0) {
      //previousTOW = TOW;
      lla_gps(0,0) = lat;
      lla_gps(1,0) = lon;
      lla_gps(2,0) = alt;
      V_gps(0,0) = vn;
      V_gps(1,0) = ve;
      V_gps(2,0) = vd;
      // Update INS values
      updateINS();
      // Create measurement Y
      updateCalculatedVsPredicted();
      // Kalman gain
      K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
      // Covariance update
      P = (Eigen::Matrix<float,15,15>::Identity()-K*H)*P*(Eigen::Matrix<float,15,15>::Identity()-K*H).transpose() + K*R*K.transpose();
      // State update
      x = K*y;
      // Update the results
      update15statesAfterKF();
      _tprev = time;
    }
    // Get the new Specific forces and Rotation Rate
    updateBias(ax, ay, az, p, q, r);
  }
}

void ekfNavINS::ekf_update(uint64_t time) {
  std::shared_lock lock(shMutex);
  ekf_update(time, /*0,*/ pGpsVelDat->vN, pGpsVelDat->vE, pGpsVelDat->vD,
                      pGpsPosDat->lat, pGpsPosDat->lon, pGpsPosDat->alt,
                      pImuDat->gyroX, pImuDat->gyroY, pImuDat->gyroZ,
                      pImuDat->acclX, pImuDat->acclY, pImuDat->acclZ,
                      pMagDat->hX, pMagDat->hY, pMagDat->hZ);
}

bool ekfNavINS::imuDataUpdateEKF(const imuDataPtr imu, ekfState* ekfOut) {
  {
    std::unique_lock lock(shMutex);
    pImuDat = imu;
    is_imu_initialized = true;
  }
  if (is_gps_pos_initialized && is_gps_vel_initialized && is_mag_initialized && is_imu_initialized) {
    ekf_update(pImuDat->imu_time);
    //
    // Update ekf output
    //
    ekfOut->timestamp = pImuDat->imu_time;
    ekfOut->lla = Eigen::Vector3d(getLatitude_rad(), getLongitude_rad(), getAltitude_m());
    ekfOut->velNED = Eigen::Vector3d(getVelNorth_ms(), getVelEast_ms(), getVelDown_ms());
    ekfOut->linear = f_b;
    ekfOut->angular = om_ib;
    ekfOut->quat = toQuaternion(getHeading_rad(), getPitch_rad(), getRoll_rad());
    ekfOut->cov = P;
    ekfOut->accl_bias = Eigen::Vector3d(getAccelBiasX_mss(), getAccelBiasY_mss(), getAccelBiasZ_mss());
    ekfOut->gyro_bias = Eigen::Vector3d(getGyroBiasX_rads(), getGyroBiasY_rads(), getGyroBiasZ_rads());
    return true;
  } else {
    return false;
  }
}

void ekfNavINS::magDataUpdateEKF(const magDataPtr mag) {
  std::unique_lock lock(shMutex);
  pMagDat = mag;
  is_mag_initialized = true;
}

void ekfNavINS::gpsPosDataUpdateEKF(const gpsPosDataPtr pos) {
  std::unique_lock lock(shMutex);
  pGpsPosDat = pos;
  is_gps_pos_initialized = true;
}

void ekfNavINS::gpsVelDataUpdateEKF(const gpsVelDataPtr vel) {
  std::unique_lock lock(shMutex);
  pGpsVelDat = vel;
  is_gps_vel_initialized = true;
}

void ekfNavINS::updateINS() {
  // Update lat, lng, alt, velocity INS values to matrix
  lla_ins(0,0) = lat_ins;
  lla_ins(1,0) = lon_ins;
  lla_ins(2,0) = alt_ins;
  V_ins(0,0) = vn_ins;
  V_ins(1,0) = ve_ins;
  V_ins(2,0) = vd_ins;
}

std::tuple<float,float,float> ekfNavINS::getPitchRollYaw(float ax, float ay, float az, float hx, float hy, float hz) {
  // initial attitude and heading
  theta = asinf(ax/G);
  phi = -asinf(ay/(G*cosf(theta)));
  // magnetic heading correction due to roll and pitch angle
  Bxc = hx*cosf(theta) + (hy*sinf(phi) + hz*cosf(phi))*sinf(theta);
  Byc = hy*cosf(phi) - hz*sinf(phi);
  // finding initial heading
  psi = -atan2f(Byc,Bxc);
  return (std::make_tuple(theta,phi,psi));
}

void ekfNavINS::updateCalculatedVsPredicted() {
  // Position, converted to NED
  pos_ecef_ins = lla2ecef(lla_ins);
  pos_ecef_gps = lla2ecef(lla_gps);
  pos_ned_gps = ecef2ned(pos_ecef_gps - pos_ecef_ins, lla_ins);
  // Update the difference between calculated and predicted
  y(0,0) = (float)(pos_ned_gps(0,0));
  y(1,0) = (float)(pos_ned_gps(1,0));
  y(2,0) = (float)(pos_ned_gps(2,0));
  y(3,0) = (float)(V_gps(0,0) - V_ins(0,0));
  y(4,0) = (float)(V_gps(1,0) - V_ins(1,0));
  y(5,0) = (float)(V_gps(2,0) - V_ins(2,0));
}

void ekfNavINS::update15statesAfterKF() {
  estmimated_ins = llarate ((x.block(0,0,3,1)).cast<double>(), lat_ins, alt_ins);
  lat_ins += estmimated_ins(0,0);
  lon_ins += estmimated_ins(1,0);
  alt_ins += estmimated_ins(2,0);
  vn_ins = vn_ins + x(3,0);
  ve_ins = ve_ins + x(4,0);
  vd_ins = vd_ins + x(5,0);
  // Attitude correction
  dq(0,0) = 1.0f;
  dq(1,0) = x(6,0);
  dq(2,0) = x(7,0);
  dq(3,0) = x(8,0);
  quat = qmult(quat,dq);
  quat.normalize();
  // obtain euler angles from quaternion
  std::tie(phi, theta, psi) = toEulerAngles(quat);
  abx = abx + x(9,0);
  aby = aby + x(10,0);
  abz = abz + x(11,0);
  gbx = gbx + x(12,0);
  gby = gby + x(13,0);
  gbz = gbz + x(14,0);
}

void ekfNavINS::updateBias(float ax,float ay,float az,float p,float q, float r) {
  f_b(0,0) = ax - abx;
  f_b(1,0) = ay - aby;
  f_b(2,0) = az - abz;
  om_ib(0,0) = p - gbx;
  om_ib(1,0) = q - gby;
  om_ib(2,0) = r - gbz;
}

void ekfNavINS::updateProcessNoiseCovarianceTime(float _dt) {
  PHI = Eigen::Matrix<float,15,15>::Identity()+Fs*_dt;
  // Process Noise
  Gs.setZero();
  Gs.block(3,0,3,3) = -C_B2N;
  Gs.block(6,3,3,3) = -0.5f*Eigen::Matrix<float,3,3>::Identity();
  Gs.block(9,6,6,6) = Eigen::Matrix<float,6,6>::Identity();
  // Discrete Process Noise
  Q = PHI*_dt*Gs*Rw*Gs.transpose();
  Q = 0.5f*(Q+Q.transpose());
  // Covariance Time Update
  P = PHI*P*PHI.transpose()+Q;
  P = 0.5f*(P+P.transpose());
}

void ekfNavINS::updateJacobianMatrix() {
    // Jacobian
  Fs.setZero();
  // ... pos2gs
  Fs.block(0,3,3,3) = Eigen::Matrix<float,3,3>::Identity();
  // ... gs2pos
  Fs(5,2) = -2.0f*G/EARTH_RADIUS;
  // ... gs2att
  Fs.block(3,6,3,3) = -2.0f*C_B2N*sk(f_b);
  // ... gs2acc
  Fs.block(3,9,3,3) = -C_B2N;
  // ... att2att
  Fs.block(6,6,3,3) = -sk(om_ib);
  // ... att2gyr
  Fs.block(6,12,3,3) = -0.5f*Eigen::Matrix<float,3,3>::Identity();
  // ... Accel Markov Bias
  Fs.block(9,9,3,3) = -1.0f/TAU_A*Eigen::Matrix<float,3,3>::Identity();
  Fs.block(12,12,3,3) = -1.0f/TAU_G*Eigen::Matrix<float,3,3>::Identity();
}

// This function gives a skew symmetric matrix from a given vector w
Eigen::Matrix<float,3,3> ekfNavINS::sk(Eigen::Matrix<float,3,1> w) {
  Eigen::Matrix<float,3,3> C;
  C(0,0) = 0.0f;    C(0,1) = -w(2,0); C(0,2) = w(1,0);
  C(1,0) = w(2,0);  C(1,1) = 0.0f;    C(1,2) = -w(0,0);
  C(2,0) = -w(1,0); C(2,1) = w(0,0);  C(2,2) = 0.0f;
  return C;
}

constexpr std::pair<double, double> ekfNavINS::earthradius(double lat) {
  double denom = fabs(1.0 - (ECC2 * pow(sin(lat),2.0)));
  double Rew = EARTH_RADIUS / sqrt(denom);
  double Rns = EARTH_RADIUS * (1.0-ECC2) / (denom*sqrt(denom));
  return (std::make_pair(Rew, Rns));
}

// This function calculates the rate of change of latitude, longitude, and altitude.
Eigen::Matrix<double,3,1> ekfNavINS::llarate(Eigen::Matrix<double,3,1> V,Eigen::Matrix<double,3,1> lla) {
  double Rew, Rns, denom;
  Eigen::Matrix<double,3,1> lla_dot;
  std::tie(Rew, Rns) = earthradius(lla(0,0));
  lla_dot(0,0) = V(0,0)/(Rns + lla(2,0));
  lla_dot(1,0) = V(1,0)/((Rew + lla(2,0))*cos(lla(0,0)));
  lla_dot(2,0) = -V(2,0);
  return lla_dot;
}

// This function calculates the rate of change of latitude, longitude, and altitude.
Eigen::Matrix<double,3,1> ekfNavINS::llarate(Eigen::Matrix<double,3,1> V, double lat, double alt) {
  Eigen::Matrix<double,3,1> lla;
  lla(0,0) = lat;
  lla(1,0) = 0.0; // Not used
  lla(2,0) = alt;
  return llarate(V, lla);
}

// This function calculates the ECEF Coordinate given the Latitude, Longitude and Altitude.
Eigen::Matrix<double,3,1> ekfNavINS::lla2ecef(Eigen::Matrix<double,3,1> lla) {
  double Rew, denom;
  Eigen::Matrix<double,3,1> ecef;
  std::tie(Rew, std::ignore) = earthradius(lla(0,0));
  ecef(0,0) = (Rew + lla(2,0)) * cos(lla(0,0)) * cos(lla(1,0));
  ecef(1,0) = (Rew + lla(2,0)) * cos(lla(0,0)) * sin(lla(1,0));
  ecef(2,0) = (Rew * (1.0 - ECC2) + lla(2,0)) * sin(lla(0,0));
  return ecef;
}

// This function converts a vector in ecef to ned coordinate centered at pos_ref.
Eigen::Matrix<double,3,1> ekfNavINS::ecef2ned(Eigen::Matrix<double,3,1> ecef,Eigen::Matrix<double,3,1> pos_ref) {
  Eigen::Matrix<double,3,1> ned;
  ned(1,0)=-sin(pos_ref(1,0))*ecef(0,0) + cos(pos_ref(1,0))*ecef(1,0);
  ned(0,0)=-sin(pos_ref(0,0))*cos(pos_ref(1,0))*ecef(0,0)-sin(pos_ref(0,0))*sin(pos_ref(1,0))*ecef(1,0)+cos(pos_ref(0,0))*ecef(2,0);
  ned(2,0)=-cos(pos_ref(0,0))*cos(pos_ref(1,0))*ecef(0,0)-cos(pos_ref(0,0))*sin(pos_ref(1,0))*ecef(1,0)-sin(pos_ref(0,0))*ecef(2,0);
  return ned;
}

// quaternion to dcm
Eigen::Matrix<float,3,3> ekfNavINS::quat2dcm(Eigen::Matrix<float,4,1> q) {
  Eigen::Matrix<float,3,3> C_N2B;
  C_N2B(0,0) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(1,0),2.0f);
  C_N2B(1,1) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(2,0),2.0f);
  C_N2B(2,2) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(3,0),2.0f);

  C_N2B(0,1) = 2.0f*q(1,0)*q(2,0) + 2.0f*q(0,0)*q(3,0);
  C_N2B(0,2) = 2.0f*q(1,0)*q(3,0) - 2.0f*q(0,0)*q(2,0);

  C_N2B(1,0) = 2.0f*q(1,0)*q(2,0) - 2.0f*q(0,0)*q(3,0);
  C_N2B(1,2) = 2.0f*q(2,0)*q(3,0) + 2.0f*q(0,0)*q(1,0);

  C_N2B(2,0) = 2.0f*q(1,0)*q(3,0) + 2.0f*q(0,0)*q(2,0);
  C_N2B(2,1) = 2.0f*q(2,0)*q(3,0) - 2.0f*q(0,0)*q(1,0);
  return C_N2B;
}

// quaternion multiplication
Eigen::Matrix<float,4,1> ekfNavINS::qmult(Eigen::Matrix<float,4,1> p, Eigen::Matrix<float,4,1> q) {
  Eigen::Matrix<float,4,1> r;
  r(0,0) = p(0,0)*q(0,0) - (p(1,0)*q(1,0) + p(2,0)*q(2,0) + p(3,0)*q(3,0));
  r(1,0) = p(0,0)*q(1,0) + q(0,0)*p(1,0) + p(2,0)*q(3,0) - p(3,0)*q(2,0);
  r(2,0) = p(0,0)*q(2,0) + q(0,0)*p(2,0) + p(3,0)*q(1,0) - p(1,0)*q(3,0);
  r(3,0) = p(0,0)*q(3,0) + q(0,0)*p(3,0) + p(1,0)*q(2,0) - p(2,0)*q(1,0);
  return r;
}

// bound angle between -180 and 180
float ekfNavINS::constrainAngle180(float dta) {
  if(dta >  M_PI) dta -= (M_PI*2.0f);
  if(dta < -M_PI) dta += (M_PI*2.0f);
  return dta;
}

// bound angle between 0 and 360
float ekfNavINS::constrainAngle360(float dta){
  dta = fmod(dta,2.0f*M_PI);
  if (dta < 0)
    dta += 2.0f*M_PI;
  return dta;
}

Eigen::Matrix<float,4,1> ekfNavINS::toQuaternion(float yaw, float pitch, float roll) {
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    Eigen::Matrix<float,4,1> q;
    q(0) = cr * cp * cy + sr * sp * sy; // w
    q(1) = cr * cp * sy - sr * sp * cy; // x
    q(2) = cr * sp * cy + sr * cp * sy; // y
    q(3) = sr * cp * cy - cr * sp * sy; // z
    return q;
}

std::tuple<float, float, float> ekfNavINS::toEulerAngles(Eigen::Matrix<float,4,1> quat) {
    float roll, pitch, yaw;
    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (quat(0,0)*quat(1,0)+quat(2,0)*quat(3,0));
    float cosr_cosp = 1.0f - 2.0f * (quat(1,0)*quat(1,0)+quat(2,0)*quat(2,0));
    roll = atan2f(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    double sinp = 2.0f * (quat(0,0)*quat(2,0) - quat(1,0)*quat(3,0));
    //angles.pitch = asinf(-2.0f*(quat(1,0)*quat(3,0)-quat(0,0)*quat(2,0)));
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2.0f, sinp); // use 90 degrees if out of range
    else
        pitch = asinf(sinp);
    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (quat(1,0)*quat(2,0)+quat(0,0)*quat(3,0));
    float cosy_cosp = 1.0f - 2.0f * (quat(2,0)*quat(2,0)+quat(3,0)*quat(3,0));
    yaw = atan2f(siny_cosp, cosy_cosp);
    return std::make_tuple(roll, pitch, yaw);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_CRC_Init();
	/* USER CODE BEGIN 2 */
	//------------------------NSLP CODE------------------------
	nslp_init(&nslp, &huart2, &hcrc, 16, 0xAA);
	uint32_t lastTime = 0;
	//initialize a Position struct as pos


	//format and fill the nslp_packet for transmition UART2
	nslp_packet_t pos_packet = {
			.receiver = 0xFF,
			.type = type_GPS,
			.size = sizeof(struct Position),
			.payload = (uint8_t*)&pos
	};

	nslp_packet_t BNO_packet = {
			.receiver = 0xFF,
			.type = type_BNO055,
			.size = sizeof(struct BNO055_payload),
			.payload = (uint8_t*)&BNO055_data
	};

	nslp_packet_t BMP_packet = {
			.receiver = 0xFF,
			.type = type_BMP,
			.size = sizeof(struct BMP_payload),
			.payload = (uint8_t*)&BMP_data
	};
	//------------------------NSLP CODE------------------------

	__HAL_DMA_ENABLE_IT(&hdma_usart1_rx, DMA_IT_TC | DMA_IT_HT);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) rx_buffer, sizeof(rx_buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT); //DMA half transfer interupt


	/* Initializes BMP180 sensor and oversampling settings. */
	BMP180_Init(&hi2c1);
	BMP180_SetOversampling(BMP180_STANDARD); //BMP180_LOW, BMP180_STANDARD, BMP180_HIGH, BMP180_ULTRA,
	/* Update calibration data. Must be called once before entering main loop. */
	BMP180_UpdateCalibrationData();

	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		
		//in this throw all packet sends for NSLP, the function will handle delays
		if (HAL_GetTick() - lastTime > nslp_min_delay(&nslp)){

			if((parsed_GPS == true)/*&&(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))*/){
				//__HAL_UART_CLEAR_IDLEFLAG(&huart1);

				nslp_send_packet(&nslp, &pos_packet);
				parsed_GPS = false;

			}

			nslp_send_packet(&nslp, &BNO_packet);
			nslp_send_packet(&nslp, &BMP_packet);

			lastTime = HAL_GetTick();
		}

//-------------------parse data from GPS------------------------
		if(recieved_GPS == true){

			recieved_GPS = false;

			//parse rx_buffer to usefull navigation data
			uint8_t parsecheck = UBX_ParseNavSolFrame(rx_buffer, sizeof(rx_buffer), &rawData,&solData);

			//pass the parsed data to Position pos packet frame
			pos.latitude = solData.latitude_deg;
			pos.longitude = solData.longitude_deg;
			pos.altitude = solData.height_m;
			pos.fixType = solData.gpsFix;
			pos.numSV = solData.numSV;

			pos.time = HAL_GetTick();

			parsed_GPS = true;
		}
//-------------------parse data from GPS------------------------

//-------------------parse data from I2C------------------------
		/* Reads BNO055 absolute position sensor*/
		//cal = bno055_getCalibrationState();		//calibration state 0-3 how good the estimate is
		v = bno055_getVectorEuler(); 			//absolute position
		ac = bno055_getVectorAccelerometer(); 	//acceleration
		mag = bno055_getVectorMagnetometer(); 	//magnetic vector

		//BNO055_data.calib = cal.sys;

		BNO055_data.vx = v.x;
		BNO055_data.vy = v.y;
		BNO055_data.vz = v.z;

		BNO055_data.ax = ac.x;
		BNO055_data.ay = ac.y;
		BNO055_data.az = ac.z;

		BNO055_data.mx = mag.x;
		BNO055_data.my = mag.y;
		BNO055_data.mz = mag.z;
		
		BNO055_data.time = HAL_GetTick();

		/* Reads temperature. */
		temperature = BMP180_GetRawTemperature();
		/* Reads pressure. */
		pressure = BMP180_GetPressure();

		BMP_data.temp = temperature;
		BMP_data.press = pressure;

		BMP_data.time = HAL_GetTick();
//-------------------parse data from I2C------------------------

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
	hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
	hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
	hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
	hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00201D2B;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
