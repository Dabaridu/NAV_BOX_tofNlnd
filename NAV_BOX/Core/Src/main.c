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
//#include "string.h"
//#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include <stdio.h>
#include <string.h>


//include netbuilt librarys
//#include "nmea_parse.h"
#include "bno055.h"
#include "bmp180_for_stm32_hal.h"
#include "ubx_nav_sol.h"
#include "nslp_dma.h"
#include "micros.h"

//homebrewed libraries
#include "nslp_packets.h"
#include "sens_fusion.h"
#include "iir_filter.h"
#include "sd_black_box.h"

//#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define SD_SPI_HANDLE hspi1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
//-------------------------DEBUG--------------------
uint8_t d_checker;


//-------------------------------BMP180---------------------------------------------
float temperature;
int32_t pressure;
uint32_t BMP180_time;
#define BMP180_interval 1000 // 1 second interval in milliseconds
bool to_send_BMP180_nslp_flag = false;
//-------------------------------BMP180---------------------------------------------

//-------------------------------BNO055---------------------------------------------
bno055_t bno;

bno055_vec3_t accel;
bno055_vec3_t filtered_accel;
bno055_vec4_t quat;
bno055_euler_t euler;

int32_t BNO055_timeStamp;

bool to_send_imu_nslp_flag = false;
//-------------------------------BNO055---------------------------------------------
//--------------------------FILTER---------------------------------
IIR_LPF_1st accel_x_filter;
IIR_LPF_1st accel_y_filter;
IIR_LPF_1st accel_z_filter;
//--------------------------FILTER---------------------------------

//-------------------------------GPS---------------------------------------------
#define rx_buffer_size 60 //GPS uart 1 recieve NAV-SOL packed size
uint8_t rx_buffer[rx_buffer_size]; //size of data recieved from GPS
UBX_NavSol rawData; //raw gps data from parsing the rx_buffer
UBX_NavSolData solData; // organized and converted gps data
uint32_t GPS_timeStamp; //timestamp of processing

bool gps_dma_data_ready = false;
bool to_send_gps_nslp_flag = false;
//-------------------------------GPS---------------------------------------------

//--------------------------NSLP--------------------------------------------
/*
 * Match this to recieving end of NSLP debugger
 * Software: -->NSLP Trace<--
 *
 *It this struct define the data you want to send from the STM machine
 */
nslp_instance_t nslp;
//uint8_t tx_buffer[sizeof(struct Position)]; //transmission size of message
//--------------------------NSLP--------------------------------------------

//---------------------------SD CARD----------------------------
char log_filename[64] = "data_log"; // Default log filename
char sd_gps_buffer[128]; // Buffer for GPS data to be written to SD card
//%G,Time,Timestamp,FixType,NumSV,Lat,Lon,Alt
bool sd_gps_store_flag = false;
char sd_bmp_buffer[128]; // Buffer for BMP180 data to be written to SD card
//%B,Temp,Press,Timestamp
bool sd_bmp_store_flag = false;
char sd_bno_buffer[128]; // Buffer for BNO055 data to be written to SD cardy
//%I,Ax,Ay,Az,Gx,Gy,Gz,Timestamp
bool sd_bno_store_flag = false;
//---------------------------SD CARD----------------------------


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

//----------------------------------INTERUPTS FUNCTIONS---------------------------------------------
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
    	BNO055_timeStamp = HAL_GetTick();
        bno.dma_data_ready = true;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        // I2C error occurred, restart the DMA read
        bno.dma_data_ready = false;
        // Optionally, you can add a counter here to track errors
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	nslp_uart_error_handler(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

  //gps uart handler
	if (huart->Instance == USART1)//Check which interupt was triggered for / on uart1 line connected to GPS
	{
		GPS_timeStamp = HAL_GetTick();
    /*
      check if are still parsing last packet
      only clear flag to reparse if last packet had been parsed
    */
    if (gps_dma_data_ready == false) {
      gps_dma_data_ready = true;

      //if LAST package had not been parsed yet, this is an ERROR !!!
    }

		/* restart the DMA  */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) rx_buffer, sizeof(rx_buffer)); //Restart interupt
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); //Dont interupt on half buffer
	}

  //seperate nslp handler
	nslp_uart_rx_event_handler(huart, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	nslp_uart_tx_cplt_handler(huart);
}

//------------------------------USER FUNCTIONS------------------------------

void parse_bno055_data(){
//On I2C dma memCplt set set dma_data_ready flag to parse data from register and restart recieve from BNI055

	if (bno.dma_data_ready) {

		bno055_get_linear_acc(&bno, &accel);

    filtered_accel.x = IIR_LPF_1st_Apply(&accel_x_filter, accel.x);
    filtered_accel.y = IIR_LPF_1st_Apply(&accel_y_filter, accel.y);
    filtered_accel.z = IIR_LPF_1st_Apply(&accel_z_filter, accel.z);
		
		bno055_get_euler(&bno, &euler);
		bno055_get_quaternion(&bno, &quat); //orientation

		//block dma_data_ready untill I2C rxCpltCallback is executed
		//and send dma_read_instruction to BNO055 returns I2C error in the HAL is buissy or read incorectly
		bno055_start_dma_read(&bno);

		to_send_imu_nslp_flag = true;

    sd_bno_store_flag = true; // Set flag to store BNO055 data to SD card
	}
}

void poll_n_parse_bmp180_data(){
//reads pressure and temperature from BMP180 sensor every BMP180_interval
  static uint32_t lastBMP180Time = 0;
  

  if (HAL_GetTick() - lastBMP180Time > BMP180_interval){

      temperature = BMP180_GetRawTemperature();
      pressure = BMP180_GetPressure();

      lastBMP180Time = HAL_GetTick();
      BMP180_time = lastBMP180Time;

      to_send_BMP180_nslp_flag = true;
      sd_bmp_store_flag = true; // Set flag to store BMP180 data to SD card
  }
}

void parse_gps_data(){
/*
  Parses gps data after recieved_GPS_flag flag is cleared
  parsed gps data is stored in -> solData <-
*/

	if(gps_dma_data_ready == true){

		//parse check rx_buffer to usefull navigation data
      uint8_t parsecheck = UBX_ParseNavSolFrame(rx_buffer, sizeof(rx_buffer), &rawData, &solData);

		//----------------------FLAGS----------------------
      //clear flag for recieving new GPS data
      gps_dma_data_ready = false;
      sd_gps_store_flag = true; // Set flag to store GPS data to SD card
      to_send_gps_nslp_flag = true; //set flag to send gps data via NSLP

		// if(GPS_nslp_data.fixType != 0){ // Only consider valid GPS fixes for fusion
		// 	return;
		// }
	}
}

void nslp_transmit_packets(){
/*
  transmits NSLP packets 
  using nslp_min_delay for optimization of transmition time
*/

  //pass the parsed data to Position pos packet frame
  GPS_nslp_data.latitude = solData.latitude_deg;
  GPS_nslp_data.longitude = solData.longitude_deg;
  GPS_nslp_data.altitude = solData.height_m;
  GPS_nslp_data.fixType = solData.gpsFix;
  GPS_nslp_data.numSV = solData.numSV;
  GPS_nslp_data.time = GPS_timeStamp;

// Copy filtered accelerometer data to transmission structure
  BNO055_nslp_data_a.ax = filtered_accel.x;
  BNO055_nslp_data_a.ay = filtered_accel.y;
  BNO055_nslp_data_a.az = filtered_accel.z;
  BNO055_nslp_data_a.time = BNO055_timeStamp;

  BNO055_nslp_data_g.gx = euler.pitch;
  BNO055_nslp_data_g.gy = euler.roll;
  BNO055_nslp_data_g.gz = euler.yaw;
  BNO055_nslp_data_g.time = BNO055_timeStamp;

  //write data to NSLP payload
  BMP_nslp_data.temp = temperature;
  BMP_nslp_data.press = pressure;
  BMP_nslp_data.time = BMP180_time;

  static uint32_t lastTime = 0;

  //------------------------------NSLP setup for optimal comunication interval-----------------------------------
  if (HAL_GetTick() - lastTime > nslp_min_delay(&nslp)){

    //-----------------------------Send packets for NSLP----------------- START

    if((to_send_gps_nslp_flag == true)){  // Only send GPS packet if new GPS data has been parsed and is ready
      nslp_send_packet(&nslp, &GPS_packet);
      to_send_gps_nslp_flag = false;
    }

        if(to_send_imu_nslp_flag == true){
        nslp_send_packet(&nslp, &BNO_packet_a);
        nslp_send_packet(&nslp, &BNO_packet_g);
        to_send_imu_nslp_flag = false;
    }

    if(to_send_BMP180_nslp_flag == true){
        nslp_send_packet(&nslp, &BMP_packet);
        to_send_BMP180_nslp_flag = false;
    }

    // nslp_send_packet(&nslp, &POS_fusion_packet);
    //-----------------------------Send packets for NSLP----------------- END
    lastTime = HAL_GetTick();
  }
}

void sd_store(){
  uint8_t check = 0;

  if(sd_gps_store_flag == true){
    sd_gps_store_flag = false;
    snprintf(sd_gps_buffer, sizeof(sd_gps_buffer), 
    "G,%d,%d,%d,%f,%f,%f \r\n", 
    GPS_timeStamp, solData.gpsFix, solData.numSV, solData.latitude_deg, solData.longitude_deg, solData.height_m
    );
    //%G,Time,Timestamp,FixType,NumSV,Lat,Lon,Alt
    check = sd_card_write((const uint8_t *)sd_gps_buffer, strlen(sd_gps_buffer));
  }

  if(sd_bno_store_flag == true){
    sd_bno_store_flag = false;
    snprintf(sd_bno_buffer, sizeof(sd_bno_buffer), "I,%f,%f,%f,%f,%f,%f,%d \r\n", filtered_accel.x, filtered_accel.y, filtered_accel.z, euler.pitch, euler.roll, euler.yaw, BNO055_timeStamp);
    //%I,Ax,Ay,Az,Gx,Gy,Gz,Timestamp
    check = sd_card_write((const uint8_t *)sd_bno_buffer, strlen(sd_bno_buffer));
  }

  if(sd_bmp_store_flag == true){
    sd_bmp_store_flag = false;
    snprintf(sd_bmp_buffer, sizeof(sd_bmp_buffer), //%G,Time,Timestamp,FixType,NumSV,Lat,Lon,Alt
    "B,%f,%ld,%d \r\n", 
    temperature, pressure, BMP180_time);
    //%B,Temp,Press,Timestamp
    check = sd_card_write((const uint8_t *)sd_bmp_buffer, strlen(sd_bmp_buffer));
  }

  if(check != 0){
    // Handle error in writing to SD card
    // You can add error handling code here, such as logging the error or retrying the write operation
	  //d_checker = check;
  }
}

void process_sensor_fusion(){
  /*
    Code implementation for Kalman filter data proccesing
  */
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
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  //innitialize library for time measurement in micros
	  DWT_Init(); 

	//------------------------NSLP CODE------------------------
	  nslp_init(&nslp, &huart2, &hcrc, 16, 0xAA);
	//------------------------NSLP CODE------------------------

	//----------------initialize DMA on uart1, rx for GPS-----------
    __HAL_DMA_ENABLE_IT(&hdma_usart1_rx, DMA_IT_TC | DMA_IT_HT);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) rx_buffer, sizeof(rx_buffer));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT); //DMA half transfer interupt

	//-----------Initializes BMP180 sensor and oversampling settings.------------
    BMP180_Init(&hi2c1);
  	BMP180_SetOversampling(BMP180_STANDARD); //BMP180_LOW, BMP180_STANDARD, BMP180_HIGH, BMP180_ULTRA,
	// Update calibration data. Must be called once before entering main loop.
	  BMP180_UpdateCalibrationData();

	//-----------innitialize BNO055 DMA on I2C------------
    bno.i2c = &hi2c1;
    bno.addr = BNO_ADDR_ALT;
    bno.mode = BNO_MODE_NDOF;
    HAL_Delay(500);
    if (bno055_init(&bno) != BNO_OK) {
      uint32_t BNO_timeout = 0;
      BNO_timeout = HAL_GetTick();
      while (HAL_GetTick() - BNO_timeout < 5000) { //wait for 5 seconds before trying to initialize again
        if (bno055_init(&bno) == BNO_OK) {
          break; //break loop if initialization is successful
        }
      }
    }

  //Set BNO055 units and start first DMA read
    bno055_set_unit(&bno, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_DPS, BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG);
    bno055_start_dma_read(&bno);

  //innitialize the filters for the accelerometer data
    float cutoff = 10; // 10Hz cutoff frequency for accelerometer data
    float sample_rate = 100; // 100Hz sample rate for accelerometer data
    IIR_LPF_1st_Init(&accel_x_filter, cutoff, sample_rate);
    IIR_LPF_1st_Init(&accel_y_filter, cutoff, sample_rate);
    IIR_LPF_1st_Init(&accel_z_filter, cutoff, sample_rate);

  //------------------------TEST SD CARD WRITE------------------------
    //test_sd_card_write(&huart2);
    
  //------------------------SD CARD INNIT-----------------------------
    sd_card_init(log_filename);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    
		parse_gps_data();
		parse_bno055_data();
		poll_n_parse_bmp180_data();
		nslp_transmit_packets();
		sd_store();

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

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

#ifdef  USE_FULL_ASSERT
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
