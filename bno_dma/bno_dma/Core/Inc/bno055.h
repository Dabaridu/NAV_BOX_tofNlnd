/**
 * BNO055 DMA-based driver for STM32F3
 * Adapted for non-blocking continuous reads.
 */
#ifndef DRV_BNO055_H_
#define DRV_BNO055_H_

#include <stm32f3xx.h>
#include "stm32f3xx_hal.h"
#include "stdbool.h"

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t s8;
typedef int16_t s16;
typedef float f32;

// I2C Addresses
#define BNO_ADDR           (0x29)
#define BNO_ADDR_ALT       (0x28)
#define BNO_DEF_CHIP_ID    (0xA0)

// Page 0 Registers
#define BNO_CHIP_ID        (0x00)
#define BNO_PAGE_ID        (0x07)
#define BNO_ACC_DATA_X_LSB (0x08)
#define BNO_TEMP           (0x34)
#define BNO_CALIB_STAT     (0x35)
#define BNO_SYS_TRIGGER    (0x3F)
#define BNO_PWR_MODE       (0x3E)
#define BNO_OPR_MODE       (0x3D)
#define BNO_UNIT_SEL       (0x3B)

// Page 1 Registers
#define BNO_ACC_CONFIG       (0x08)
#define BNO_MAG_CONFIG       (0x09)
#define BNO_GYR_CONFIG_0     (0x0A)
#define BNO_GYR_CONFIG_1     (0x0B)

// Offsets & Scales
#define BNO_ACC_BAND_OFFSET     (0x02)
#define BNO_ACC_MODE_OFFSET     (0x05)
#define BNO_GYR_BAND_OFFSET     (0x03)
#define BNO_GYR_MODE_OFFSET     (0x00)
#define BNO_MAG_MODE_OFFSET     (0x03)
#define BNO_MAG_PWRMODE_OFFSET  (0x05)
#define BNO_GYR_UNITSEL_OFFSET  (0x01)
#define BNO_EUL_UNITSEL_OFFSET  (0x02)
#define BNO_TEMP_UNITSEL_OFFSET (0x04)

#define BNO_ACC_SCALE_M_2 100.0f
#define BNO_ACC_SCALE_MG  1.0f
#define BNO_GYR_SCALE_DPS 16.0f
#define BNO_GYR_SCALE_RPS 900.0f
#define BNO_MAG_SCALE     16.0f
#define BNO_EUL_SCALE_DEG 16.0f
#define BNO_EUL_SCALE_RAD 900.0f
#define BNO_QUA_SCALE     (0x4000)

// Delays
#define BNO_CONFIG_TIME_DELAY 7
#define BNO_ANY_TIME_DELAY    19

// Enums
typedef enum { BNO_PAGE_0 = 0x00U, BNO_PAGE_1 = 0x01U } bno055_page_t;
typedef enum { BNO_PWR_NORMAL, BNO_PWR_LOW, BNO_PWR_SUSPEND } bno055_pwr_t;
typedef enum { BNO_ACC_RANGE_2G, BNO_ACC_RANGE_4G, BNO_ACC_RANGE_8G, BNO_ACC_RANGE_16G } bno055_acc_range_t;
typedef enum { BNO_ACC_BAND_7_81 = (0<<2), BNO_ACC_BAND_15_63 = (1<<2), BNO_ACC_BAND_31_25 = (2<<2), BNO_ACC_BAND_62_5 = (3<<2), BNO_ACC_BAND_125 = (4<<2), BNO_ACC_BAND_250 = (5<<2), BNO_ACC_BAND_500 = (6<<2), BNO_ACC_BAND_1000 = (7<<2) } bno055_acc_band_t;
typedef enum { BNO_ACC_MODE_NORMAL = (0<<5), BNO_ACC_MODE_SUSPEND = (1<<5), BNO_ACC_MODE_LOW1 = (2<<5), BNO_ACC_MODE_STANDY = (3<<5), BNO_ACC_MODE_LOW2 = (4<<5), BNO_ACC_MODE_DEEP_SUSPEND = (5<<5) } bno055_acc_mode_t;
typedef enum { BNO_GYR_RANGE_2000_DPS, BNO_GYR_RANGE_1000_DPS, BNO_GYR_RANGE_500_DPS, BNO_GYR_RANGE_250_DPS, BNO_GYR_RANGE_125_DPS } bno055_gyr_range_t;
typedef enum { BNO_GYR_BAND_523 = (0<<3), BNO_GYR_BAND_230 = (1<<3), BNO_GYR_BAND_116=(2<<3), BNO_GYR_BAND_47=(3<<3), BNO_GYR_BAND_23=(4<<3), BNO_GYR_BAND_12=(5<<3), BNO_GYR_BAND_64=(6<<3), BNO_GYR_BAND_32=(7<<3) } bno055_gyr_band_t;
typedef enum { BNO_GYR_MODE_NORMAL, BNO_GYR_MODE_FPU, BNO_GYR_MODE_DEEP_SUSPEND, BNO_GYR_MODE_SUSPEND, BNO_GYR_MODE_APS } bno055_gyr_mode_t;
typedef enum { BNO_MAG_RATE_2HZ, BNO_MAG_RATE_6HZ, BNO_MAG_RATE_8HZ, BNO_MAG_RATE_10HZ, BNO_MAG_RATE_15HZ, BNO_MAG_RATE_20HZ, BNO_MAG_RATE_25HZ, BNO_MAG_RATE_30HZ } bno055_mag_rate_t;
typedef enum { BNO_MAG_MODE_LOW_PWR = (0<<3), BNO_MAG_MODE_REGULAR = (1<<3), BNO_MAG_MODE_ENH_REG=(2<<3), BNO_MAG_MODE_HIGH_ACC=(3<<3) } bno055_mag_mode_t;
typedef enum { BNO_MAG_PWRMODE_NORMAL = (0<<5), BNO_MAG_PWRMODE_SLEEP = (1<<5), BNO_MAG_PWRMODE_SUSPEND=(2<<5), BNO_MAG_PWRMODE_FORCE=(3<<5) } bno055_mag_pwr_t;
typedef enum { BNO_ACC_UNITSEL_M_S2, BNO_ACC_UNITSEL_MG } bno055_acc_unitsel_t;
typedef enum { BNO_GYR_UNIT_DPS = (0<<1), BNO_GYR_UNIT_RPS = (1<<1) } bno055_gyr_unitsel_t;
typedef enum { BNO_EUL_UNIT_DEG = (0<<2), BNO_EUL_UNIT_RAD = (1<<2) } bno055_eul_unitsel_t;
typedef enum { BNO_TEMP_UNIT_C = (0<<4), BNO_TEMP_UNIT_F = (1<<4) } bno055_temp_unitsel_t;
typedef enum { BNO_MODE_CONFIG, BNO_MODE_AO, BNO_MODE_MO, BNO_MODE_GO, BNO_MODE_AM, BNO_MODE_AG, BNO_MODE_MG, BNO_MODE_AMG, BNO_MODE_IMU, BNO_MODE_COMPASS, BNO_MODE_M4G, BNO_MODE_NDOF_FMC_OFF, BNO_MODE_NDOF } bno055_opmode_t;

typedef enum _error_bno {
    BNO_OK, BNO_ERR_I2C, BNO_ERR_PAGE_TOO_HIGH, BNO_ERR_SETTING_PAGE, BNO_ERR_NULL_PTR, BNO_ERR_AXIS_REMAP, BNO_ERR_WRONG_CHIP_ID,
} error_bno;

typedef struct bno055_euler { f32 roll; f32 pitch; f32 yaw; } bno055_euler_t;
typedef struct bno055_vec3 { f32 x; f32 y; f32 z; } bno055_vec3_t;
typedef struct bno055_vec4 { f32 x; f32 y; f32 z; f32 w; } bno055_vec4_t;

// The Main Struct
typedef struct _bno055_t {
    I2C_HandleTypeDef* i2c;
    bno055_opmode_t mode;
    u8 addr;

    bno055_page_t _page;
    bno055_acc_unitsel_t _acc_unit;
    bno055_temp_unitsel_t _temp_unit;
    bno055_gyr_unitsel_t _gyr_unit;
    bno055_eul_unitsel_t _eul_unit;

    // DMA Data Buffer: Contains registers 0x08 through 0x34
    // 0x34 - 0x08 + 1 = 45 bytes needed
    uint8_t rx_buf[45];
    volatile bool dma_data_ready;
} bno055_t;

// Setup & config functions
error_bno bno055_init(bno055_t* imu);
error_bno bno055_reset(bno055_t* imu);
error_bno bno055_set_page(bno055_t* imu, const bno055_page_t page);
error_bno bno055_set_opmode(bno055_t* imu, const bno055_opmode_t mode);
error_bno bno055_set_pwr_mode(bno055_t* imu, bno055_pwr_t pwr_mode);
error_bno bno055_set_unit(bno055_t* bno, const bno055_temp_unitsel_t t_unit, const bno055_gyr_unitsel_t g_unit, const bno055_acc_unitsel_t a_unit, const bno055_eul_unitsel_t e_unit);

// Configuration functions
error_bno bno055_acc_conf(bno055_t* bno, const bno055_acc_range_t range, const bno055_acc_band_t bandwidth, const bno055_acc_mode_t mode);
error_bno bno055_gyr_conf(bno055_t* bno, const bno055_gyr_range_t range, const bno055_gyr_band_t bandwidth, const bno055_gyr_mode_t mode);
error_bno bno055_mag_conf(bno055_t* bno, const bno055_mag_rate_t out_rate, const bno055_mag_pwr_t pwr_mode, const bno055_mag_mode_t mode);

// DMA Read Trigger
error_bno bno055_start_dma_read(bno055_t* imu);

// DMA Buffer Getters (Non-blocking)
void bno055_get_acc(bno055_t* imu, bno055_vec3_t* buf);
void bno055_get_mag(bno055_t* imu, bno055_vec3_t* buf);
void bno055_get_gyro(bno055_t* imu, bno055_vec3_t* buf);
void bno055_get_euler(bno055_t* imu, bno055_euler_t* buf);
void bno055_get_quaternion(bno055_t* imu, bno055_vec4_t* buf);
void bno055_get_linear_acc(bno055_t* imu, bno055_vec3_t* buf);
void bno055_get_gravity(bno055_t* imu, bno055_vec3_t* buf);
void bno055_get_temp(bno055_t* imu, s8* buf);

#endif
