#include "bno055.h"
#include "bno_config.h"

static error_bno bno055_write_reg(bno055_t* imu, u32 addr, u8* buf, u32 size) {
    if (HAL_I2C_Mem_Write(imu->i2c, imu->addr, addr, I2C_MEMADD_SIZE_8BIT, buf, size, HAL_MAX_DELAY) != HAL_OK) {
        return BNO_ERR_I2C;
    }
    return BNO_OK;
}

static error_bno bno055_read_reg(bno055_t* imu, u8 addr, u8* buf, u32 size) {
    if (HAL_I2C_Mem_Read(imu->i2c, imu->addr, addr, I2C_MEMADD_SIZE_8BIT, buf, size, HAL_MAX_DELAY) != HAL_OK) {
        return BNO_ERR_I2C;
    }
    return BNO_OK;
}

error_bno bno055_set_page(bno055_t* imu, const bno055_page_t page) {
    if (imu->_page != page) {
        if (page > 0x01) return BNO_ERR_PAGE_TOO_HIGH;
        if (bno055_write_reg(imu, BNO_PAGE_ID, (u8*)&page, 1) != BNO_OK) return BNO_ERR_I2C;
        imu->_page = page;
        HAL_Delay(2);
    }
    return BNO_OK;
}

error_bno bno055_set_opmode(bno055_t* imu, const bno055_opmode_t mode) {
    bno055_set_page(imu, BNO_PAGE_0);
    if (bno055_write_reg(imu, BNO_OPR_MODE, (u8*)&mode, 1) != BNO_OK) return BNO_ERR_I2C;
    HAL_Delay(BNO_ANY_TIME_DELAY + 5);
    return BNO_OK;
}

error_bno bno055_set_pwr_mode(bno055_t* imu, bno055_pwr_t pwr_mode) {
    bno055_set_opmode(imu, BNO_MODE_CONFIG);
    bno055_set_page(imu, BNO_PAGE_0);
    if (bno055_write_reg(imu, BNO_PWR_MODE, (u8*)&pwr_mode, 1) != BNO_OK) return BNO_ERR_I2C;
    bno055_set_opmode(imu, imu->mode);
    HAL_Delay(2);
    return BNO_OK;
}

error_bno bno055_reset(bno055_t* imu) {
    u8 data = 0x20U;
    return bno055_write_reg(imu, BNO_SYS_TRIGGER, &data, 1);
}

error_bno bno055_init(bno055_t* imu) {
    u8 id = 0;

    imu->addr = (imu->addr << 1);

    if (bno055_read_reg(imu, BNO_CHIP_ID, &id, 1) != BNO_OK) return BNO_ERR_I2C;
    if (id != BNO_DEF_CHIP_ID) return BNO_ERR_WRONG_CHIP_ID;

    bno055_set_opmode(imu, BNO_MODE_CONFIG);
    HAL_Delay(2);
    bno055_reset(imu);
    HAL_Delay(800); // Reset takes time

    bno055_set_pwr_mode(imu, BNO_PWR_NORMAL);
    bno055_set_page(imu, BNO_PAGE_0);

    // Trigger internal clock
    u8 trigger = 0x00U;
    bno055_write_reg(imu, BNO_SYS_TRIGGER, &trigger, 1);

    bno055_set_opmode(imu, imu->mode);

    imu->dma_data_ready = false;
    return BNO_OK;
}

error_bno bno055_set_unit(bno055_t* bno, const bno055_temp_unitsel_t t_unit, const bno055_gyr_unitsel_t g_unit, const bno055_acc_unitsel_t a_unit, const bno055_eul_unitsel_t e_unit) {
    bno055_set_opmode(bno, BNO_MODE_CONFIG);
    bno055_set_page(bno, BNO_PAGE_0);

    u8 data = t_unit | g_unit | a_unit | e_unit;
    if (bno055_write_reg(bno, BNO_UNIT_SEL, &data, 1) != BNO_OK) return BNO_ERR_I2C;

    bno->_gyr_unit = g_unit;
    bno->_acc_unit = a_unit;
    bno->_eul_unit = e_unit;
    bno->_temp_unit = t_unit;

    bno055_set_opmode(bno, bno->mode);
    return BNO_OK;
}

error_bno bno055_acc_conf(bno055_t* bno, const bno055_acc_range_t range, const bno055_acc_band_t bandwidth, const bno055_acc_mode_t mode) {
    bno055_set_page(bno, BNO_PAGE_1);
    bno055_set_opmode(bno, BNO_MODE_CONFIG);
    u8 config = range | bandwidth | mode;
    bno055_write_reg(bno, BNO_ACC_CONFIG, &config, 1);
    bno055_set_opmode(bno, bno->mode);
    bno055_set_page(bno, BNO_PAGE_0);
    return BNO_OK;
}

error_bno bno055_gyr_conf(bno055_t* bno, const bno055_gyr_range_t range, const bno055_gyr_band_t bandwidth, const bno055_gyr_mode_t mode) {
    bno055_set_page(bno, BNO_PAGE_1);
    bno055_set_opmode(bno, BNO_MODE_CONFIG);
    u8 config[2] = {range | bandwidth, mode};
    bno055_write_reg(bno, BNO_GYR_CONFIG_0, config, 2);
    bno055_set_opmode(bno, bno->mode);
    bno055_set_page(bno, BNO_PAGE_0);
    return BNO_OK;
}

error_bno bno055_mag_conf(bno055_t* bno, const bno055_mag_rate_t out_rate, const bno055_mag_pwr_t pwr_mode, const bno055_mag_mode_t mode) {
    bno055_set_page(bno, BNO_PAGE_1);
    bno055_set_opmode(bno, BNO_MODE_CONFIG);
    u8 config = out_rate | pwr_mode | mode;
    bno055_write_reg(bno, BNO_MAG_CONFIG, &config, 1);
    bno055_set_opmode(bno, bno->mode);
    bno055_set_page(bno, BNO_PAGE_0);
    return BNO_OK;
}

error_bno bno055_start_dma_read(bno055_t* imu) {
    imu->dma_data_ready = false;
    if (HAL_I2C_Mem_Read_DMA(imu->i2c, imu->addr, BNO_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, imu->rx_buf, 45) != HAL_OK) {
        return BNO_ERR_I2C;
    }
    return BNO_OK;
}


void bno055_get_acc(bno055_t* imu, bno055_vec3_t* buf) {
    float scale = (imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2 : BNO_ACC_SCALE_MG;
    buf->x = (s16)((imu->rx_buf[1] << 8) | imu->rx_buf[0]) / scale;
    buf->y = (s16)((imu->rx_buf[3] << 8) | imu->rx_buf[2]) / scale;
    buf->z = (s16)((imu->rx_buf[5] << 8) | imu->rx_buf[4]) / scale;
}

void bno055_get_mag(bno055_t* imu, bno055_vec3_t* buf) {
    buf->x = (s16)((imu->rx_buf[7] << 8)  | imu->rx_buf[6])  / BNO_MAG_SCALE;
    buf->y = (s16)((imu->rx_buf[9] << 8)  | imu->rx_buf[8])  / BNO_MAG_SCALE;
    buf->z = (s16)((imu->rx_buf[11] << 8) | imu->rx_buf[10]) / BNO_MAG_SCALE;
}

void bno055_get_gyro(bno055_t* imu, bno055_vec3_t* buf) {
    f32 scale = (imu->_gyr_unit == BNO_GYR_UNIT_DPS) ? BNO_GYR_SCALE_DPS : BNO_GYR_SCALE_RPS;
    buf->x = (s16)((imu->rx_buf[13] << 8) | imu->rx_buf[12]) / scale;
    buf->y = (s16)((imu->rx_buf[15] << 8) | imu->rx_buf[14]) / scale;
    buf->z = (s16)((imu->rx_buf[17] << 8) | imu->rx_buf[16]) / scale;
}

void bno055_get_euler(bno055_t* imu, bno055_euler_t* buf) {
    f32 scale = (imu->_eul_unit == BNO_EUL_UNIT_DEG) ? BNO_EUL_SCALE_DEG : BNO_EUL_SCALE_RAD;
    buf->yaw   = (s16)((imu->rx_buf[19] << 8) | imu->rx_buf[18]) / scale;
    buf->roll  = (s16)((imu->rx_buf[21] << 8) | imu->rx_buf[20]) / scale;
    buf->pitch = (s16)((imu->rx_buf[23] << 8) | imu->rx_buf[22]) / scale;
}

void bno055_get_quaternion(bno055_t* imu, bno055_vec4_t* buf) {
    buf->w = (s16)((imu->rx_buf[25] << 8) | imu->rx_buf[24]) / (f32)BNO_QUA_SCALE;
    buf->x = (s16)((imu->rx_buf[27] << 8) | imu->rx_buf[26]) / (f32)BNO_QUA_SCALE;
    buf->y = (s16)((imu->rx_buf[29] << 8) | imu->rx_buf[28]) / (f32)BNO_QUA_SCALE;
    buf->z = (s16)((imu->rx_buf[31] << 8) | imu->rx_buf[30]) / (f32)BNO_QUA_SCALE;
}

void bno055_get_linear_acc(bno055_t* imu, bno055_vec3_t* buf) {
    float scale = (imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2 : BNO_ACC_SCALE_MG;
    buf->x = (s16)((imu->rx_buf[33] << 8) | imu->rx_buf[32]) / scale;
    buf->y = (s16)((imu->rx_buf[35] << 8) | imu->rx_buf[34]) / scale;
    buf->z = (s16)((imu->rx_buf[37] << 8) | imu->rx_buf[36]) / scale;
}

void bno055_get_gravity(bno055_t* imu, bno055_vec3_t* buf) {
    f32 scale = (imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2 : BNO_ACC_SCALE_MG;
    buf->x = (s16)((imu->rx_buf[39] << 8) | imu->rx_buf[38]) / scale;
    buf->y = (s16)((imu->rx_buf[41] << 8) | imu->rx_buf[40]) / scale;
    buf->z = (s16)((imu->rx_buf[43] << 8) | imu->rx_buf[42]) / scale;
}

void bno055_get_temp(bno055_t* imu, s8* buf) {
    *buf = (imu->_temp_unit) ? imu->rx_buf[44] * 2 : imu->rx_buf[44];
}