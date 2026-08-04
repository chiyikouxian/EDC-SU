#include "icm42688p.h"

#include <stddef.h>

#define ICM42688P_ADDRESS_AD0_LOW       0x68U
#define ICM42688P_ADDRESS_AD0_HIGH      0x69U
#define ICM42688P_REG_DEVICE_CONFIG     0x11U
#define ICM42688P_REG_TEMP_DATA1        0x1DU
#define ICM42688P_REG_PWR_MGMT0         0x4EU
#define ICM42688P_REG_GYRO_CONFIG0      0x4FU
#define ICM42688P_REG_ACCEL_CONFIG0     0x50U
#define ICM42688P_REG_WHO_AM_I          0x75U
#define ICM42688P_DEVICE_SOFT_RESET     0x01U
#define ICM42688P_ACCEL_GYRO_LOW_NOISE  0x0FU
#define ICM42688P_RAW_DATA_LENGTH       14U

/* ODR field (bits 3:0), from the datasheet register map. The reference Python
   driver's table disagrees with this for every entry except 100 Hz. */
#define ICM42688P_ODR_1KHZ   0x06U
#define ICM42688P_ODR_200HZ  0x07U
#define ICM42688P_ODR_100HZ  0x08U
#define ICM42688P_ODR_50HZ   0x09U
#define ICM42688P_ODR_25HZ   0x0AU

/* FS_SEL field (bits 7:5). Accel: 0=+/-16 g, 1=+/-8, 2=+/-4, 3=+/-2.
   Gyro: 0=+/-2000 dps, 1=+/-1000, 2=+/-500, 3=+/-250. */
#define ICM42688P_ACCEL_FS_4G      (2U << 5U)
#define ICM42688P_GYRO_FS_500DPS   (2U << 5U)

#define ICM42688P_ACCEL_CONFIG0_VALUE \
    (ICM42688P_ACCEL_FS_4G | ICM42688P_ODR_100HZ)
#define ICM42688P_GYRO_CONFIG0_VALUE \
    (ICM42688P_GYRO_FS_500DPS | ICM42688P_ODR_100HZ)

/* Scaling as exact integer ratios, so no floating point enters the build.
   accel: 4000 mg / 32768 counts     = 125/1024 mg per count
   gyro:  5000 deci-dps / 32768      = 625/4096 deci-dps per count
   temp:  degC = count/132.48 + 25  ->  cdegC = count*100/132.48 + 2500,
          approximated as count*3125/4140 + 2500 (within 0.01 C).
   Intermediates are int32: the largest product is 32767*3125 = 1.02e8, which
   fits comfortably. */
#define ICM42688P_ACCEL_NUM   125
#define ICM42688P_ACCEL_DEN   1024
#define ICM42688P_GYRO_NUM    625
#define ICM42688P_GYRO_DEN    4096
#define ICM42688P_TEMP_NUM    3125
#define ICM42688P_TEMP_DEN    4140
#define ICM42688P_TEMP_OFFSET 2500

static int16_t parse_int16(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[0] << 8U) | bytes[1]);
}

/* Rounds toward nearest on both signs; truncating division would bias every
   reading toward zero and show up as a standing offset in the tilt estimate. */
static int16_t scale_count(int16_t count, int32_t numerator,
    int32_t denominator)
{
    int32_t scaled = (int32_t)count * numerator;

    if (scaled >= 0) {
        scaled = (scaled + (denominator / 2)) / denominator;
    } else {
        scaled = -((-scaled + (denominator / 2)) / denominator);
    }
    return (int16_t)scaled;
}

static bool read_who_am_i(ICM42688PDevice_t *device, uint8_t address)
{
    uint8_t value = 0U;

    device->last_i2c_status = soft_i2c_read_register(
        address, ICM42688P_REG_WHO_AM_I, &value, 1U);
    if (device->last_i2c_status != SOFT_I2C_STATUS_OK) {
        return false;
    }
    device->address = address;
    device->who_am_i = value;
    return true;
}

/* AD0 wiring decides the address, so probe both rather than requiring the
   board to be strapped a particular way. */
static ICM42688PStatus_t find_device(ICM42688PDevice_t *device)
{
    bool responded = false;

    if (read_who_am_i(device, ICM42688P_ADDRESS_AD0_LOW)) {
        responded = true;
        if (device->who_am_i == ICM42688P_WHO_AM_I_VALUE) {
            return ICM42688P_STATUS_OK;
        }
    }
    if (read_who_am_i(device, ICM42688P_ADDRESS_AD0_HIGH)) {
        responded = true;
        if (device->who_am_i == ICM42688P_WHO_AM_I_VALUE) {
            return ICM42688P_STATUS_OK;
        }
    }
    return responded ? ICM42688P_STATUS_BAD_WHO_AM_I :
                       ICM42688P_STATUS_NOT_FOUND;
}

static ICM42688PStatus_t write_one(
    ICM42688PDevice_t *device, uint8_t reg, uint8_t value)
{
    device->last_i2c_status = soft_i2c_write_register(
        device->address, reg, &value, 1U);
    return (device->last_i2c_status == SOFT_I2C_STATUS_OK) ?
        ICM42688P_STATUS_OK : ICM42688P_STATUS_I2C_ERROR;
}

ICM42688PStatus_t icm42688p_init(ICM42688PDevice_t *device)
{
    ICM42688PStatus_t status;

    if (device == NULL) {
        return ICM42688P_STATUS_INVALID_ARGUMENT;
    }
    device->address = 0U;
    device->who_am_i = 0U;
    device->last_i2c_status = SOFT_I2C_STATUS_OK;
    device->initialized = false;

    status = find_device(device);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }
    status = write_one(device, ICM42688P_REG_DEVICE_CONFIG,
        ICM42688P_DEVICE_SOFT_RESET);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }
    /* Datasheet requires 1 ms after soft reset before any register access. */
    soft_i2c_delay_ms(4U);
    if (!read_who_am_i(device, device->address)) {
        return ICM42688P_STATUS_I2C_ERROR;
    }
    if (device->who_am_i != ICM42688P_WHO_AM_I_VALUE) {
        return ICM42688P_STATUS_BAD_WHO_AM_I;
    }
    /* Range and rate must be written before leaving standby: PWR_MGMT0 last. */
    status = write_one(device, ICM42688P_REG_GYRO_CONFIG0,
        ICM42688P_GYRO_CONFIG0_VALUE);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }
    status = write_one(device, ICM42688P_REG_ACCEL_CONFIG0,
        ICM42688P_ACCEL_CONFIG0_VALUE);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }
    status = write_one(device, ICM42688P_REG_PWR_MGMT0,
        ICM42688P_ACCEL_GYRO_LOW_NOISE);
    if (status != ICM42688P_STATUS_OK) {
        return status;
    }
    /* Gyro needs ~45 ms to settle after entering low-noise mode. */
    soft_i2c_delay_ms(50U);
    device->initialized = true;
    return ICM42688P_STATUS_OK;
}

ICM42688PStatus_t icm42688p_read_raw(
    ICM42688PDevice_t *device, ICM42688PRawData_t *data)
{
    uint8_t raw[ICM42688P_RAW_DATA_LENGTH];

    if ((device == NULL) || (data == NULL) || !device->initialized) {
        return ICM42688P_STATUS_INVALID_ARGUMENT;
    }
    /* One burst from TEMP_DATA1 covers temperature, accel and gyro, so all
       three come from the same sample interval. */
    device->last_i2c_status = soft_i2c_read_register(device->address,
        ICM42688P_REG_TEMP_DATA1, raw, sizeof(raw));
    if (device->last_i2c_status != SOFT_I2C_STATUS_OK) {
        /* Force re-init: the caller's retry path re-probes the bus. */
        device->initialized = false;
        return ICM42688P_STATUS_I2C_ERROR;
    }
    data->temperature = parse_int16(&raw[0]);
    data->accel[ICM42688P_AXIS_X] = parse_int16(&raw[2]);
    data->accel[ICM42688P_AXIS_Y] = parse_int16(&raw[4]);
    data->accel[ICM42688P_AXIS_Z] = parse_int16(&raw[6]);
    data->gyro[ICM42688P_AXIS_X] = parse_int16(&raw[8]);
    data->gyro[ICM42688P_AXIS_Y] = parse_int16(&raw[10]);
    data->gyro[ICM42688P_AXIS_Z] = parse_int16(&raw[12]);
    return ICM42688P_STATUS_OK;
}

void icm42688p_scale(
    const ICM42688PRawData_t *raw, ICM42688PScaledData_t *scaled)
{
    uint8_t axis;

    if ((raw == NULL) || (scaled == NULL)) {
        return;
    }
    for (axis = 0U; axis < ICM42688P_AXIS_COUNT; axis++) {
        scaled->accel_mg[axis] = scale_count(raw->accel[axis],
            ICM42688P_ACCEL_NUM, ICM42688P_ACCEL_DEN);
        scaled->gyro_dps_x10[axis] = scale_count(raw->gyro[axis],
            ICM42688P_GYRO_NUM, ICM42688P_GYRO_DEN);
    }
    scaled->temp_cdegc = (int16_t)(scale_count(raw->temperature,
        ICM42688P_TEMP_NUM, ICM42688P_TEMP_DEN) + ICM42688P_TEMP_OFFSET);
}
