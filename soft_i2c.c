#include "soft_i2c.h"

#include <stdbool.h>

#include "ti_msp_dl_config.h"

#define SOFT_I2C_HALF_PERIOD_US       4U
#define SOFT_I2C_SCL_TIMEOUT_US       1000U
#define SOFT_I2C_CYCLES_PER_US        (CPUCLK_FREQ / 1000000U)

static void delay_us(uint32_t microseconds)
{
    delay_cycles(SOFT_I2C_CYCLES_PER_US * microseconds);
}

void soft_i2c_delay_ms(uint32_t milliseconds)
{
    while (milliseconds > 0U) {
        delay_cycles(CPUCLK_FREQ / 1000U);
        milliseconds--;
    }
}

static void scl_low(void)
{
    DL_GPIO_clearPins(IMU_I2C_PORT, IMU_I2C_IMU_SCL_PIN);
    DL_GPIO_enableOutput(IMU_I2C_PORT, IMU_I2C_IMU_SCL_PIN);
}

static SoftI2CStatus_t scl_release(void)
{
    uint32_t timeout = SOFT_I2C_SCL_TIMEOUT_US;

    DL_GPIO_disableOutput(IMU_I2C_PORT, IMU_I2C_IMU_SCL_PIN);
    while ((DL_GPIO_readPins(IMU_I2C_PORT, IMU_I2C_IMU_SCL_PIN) == 0U) &&
           (timeout > 0U)) {
        delay_us(1U);
        timeout--;
    }
    return (timeout > 0U) ? SOFT_I2C_STATUS_OK :
                            SOFT_I2C_STATUS_SCL_TIMEOUT;
}

static void sda_low(void)
{
    DL_GPIO_clearPins(IMU_I2C_PORT, IMU_I2C_IMU_SDA_PIN);
    DL_GPIO_enableOutput(IMU_I2C_PORT, IMU_I2C_IMU_SDA_PIN);
}

static void sda_release(void)
{
    DL_GPIO_disableOutput(IMU_I2C_PORT, IMU_I2C_IMU_SDA_PIN);
}

static bool sda_is_high(void)
{
    return DL_GPIO_readPins(IMU_I2C_PORT, IMU_I2C_IMU_SDA_PIN) != 0U;
}

static SoftI2CStatus_t start_condition(void)
{
    SoftI2CStatus_t status;

    sda_release();
    status = scl_release();
    if (status != SOFT_I2C_STATUS_OK) {
        return status;
    }
    delay_us(SOFT_I2C_HALF_PERIOD_US);
    if (!sda_is_high()) {
        return SOFT_I2C_STATUS_BUS_BUSY;
    }
    sda_low();
    delay_us(SOFT_I2C_HALF_PERIOD_US);
    scl_low();
    return SOFT_I2C_STATUS_OK;
}

static void stop_condition(void)
{
    sda_low();
    delay_us(SOFT_I2C_HALF_PERIOD_US);
    (void)scl_release();
    delay_us(SOFT_I2C_HALF_PERIOD_US);
    sda_release();
    delay_us(SOFT_I2C_HALF_PERIOD_US);
}

static SoftI2CStatus_t write_byte(uint8_t value, bool *acknowledged)
{
    SoftI2CStatus_t status;
    uint8_t mask;

    if (acknowledged == NULL) {
        return SOFT_I2C_STATUS_INVALID_ARGUMENT;
    }
    for (mask = 0x80U; mask != 0U; mask >>= 1U) {
        if ((value & mask) != 0U) {
            sda_release();
        } else {
            sda_low();
        }
        delay_us(SOFT_I2C_HALF_PERIOD_US);
        status = scl_release();
        if (status != SOFT_I2C_STATUS_OK) {
            return status;
        }
        delay_us(SOFT_I2C_HALF_PERIOD_US);
        scl_low();
    }
    sda_release();
    delay_us(SOFT_I2C_HALF_PERIOD_US);
    status = scl_release();
    if (status != SOFT_I2C_STATUS_OK) {
        return status;
    }
    delay_us(SOFT_I2C_HALF_PERIOD_US);
    *acknowledged = !sda_is_high();
    scl_low();
    return SOFT_I2C_STATUS_OK;
}

static SoftI2CStatus_t read_byte(uint8_t *value, bool acknowledge)
{
    SoftI2CStatus_t status;
    uint8_t data = 0U;
    uint8_t bit;

    if (value == NULL) {
        return SOFT_I2C_STATUS_INVALID_ARGUMENT;
    }
    sda_release();
    for (bit = 0U; bit < 8U; bit++) {
        data <<= 1U;
        delay_us(SOFT_I2C_HALF_PERIOD_US);
        status = scl_release();
        if (status != SOFT_I2C_STATUS_OK) {
            return status;
        }
        delay_us(SOFT_I2C_HALF_PERIOD_US);
        if (sda_is_high()) {
            data |= 1U;
        }
        scl_low();
    }
    if (acknowledge) {
        sda_low();
    } else {
        sda_release();
    }
    delay_us(SOFT_I2C_HALF_PERIOD_US);
    status = scl_release();
    if (status != SOFT_I2C_STATUS_OK) {
        return status;
    }
    delay_us(SOFT_I2C_HALF_PERIOD_US);
    scl_low();
    sda_release();
    *value = data;
    return SOFT_I2C_STATUS_OK;
}

SoftI2CStatus_t soft_i2c_recover(void)
{
    SoftI2CStatus_t status;
    uint8_t pulse;

    sda_release();
    status = scl_release();
    if (status != SOFT_I2C_STATUS_OK) {
        return status;
    }
    for (pulse = 0U; !sda_is_high() && (pulse < 9U); pulse++) {
        scl_low();
        delay_us(SOFT_I2C_HALF_PERIOD_US);
        status = scl_release();
        if (status != SOFT_I2C_STATUS_OK) {
            return status;
        }
        delay_us(SOFT_I2C_HALF_PERIOD_US);
    }
    stop_condition();
    return sda_is_high() ? SOFT_I2C_STATUS_OK : SOFT_I2C_STATUS_BUS_BUSY;
}

SoftI2CStatus_t soft_i2c_init(void)
{
    DL_GPIO_clearPins(IMU_I2C_PORT,
        IMU_I2C_IMU_SCL_PIN | IMU_I2C_IMU_SDA_PIN);
    DL_GPIO_disableOutput(IMU_I2C_PORT,
        IMU_I2C_IMU_SCL_PIN | IMU_I2C_IMU_SDA_PIN);
    delay_us(SOFT_I2C_HALF_PERIOD_US);
    return soft_i2c_recover();
}

SoftI2CStatus_t soft_i2c_write_register(
    uint8_t address, uint8_t reg, const uint8_t *data, size_t length)
{
    SoftI2CStatus_t status;
    bool ack;
    size_t index;

    if ((address > 0x7FU) || ((length > 0U) && (data == NULL))) {
        return SOFT_I2C_STATUS_INVALID_ARGUMENT;
    }
    status = start_condition();
    if (status != SOFT_I2C_STATUS_OK) {
        return status;
    }
    status = write_byte((uint8_t)(address << 1U), &ack);
    if ((status != SOFT_I2C_STATUS_OK) || !ack) {
        stop_condition();
        return (status != SOFT_I2C_STATUS_OK) ? status :
                                                SOFT_I2C_STATUS_ADDRESS_NACK;
    }
    status = write_byte(reg, &ack);
    if ((status != SOFT_I2C_STATUS_OK) || !ack) {
        stop_condition();
        return (status != SOFT_I2C_STATUS_OK) ? status :
                                                SOFT_I2C_STATUS_DATA_NACK;
    }
    for (index = 0U; index < length; index++) {
        status = write_byte(data[index], &ack);
        if ((status != SOFT_I2C_STATUS_OK) || !ack) {
            stop_condition();
            return (status != SOFT_I2C_STATUS_OK) ? status :
                                                    SOFT_I2C_STATUS_DATA_NACK;
        }
    }
    stop_condition();
    return SOFT_I2C_STATUS_OK;
}

SoftI2CStatus_t soft_i2c_read_register(
    uint8_t address, uint8_t reg, uint8_t *data, size_t length)
{
    SoftI2CStatus_t status;
    bool ack;
    size_t index;

    if ((address > 0x7FU) || (data == NULL) || (length == 0U)) {
        return SOFT_I2C_STATUS_INVALID_ARGUMENT;
    }
    status = start_condition();
    if (status != SOFT_I2C_STATUS_OK) {
        return status;
    }
    status = write_byte((uint8_t)(address << 1U), &ack);
    if ((status != SOFT_I2C_STATUS_OK) || !ack) {
        stop_condition();
        return (status != SOFT_I2C_STATUS_OK) ? status :
                                                SOFT_I2C_STATUS_ADDRESS_NACK;
    }
    status = write_byte(reg, &ack);
    if ((status != SOFT_I2C_STATUS_OK) || !ack) {
        stop_condition();
        return (status != SOFT_I2C_STATUS_OK) ? status :
                                                SOFT_I2C_STATUS_DATA_NACK;
    }
    status = start_condition();
    if (status != SOFT_I2C_STATUS_OK) {
        stop_condition();
        return status;
    }
    status = write_byte((uint8_t)((address << 1U) | 1U), &ack);
    if ((status != SOFT_I2C_STATUS_OK) || !ack) {
        stop_condition();
        return (status != SOFT_I2C_STATUS_OK) ? status :
                                                SOFT_I2C_STATUS_ADDRESS_NACK;
    }
    for (index = 0U; index < length; index++) {
        status = read_byte(&data[index], index < (length - 1U));
        if (status != SOFT_I2C_STATUS_OK) {
            stop_condition();
            return status;
        }
    }
    stop_condition();
    return SOFT_I2C_STATUS_OK;
}
