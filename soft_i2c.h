#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stddef.h>
#include <stdint.h>

/* Bit-banged I2C master on IMU_I2C_SCL/SDA (PB17/PB12). Open-drain emulated
   by releasing the line to the internal pull-up; the pins are never driven
   HIGH. Blocking: a 14-byte burst read costs roughly 1.2 ms. */

typedef enum {
    SOFT_I2C_STATUS_OK = 0,
    SOFT_I2C_STATUS_INVALID_ARGUMENT,
    SOFT_I2C_STATUS_BUS_BUSY,
    SOFT_I2C_STATUS_SCL_TIMEOUT,
    SOFT_I2C_STATUS_ADDRESS_NACK,
    SOFT_I2C_STATUS_DATA_NACK
} SoftI2CStatus_t;

SoftI2CStatus_t soft_i2c_init(void);
SoftI2CStatus_t soft_i2c_recover(void);
SoftI2CStatus_t soft_i2c_write_register(
    uint8_t address, uint8_t reg, const uint8_t *data, size_t length);
SoftI2CStatus_t soft_i2c_read_register(
    uint8_t address, uint8_t reg, uint8_t *data, size_t length);
void soft_i2c_delay_ms(uint32_t milliseconds);

#endif /* SOFT_I2C_H */
