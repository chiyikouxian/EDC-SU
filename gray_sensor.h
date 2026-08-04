#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include "app_common.h"

/*
 * 6-channel IR line sensor over software I2C.
 *
 * Protocol from "2.快速上手.pdf":
 *   I2C address 0x5C, register 5 returns all six digital channel results.
 *
 * Wiring for this car:
 *   PB1 = SCL, PB2 = SDA.
 * The existing SysConfig names these old pins as GRAY_SENSOR_AD1/AD2; the
 * driver aliases them as I2C lines to avoid regenerating SysConfig now.
 */

#define GRAY_SENSOR_CHANNELS  6

/* Active level: 1 = black line makes OUT high.  Change to 0 for inverted modules. */
#ifndef GRAY_ACTIVE_LEVEL
#define GRAY_ACTIVE_LEVEL  1
#endif

/* Software I2C timing. */
#define GRAY_I2C_DELAY_US           5

void gray_sensor_init(void);
void gray_sensor_read_all(uint8_t values[GRAY_SENSOR_CHANNELS]);
bool gray_sensor_read_ok(void);
bool gray_sensor_has_line(const uint8_t values[GRAY_SENSOR_CHANNELS]);
bool gray_sensor_is_all_white(const uint8_t values[GRAY_SENSOR_CHANNELS]);
const char *gray_sensor_debug_string(const uint8_t values[GRAY_SENSOR_CHANNELS]);

#endif /* GRAY_SENSOR_H */
