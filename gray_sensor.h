#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include "app_common.h"

/*
 * 8-channel red-light grayscale module:
 *   AD0/AD1/AD2 select X1..X8, OUT returns the selected comparator level.
 *
 * Pin definitions come from ti_msp_dl_config.h:
 *   GRAY_SENSOR_PORT, GRAY_SENSOR_AD0_PIN, GRAY_SENSOR_AD1_PIN,
 *   GRAY_SENSOR_AD2_PIN, GRAY_SENSOR_OUT_PIN.
 *
 * Channel order:
 *   000 -> X1  001 -> X2  010 -> X3  011 -> X4
 *   100 -> X5  101 -> X6  110 -> X7  111 -> X8
 */

#define GRAY_SENSOR_CHANNELS  8

#define GRAY_AD_MASK  (GRAY_SENSOR_AD0_PIN | GRAY_SENSOR_AD1_PIN | GRAY_SENSOR_AD2_PIN)

/* Active level: 1 = black line makes OUT high.  Change to 0 for inverted modules. */
#ifndef GRAY_ACTIVE_LEVEL
#define GRAY_ACTIVE_LEVEL  1
#endif

/* Mux/comparator settle and anti-noise sampling. */
#define GRAY_SETTLE_US             50
#define GRAY_SAMPLE_COUNT           5
#define GRAY_SAMPLE_INTERVAL_US     6

void gray_sensor_init(void);
void gray_sensor_read_all(uint8_t values[GRAY_SENSOR_CHANNELS]);
bool gray_sensor_has_line(const uint8_t values[GRAY_SENSOR_CHANNELS]);
bool gray_sensor_is_all_white(const uint8_t values[GRAY_SENSOR_CHANNELS]);
const char *gray_sensor_debug_string(const uint8_t values[GRAY_SENSOR_CHANNELS]);

#endif /* GRAY_SENSOR_H */
