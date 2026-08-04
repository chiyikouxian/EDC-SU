#ifndef IMU_SENSING_H
#define IMU_SENSING_H

#include <stdbool.h>
#include <stdint.h>

#include "icm42688p.h"

/* Sampling cadence.
 *
 * A 14-byte burst read over bit-banged I2C blocks for roughly 1.2 ms, which is
 * longer than one main-loop tick. Sampling every 20 ms keeps that cost near
 * 6% of the loop instead of the ~12% a 10 ms period would cost. 50 Hz is
 * ample here: the IMU informs ball-balance compensation and chassis motion
 * checks, while line tracking stays on the grayscale array.
 *
 * If the ball loop later needs faster data, lower this to 10 ms and drop
 * SOFT_I2C_HALF_PERIOD_US to 2 to keep the blocking budget unchanged.
 */
#define IMU_SAMPLE_PERIOD_MS 20U
#define IMU_RETRY_PERIOD_MS  500U

typedef struct {
    /* Sensor-frame counts, for diagnostics and re-scaling. */
    ICM42688PRawData_t raw;
    /* Fixed-point engineering units: accel_mg (1000 = 1 g),
       gyro_dps_x10 (10 = 1 deg/s), temp_cdegc (2500 = 25.00 C). */
    ICM42688PScaledData_t scaled;

    ICM42688PStatus_t status;
    SoftI2CStatus_t i2c_status;

    uint32_t timestamp_ms;      /* system_time_ms() when the sample completed */
    uint32_t sample_count;
    uint32_t error_count;
    uint32_t recovery_attempt_count;
    uint32_t recovery_success_count;
    uint32_t late_count;        /* sample slots missed by >1 ms */
    uint16_t last_transaction_ms;
    uint16_t max_transaction_ms;
    uint8_t address;
    uint8_t who_am_i;
    bool valid;                 /* last read succeeded */
    bool fresh;                 /* new since the previous get_snapshot() */
} ImuSnapshot_t;

void imu_sensing_init(uint32_t now_ms);
void imu_sensing_process(uint32_t now_ms);

/* Returns a consistent copy and clears the fresh flag. */
ImuSnapshot_t imu_sensing_get_snapshot(void);

#endif /* IMU_SENSING_H */
