#ifndef ICM42688P_H
#define ICM42688P_H

#include <stdbool.h>
#include <stdint.h>

#include "soft_i2c.h"

#define ICM42688P_WHO_AM_I_VALUE 0x47U

/* Full-scale and rate chosen for this chassis rather than the part defaults.
 *
 * Accel +/-4 g: on the 0.5 m arcs the centripetal term is only about 0.05 g
 * (v^2/r at ~0.5 m/s) on top of 1 g of gravity, so 4 g leaves headroom for
 * vibration spikes while giving 0.12 mg/LSB -- 4x the resolution of the
 * +/-16 g reset default.
 * Gyro +/-500 dps: the same arc at 0.5 m/s is only ~57 dps.
 * 100 Hz ODR matches the 10 ms control tick; the 1 kHz reset default would
 * just burn bus time producing samples nothing reads.
 *
 * These are declarations of intent for readers. The bits actually written are
 * ICM42688P_{ACCEL,GYRO}_CONFIG0_VALUE in icm42688p.c, and the scale ratios
 * below depend on them -- change all three together or the reported units
 * will silently be wrong.
 */
#define ICM42688P_ACCEL_FS_G   4
#define ICM42688P_GYRO_FS_DPS  500
#define ICM42688P_ODR_HZ       100

typedef enum {
    ICM42688P_AXIS_X = 0,
    ICM42688P_AXIS_Y,
    ICM42688P_AXIS_Z,
    ICM42688P_AXIS_COUNT
} ICM42688PAxis_t;

typedef enum {
    ICM42688P_STATUS_OK = 0,
    ICM42688P_STATUS_INVALID_ARGUMENT,
    ICM42688P_STATUS_NOT_FOUND,
    ICM42688P_STATUS_BAD_WHO_AM_I,
    ICM42688P_STATUS_I2C_ERROR
} ICM42688PStatus_t;

typedef struct {
    uint8_t address;
    uint8_t who_am_i;
    SoftI2CStatus_t last_i2c_status;
    bool initialized;
} ICM42688PDevice_t;

/* Sensor-frame counts exactly as the chip reported them. Kept alongside the
   scaled values so a suspect reading can be traced back to raw bytes. */
typedef struct {
    int16_t temperature;
    int16_t accel[ICM42688P_AXIS_COUNT];
    int16_t gyro[ICM42688P_AXIS_COUNT];
} ICM42688PRawData_t;

/* Fixed-point engineering units, matching the project convention of an
   explicit scale suffix and no floating point in control paths.
     accel_mg     : milli-g        (1000 = 1 g)
     gyro_dps_x10 : deci-deg/s     (10 = 1 deg/s)
     temp_cdegc   : centi-Celsius  (2500 = 25.00 C)
   Full scale lands at 4000 mg and 5000 deci-dps, both inside int16. Note the
   gyro is deliberately NOT centi-dps: 500 dps would be 50000, which overflows
   int16 and would wrap negative on a hard spin. Resolution is still 0.15 dps
   per LSB, far finer than this chassis needs. */
typedef struct {
    int16_t accel_mg[ICM42688P_AXIS_COUNT];
    int16_t gyro_dps_x10[ICM42688P_AXIS_COUNT];
    int16_t temp_cdegc;
} ICM42688PScaledData_t;

ICM42688PStatus_t icm42688p_init(ICM42688PDevice_t *device);
ICM42688PStatus_t icm42688p_read_raw(
    ICM42688PDevice_t *device, ICM42688PRawData_t *data);
void icm42688p_scale(
    const ICM42688PRawData_t *raw, ICM42688PScaledData_t *scaled);

#endif /* ICM42688P_H */
