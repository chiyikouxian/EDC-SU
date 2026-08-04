#include "imu_sensing.h"

#include <limits.h>

#include "system_time.h"
#include "ti_msp_dl_config.h"

static ICM42688PDevice_t imu_device;
static volatile ImuSnapshot_t imu_snapshot;
static uint32_t next_sample_ms;
static uint32_t next_retry_ms;

static uint32_t enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void exit_critical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/* Re-probes the bus and re-configures the part after a failed read. Called on
   a slow cadence so a missing sensor cannot monopolise the control loop. */
static void try_recover(void)
{
    SoftI2CStatus_t bus_status;
    ICM42688PStatus_t status;
    uint32_t finished_ms;
    uint32_t primask;

    bus_status = soft_i2c_init();
    if (bus_status == SOFT_I2C_STATUS_OK) {
        status = icm42688p_init(&imu_device);
    } else {
        status = ICM42688P_STATUS_I2C_ERROR;
        imu_device.last_i2c_status = bus_status;
        imu_device.initialized = false;
    }
    finished_ms = system_time_ms();

    primask = enter_critical();
    imu_snapshot.recovery_attempt_count++;
    imu_snapshot.status = status;
    imu_snapshot.i2c_status = imu_device.last_i2c_status;
    imu_snapshot.address = imu_device.address;
    imu_snapshot.who_am_i = imu_device.who_am_i;
    imu_snapshot.valid = false;
    imu_snapshot.fresh = false;
    if (status == ICM42688P_STATUS_OK) {
        imu_snapshot.recovery_success_count++;
    } else {
        imu_snapshot.error_count++;
    }
    exit_critical(primask);

    if (status == ICM42688P_STATUS_OK) {
        next_sample_ms = finished_ms + IMU_SAMPLE_PERIOD_MS;
    } else {
        next_retry_ms = finished_ms + IMU_RETRY_PERIOD_MS;
    }
}

void imu_sensing_init(uint32_t now_ms)
{
    SoftI2CStatus_t bus_status;
    ICM42688PStatus_t status;

    imu_snapshot.valid = false;
    imu_snapshot.fresh = false;
    imu_snapshot.sample_count = 0U;
    imu_snapshot.error_count = 0U;
    imu_snapshot.recovery_attempt_count = 0U;
    imu_snapshot.recovery_success_count = 0U;
    imu_snapshot.late_count = 0U;
    imu_snapshot.last_transaction_ms = 0U;
    imu_snapshot.max_transaction_ms = 0U;
    imu_snapshot.timestamp_ms = now_ms;

    bus_status = soft_i2c_init();
    if (bus_status == SOFT_I2C_STATUS_OK) {
        status = icm42688p_init(&imu_device);
    } else {
        status = ICM42688P_STATUS_I2C_ERROR;
        imu_device.last_i2c_status = bus_status;
        imu_device.initialized = false;
    }
    imu_snapshot.status = status;
    imu_snapshot.i2c_status = imu_device.last_i2c_status;
    imu_snapshot.address = imu_device.address;
    imu_snapshot.who_am_i = imu_device.who_am_i;
    if (status != ICM42688P_STATUS_OK) {
        imu_snapshot.error_count = 1U;
    }
    /* icm42688p_init() spends ~54 ms in its mandated settling delays, so
       re-read the clock rather than scheduling from the stale now_ms. */
    now_ms = system_time_ms();
    next_sample_ms = now_ms + IMU_SAMPLE_PERIOD_MS;
    next_retry_ms = now_ms + IMU_RETRY_PERIOD_MS;
}

void imu_sensing_process(uint32_t now_ms)
{
    ICM42688PRawData_t raw;
    ICM42688PScaledData_t scaled;
    ICM42688PStatus_t status;
    uint32_t started_ms;
    uint32_t finished_ms;
    uint32_t duration_ms;
    uint32_t primask;

    if (!imu_device.initialized) {
        if ((int32_t)(now_ms - next_retry_ms) >= 0) {
            try_recover();
        }
        return;
    }
    if ((int32_t)(now_ms - next_sample_ms) < 0) {
        return;
    }
    if ((uint32_t)(now_ms - next_sample_ms) > 1U) {
        imu_snapshot.late_count++;
    }
    next_sample_ms = now_ms + IMU_SAMPLE_PERIOD_MS;
    imu_snapshot.fresh = false;

    started_ms = system_time_ms();
    status = icm42688p_read_raw(&imu_device, &raw);
    finished_ms = system_time_ms();
    duration_ms = finished_ms - started_ms;
    if (duration_ms > UINT16_MAX) {
        duration_ms = UINT16_MAX;
    }
    if (status == ICM42688P_STATUS_OK) {
        /* Scale outside the critical section: it is pure arithmetic on locals
           and would otherwise lengthen the window with interrupts masked. */
        icm42688p_scale(&raw, &scaled);
    }

    primask = enter_critical();
    imu_snapshot.status = status;
    imu_snapshot.i2c_status = imu_device.last_i2c_status;
    imu_snapshot.last_transaction_ms = (uint16_t)duration_ms;
    if (duration_ms > imu_snapshot.max_transaction_ms) {
        imu_snapshot.max_transaction_ms = (uint16_t)duration_ms;
    }
    if (status == ICM42688P_STATUS_OK) {
        imu_snapshot.raw = raw;
        imu_snapshot.scaled = scaled;
        imu_snapshot.timestamp_ms = finished_ms;
        imu_snapshot.sample_count++;
        imu_snapshot.valid = true;
        imu_snapshot.fresh = true;
    } else {
        imu_snapshot.error_count++;
        imu_snapshot.valid = false;
        next_retry_ms = finished_ms + IMU_RETRY_PERIOD_MS;
    }
    exit_critical(primask);
}

ImuSnapshot_t imu_sensing_get_snapshot(void)
{
    ImuSnapshot_t snapshot;
    uint32_t primask = enter_critical();

    snapshot = imu_snapshot;
    imu_snapshot.fresh = false;
    exit_critical(primask);
    return snapshot;
}
