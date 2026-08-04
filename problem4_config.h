#ifndef PROBLEM4_CONFIG_H
#define PROBLEM4_CONFIG_H

#define P4_AB_TARGET_COUNTS                 5200
#define P4_AB_SLOWDOWN_COUNTS               4300
#define P4_MANUAL_PUSH_MEASURE              0

#define P4_B_LINE_ACTIVE_CHANNELS           5U
#define P4_B_LINE_RELEASE_CHANNELS          3U
#define P4_B_LINE_CONFIRM_TICKS             1U

#define P4_BASE_SPEED                       26
#define P4_STRAIGHT_SPEED                   30
#define P4_CURVE_SPEED                      24
#define P4_FIND_SPEED                       12
#define P4_MAX_SPEED                        36
#define P4_TURN_LIMIT                       10
#define P4_CURVE_ERROR                      45
#define P4_AB_SLOWDOWN_SPEED                18

#define P4_LINE_PID_KP                      18
#define P4_LINE_PID_KI                      0
#define P4_LINE_PID_KD                      4
#define P4_LINE_ACCEL_STEP                  1
#define P4_LINE_DECEL_STEP                  2

/* Item 4 pendulum damping.
 * The IMU is mounted with Y along the rod, so rod tilt is rotation about X.
 * gyro_dps_x10 is in 0.1 deg/s. A positive rod rate must reduce a positive
 * angle command, hence it is subtracted from the PA26 position-loop error.
 * If the physical IMU X direction is reversed, change only the SIGN to -1. */
#define P4_IMU_BALANCE_ENABLE               1U
#define P4_IMU_GYRO_X_SIGN                  1L
#define P4_IMU_GYRO_X_DAMP_CDEG_PER_DPS_X10 2L
#define P4_IMU_GYRO_X_DAMP_LIMIT_CDEG       300L
#define P4_IMU_MAX_AGE_MS                   80U

#endif
