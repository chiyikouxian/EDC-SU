#ifndef PROBLEM56_CONFIG_H
#define PROBLEM56_CONFIG_H

#define P56_A_LINE_CENTER_CHANNELS          3U
#define P56_A_LINE_RELEASE_CHANNELS         3U
#define P56_A_LINE_CONFIRM_TICKS            1U
#define P56_A_LINE_MIN_STOP_COUNTS          18000

#define P56_BASE_SPEED                      24
#define P56_STRAIGHT_SPEED                  28
#define P56_CURVE_SPEED                     22
#define P56_FIND_SPEED                      12
#define P56_MAX_SPEED                       34
#define P56_TURN_LIMIT                      9
#define P56_CURVE_ERROR                     45
#define P56_FINISH_SLOWDOWN_COUNTS          21800
#define P56_FINISH_SLOWDOWN_SPEED           12

#define P56_LINE_PID_KP                     18
#define P56_LINE_PID_KI                     0
#define P56_LINE_PID_KD                     4
#define P56_LINE_ACCEL_STEP                 1
#define P56_LINE_DECEL_STEP                 2

/* Item 5/6 uses the same pendulum IMU damping topology as item 4.  Keep a
   separate set of knobs so vehicle-speed changes can be tuned independently. */
#define P56_IMU_BALANCE_ENABLE               1U
#define P56_IMU_GYRO_X_SIGN                  1L
#define P56_IMU_GYRO_X_DAMP_CDEG_PER_DPS_X10 2L
#define P56_IMU_GYRO_X_DAMP_LIMIT_CDEG       300L
#define P56_IMU_MAX_AGE_MS                   80U

#endif
