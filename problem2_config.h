#ifndef PROBLEM2_CONFIG_H
#define PROBLEM2_CONFIG_H

#define P2_DISABLE_RUN_TIMEOUT              1
#define P2_DEBUG_STOP_ON_START_KEY          1

#define P2_A_LINE_CENTER_CHANNELS           3U
#define P2_A_LINE_RELEASE_CHANNELS          3U
#define P2_A_LINE_CONFIRM_TICKS             1U
#define P2_A_LINE_MIN_STOP_COUNTS           18000
#define P2_PURE_LINE_TRACK_TEST             0

#define P2_BASE_SPEED                       35
#define P2_STRAIGHT_SPEED                   41
#define P2_CURVE_SPEED                      39
#define P2_FIND_SPEED                       15
#define P2_MAX_SPEED                        48
#define P2_TURN_LIMIT                       10
#define P2_CURVE_ERROR                      45
#define P2_FINISH_SLOWDOWN_COUNTS           21800
#define P2_FINISH_SLOWDOWN_SPEED            22  

#define P2_LINE_PID_KP                      20
#define P2_LINE_PID_KI                      0
#define P2_LINE_PID_KD                      4
#define P2_LINE_ACCEL_STEP                  1
#define P2_LINE_DECEL_STEP                  2

#endif