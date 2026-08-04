#ifndef APP_COMMON_H
#define APP_COMMON_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    RUN_MODE_BASIC_DRIVE = 0,
    RUN_MODE_BASIC_NAV,
    RUN_MODE_ADVANCED_NAV,
    RUN_MODE_PROBLEM3,
    RUN_MODE_PROBLEM4,
    RUN_MODE_PROBLEM56,
    RUN_MODE_COUNT
} RunMode_t;

typedef enum {
    SHAPE_NONE = 0,
    SHAPE_CIRCLE,
    SHAPE_TRIANGLE,
    SHAPE_RECT,
    SHAPE_PENTAGON
} Shape_t;

typedef enum {
    TARGET_NONE = 0,
    TARGET_A,
    TARGET_B,
    TARGET_C,
    TARGET_D
} TargetPoint_t;

typedef enum {
    APP_STATE_IDLE = 0,
    APP_STATE_FIXED_DETECT,
    APP_STATE_GIMBAL_SCAN,
    APP_STATE_START_BEEP,
    APP_STATE_RUNNING,
    APP_STATE_TARGET_STOP,
    APP_STATE_FINISH_BEEP,
    APP_STATE_ERROR
} AppState_t;

#define TICK_MS          10
#define TICKS_PER_SEC    (1000 / TICK_MS)

#endif /* APP_COMMON_H */
