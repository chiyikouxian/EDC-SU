#include "chassis_iface.h"
#include "app_state.h"
#include "line_track.h"
#include "motor.h"
#include "pid.h"
#include "ti_msp_dl_config.h"
#include "track_bridge.h"
#include "problem2_config.h"
#include "problem4_config.h"
#include "problem56_config.h"

/* Normal PID telemetry is queued from main.c.  Keep event logging off so a
 * state transition cannot block the 1 ms software-PWM service. */
#ifndef CHASSIS_EVENT_LOG_ENABLE
#define CHASSIS_EVENT_LOG_ENABLE 0
#endif

#if CHASSIS_EVENT_LOG_ENABLE
#include <stdio.h>
#endif

/*
 * Chassis line following.
 *
 * The pin layer stays in motor.c / ti_msp_dl_config.h.  This file only turns
 * the latest track_bridge error into differential wheel speed, using the same
 * control idea as the reference CCD sample:
 *
 *   target_left  = base_speed + turn
 *   target_right = base_speed - turn
 *   turn         = KP * error + KD * (error - last_error)
 */

#define CHASSIS_ENABLE_SHARP_TURN            0  /* Oval track has no 90-degree corners. */

/* 90-degree corner assist.  A left-side black block raises turn_hint=-1.
 * Keep a strong pivot command until the line returns to the centre. */
#define CHASSIS_SHARP_LEFT_INNER_SPEED      -6
#define CHASSIS_SHARP_LEFT_OUTER_SPEED      20
#define CHASSIS_SHARP_RIGHT_OUTER_SPEED     20
#define CHASSIS_SHARP_RIGHT_INNER_SPEED     -6
#define CHASSIS_SHARP_APPROACH_INNER_SPEED   5
#define CHASSIS_SHARP_APPROACH_OUTER_SPEED  15
#define CHASSIS_SHARP_APPROACH_ERROR          45
#define CHASSIS_SHARP_TURN_TIMEOUT_TICKS    80  /* 800 ms at 10 ms/tick */
#define CHASSIS_SHARP_TURN_CONFIRM_TICKS     2

typedef enum {
    CHASSIS_LOCKED = 0,
    CHASSIS_IDLE,
    CHASSIS_LINE,
    CHASSIS_FIND_LINE,
    CHASSIS_FINISHED,
    CHASSIS_ERROR,
    CHASSIS_LINE_LOST
} chassis_state_t;

typedef enum {
    CHASSIS_TARGET_A = 0,
    CHASSIS_TARGET_B,
    CHASSIS_TARGET_C,
    CHASSIS_TARGET_D
} chassis_target_t;

typedef struct {
    int error;
    int line_detected;
    int all_white;
    int active_count;
    int center_hit;
    int turn_hint;
    int stale;
} chassis_line_input_t;

static pid_t            line_pid;
static chassis_state_t  chassis_state = CHASSIS_LOCKED;
static chassis_target_t chassis_target = CHASSIS_TARGET_C;
static TargetPoint_t    external_target = TARGET_NONE;
static int              chassis_needs_pid_reset = 0;
static int              last_line_error = 0;
static int              last_line_correction = 0;
static int              last_base_speed = 0;
static int              lost_count = 0;
static int              sharp_left_active = 0;
static int              sharp_right_active = 0;
static int              sharp_turn_ticks = 0;
static int              sharp_left_confirm = 0;
static int              sharp_right_confirm = 0;

#if CHASSIS_USE_ENCODER && CHASSIS_USE_SPEED_PID
static pid_t speed_pid_left;
static pid_t speed_pid_right;
static int speed_last_encoder_left = 0;
static int speed_last_encoder_right = 0;
static int speed_target_left_x100 = 0;
static int speed_target_right_x100 = 0;
static int speed_measured_left_x100 = 0;
static int speed_measured_right_x100 = 0;
static int speed_pwm_left = 0;
static int speed_pwm_right = 0;
#endif

#if CHASSIS_USE_ENCODER
#define ENCODER_LEFT_POLARITY   1
#define ENCODER_RIGHT_POLARITY -1

static volatile int encoder_left_count = 0;
static volatile int encoder_right_count = 0;
#endif

static int clamp_speed(int speed)
{
    RunMode_t mode = app_state_get_mode();
    int max_speed = P2_MAX_SPEED;

    if (mode == RUN_MODE_PROBLEM4) {
        max_speed = P4_MAX_SPEED;
    } else if (mode == RUN_MODE_PROBLEM56) {
        max_speed = P56_MAX_SPEED;
    }

    if (speed > max_speed) return max_speed;
    if (speed < -max_speed) return -max_speed;
    return speed;
}

/* Convert logical wheel commands to the physical motor channels.  With the
 * tracking-module side used as the front, the chassis frame is rotated 180°:
 * logical left/right become physical right/left and both directions invert. */
static void chassis_set_logical_wheel_speed(int left, int right)
{
#if CHASSIS_REVERSE_FORWARD_DIRECTION
    motor_set_speed(-right, -left);
#else
    motor_set_speed(left, right);
#endif
}

static int clamp_symmetric(int value, int limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static int abs_int(int value)
{
    return (value < 0) ? -value : value;
}

static ChassisStatus_t chassis_map_state_to_status(chassis_state_t state)
{
    switch (state) {
        case CHASSIS_LOCKED:
        case CHASSIS_IDLE:      return CHASSIS_STATUS_IDLE;
        case CHASSIS_FIND_LINE: return CHASSIS_STATUS_FINDING_LINE;
        case CHASSIS_LINE:      return CHASSIS_STATUS_FOLLOWING;
        case CHASSIS_FINISHED:  return CHASSIS_STATUS_TARGET_REACHED;
        case CHASSIS_LINE_LOST: return CHASSIS_STATUS_LINE_LOST;
        case CHASSIS_ERROR:     return CHASSIS_STATUS_ERROR;
        default:                return CHASSIS_STATUS_IDLE;
    }
}

static chassis_target_t chassis_convert_target(TargetPoint_t target)
{
    switch (target) {
        case TARGET_A: return CHASSIS_TARGET_A;
        case TARGET_B: return CHASSIS_TARGET_B;
        case TARGET_C: return CHASSIS_TARGET_C;
        case TARGET_D: return CHASSIS_TARGET_D;
        case TARGET_NONE:
        default:       return CHASSIS_TARGET_C;
    }
}

#if CHASSIS_EVENT_LOG_ENABLE
static void debug_uart_send_char(char c)
{
    while (DL_UART_isBusy(DEBUG_UART_INST)) {}
    DL_UART_transmitData(DEBUG_UART_INST, (uint8_t)c);
}

static void debug_uart_send_str(const char *s)
{
    while (*s) {
        debug_uart_send_char(*s++);
    }
}

static const char *chassis_state_name(chassis_state_t state)
{
    switch (state) {
        case CHASSIS_LOCKED:    return "LOCK";
        case CHASSIS_IDLE:      return "IDLE";
        case CHASSIS_LINE:      return "LINE";
        case CHASSIS_FIND_LINE: return "FIND";
        case CHASSIS_FINISHED:  return "FIN";
        case CHASSIS_ERROR:     return "ERR";
        case CHASSIS_LINE_LOST: return "LOST";
        default:                return "UNK";
    }
}

static const char *chassis_status_name(ChassisStatus_t status)
{
    switch (status) {
        case CHASSIS_STATUS_IDLE:           return "IDLE";
        case CHASSIS_STATUS_FINDING_LINE:   return "FLIN";
        case CHASSIS_STATUS_FOLLOWING:      return "FOLL";
        case CHASSIS_STATUS_TARGET_REACHED: return "RCHD";
        case CHASSIS_STATUS_LINE_LOST:      return "LOST";
        case CHASSIS_STATUS_ERROR:          return "ERR";
        default:                            return "UNK";
    }
}

static const char *chassis_mode_name(RunMode_t mode)
{
    switch (mode) {
        case RUN_MODE_BASIC_DRIVE: return "DRV";
        case RUN_MODE_BASIC_NAV:   return "NAV";
        case RUN_MODE_ADVANCED_NAV:return "ADV";
        case RUN_MODE_PROBLEM3:    return "P3";
        case RUN_MODE_PROBLEM4:    return "P4";
        case RUN_MODE_PROBLEM56:   return "P56";
        default:                   return "?";
    }
}
#endif

static void chassis_debug_log(const chassis_line_input_t *line)
{
#if CHASSIS_EVENT_LOG_ENABLE
    static chassis_state_t last_state = (chassis_state_t)0xFF;
    static ChassisStatus_t last_status = (ChassisStatus_t)0xFF;
    RunMode_t mode = app_state_get_mode();
    ChassisStatus_t status = chassis_map_state_to_status(chassis_state);
    char buf[160];

    if (!(chassis_state != last_state || status != last_status ||
          line->stale || status == CHASSIS_STATUS_LINE_LOST ||
          status == CHASSIS_STATUS_ERROR)) {
        return;
    }

    last_state = chassis_state;
    last_status = status;

    (void)snprintf(buf, sizeof(buf),
        "CH: st=%s ext=%s mode=%s err=%d line=%d white=%d active=%d stale=%d lost=%d tgt=%d lock=%d\r\n",
        chassis_state_name(chassis_state),
        chassis_status_name(status),
        chassis_mode_name(mode),
        line->error,
        line->line_detected,
        line->all_white,
        line->active_count,
        line->stale,
        lost_count,
        (int)chassis_target,
        motor_is_locked());
    debug_uart_send_str(buf);
#else
    (void)line;
#endif
}

static chassis_line_input_t chassis_read_line_input(void)
{
    chassis_line_input_t line = {0};
    const TrackBridgeData_t *bridge = track_bridge_get();

    if (bridge == 0) {
        line.stale = 1;
        line.all_white = 1;
        return line;
    }

    line.error = bridge->error * CHASSIS_TRACK_ERROR_SCALE;
    line.line_detected = bridge->line_detected ? 1 : 0;
    line.all_white = bridge->all_white ? 1 : 0;
    line.active_count = bridge->active_count;
    line.center_hit = bridge->center_hit ? 1 : 0;
    line.turn_hint = bridge->turn_hint;
    line.stale = bridge->stale ? 1 : 0;
    return line;
}

static void speed_pid_reset(void)
{
#if CHASSIS_USE_ENCODER && CHASSIS_USE_SPEED_PID
    int encoder_left_now;
    int encoder_right_now;

    pid_reset(&speed_pid_left);
    pid_reset(&speed_pid_right);
    chassis_encoder_get_counts(&encoder_left_now, &encoder_right_now);
    speed_last_encoder_left = encoder_left_now;
    speed_last_encoder_right = encoder_right_now;
    speed_target_left_x100 = 0;
    speed_target_right_x100 = 0;
    speed_measured_left_x100 = 0;
    speed_measured_right_x100 = 0;
    speed_pwm_left = 0;
    speed_pwm_right = 0;
#endif
}

#if CHASSIS_USE_ENCODER && CHASSIS_USE_SPEED_PID
static int speed_command_to_target_x100(int command)
{
    return command * CHASSIS_SPEED_COUNTS_PER_CMD_X100;
}
#endif

static void chassis_apply_speed(int left, int right)
{
    left = clamp_speed(left);
    right = clamp_speed(right);

#if CHASSIS_USE_ENCODER && CHASSIS_USE_SPEED_PID
    {
        int encoder_left_now;
        int encoder_right_now;
        int correction_left;
        int correction_right;

        chassis_encoder_get_counts(&encoder_left_now, &encoder_right_now);

        speed_measured_left_x100 =
            (encoder_left_now - speed_last_encoder_left) * 100;
        speed_measured_right_x100 =
            (encoder_right_now - speed_last_encoder_right) * 100;
        speed_last_encoder_left = encoder_left_now;
        speed_last_encoder_right = encoder_right_now;

        speed_target_left_x100 = speed_command_to_target_x100(left);
        speed_target_right_x100 = speed_command_to_target_x100(right);

        if (left == 0) {
            pid_reset(&speed_pid_left);
            correction_left = 0;
        } else {
            correction_left = pid_update(
                &speed_pid_left,
                speed_target_left_x100 - speed_measured_left_x100);
        }

        if (right == 0) {
            pid_reset(&speed_pid_right);
            correction_right = 0;
        } else {
            correction_right = pid_update(
                &speed_pid_right,
                speed_target_right_x100 - speed_measured_right_x100);
        }

        speed_pwm_left = clamp_speed(left + correction_left);
        speed_pwm_right = clamp_speed(right + correction_right);
        chassis_set_logical_wheel_speed(speed_pwm_left, speed_pwm_right);
    }
#else
    chassis_set_logical_wheel_speed(left, right);
#endif
}

static void chassis_reset_tracking(void)
{
    pid_reset(&line_pid);
    lost_count = 0;
    last_line_error = 0;
    last_line_correction = 0;
    last_base_speed = 0;
    sharp_left_active = 0;
    sharp_right_active = 0;
    sharp_turn_ticks = 0;
    sharp_left_confirm = 0;
    sharp_right_confirm = 0;
}

static int chassis_select_base_speed(const chassis_line_input_t *line)
{
    RunMode_t mode = app_state_get_mode();
    int find_speed = P2_FIND_SPEED;
    int curve_error = P2_CURVE_ERROR;
    int curve_speed = P2_CURVE_SPEED;
    int straight_speed = P2_STRAIGHT_SPEED;
    int base_speed = P2_BASE_SPEED;

    if (mode == RUN_MODE_PROBLEM4) {
        find_speed = P4_FIND_SPEED;
        curve_error = P4_CURVE_ERROR;
        curve_speed = P4_CURVE_SPEED;
        straight_speed = P4_STRAIGHT_SPEED;
        base_speed = P4_BASE_SPEED;
    } else if (mode == RUN_MODE_PROBLEM56) {
        find_speed = P56_FIND_SPEED;
        curve_error = P56_CURVE_ERROR;
        curve_speed = P56_CURVE_SPEED;
        straight_speed = P56_STRAIGHT_SPEED;
        base_speed = P56_BASE_SPEED;
    }

    if (!line->line_detected || line->all_white) {
        return find_speed;
    }

    if (abs_int(line->error) >= curve_error) {
        return curve_speed;
    }

    if (line->center_hit && abs_int(line->error) <= 10 &&
        line->turn_hint == LINE_TRACK_TURN_HINT_NONE) {
        return straight_speed;
    }

    return base_speed;
}

#if CHASSIS_USE_ENCODER
static void encoder_get_snapshot(int *left, int *right)
{
    uint32_t primask = __get_PRIMASK();
    int physical_left;
    int physical_right;

    __disable_irq();
    physical_left = encoder_left_count;
    physical_right = encoder_right_count;
    if (primask == 0U) {
        __enable_irq();
    }

#if CHASSIS_REVERSE_FORWARD_DIRECTION
    if (left) *left = -physical_right;
    if (right) *right = -physical_left;
#else
    if (left) *left = physical_left;
    if (right) *right = physical_right;
#endif
}

static void encoder_reset_counts(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    encoder_left_count = 0;
    encoder_right_count = 0;
    DL_GPIO_clearInterruptStatus(GPIOB,
                                 GPIO_ENCODER_LEFT_A_PIN |
                                 GPIO_ENCODER_RIGHT_A_PIN);
    if (primask == 0U) {
        __enable_irq();
    }
}
#else
static void encoder_reset_counts(void) {}
#endif

static int chassis_finish_slowdown_active(void)
{
#if CHASSIS_USE_ENCODER
    int left_count;
    int right_count;
    int distance_count;

    encoder_get_snapshot(&left_count, &right_count);
    if (left_count < 0) left_count = -left_count;
    if (right_count < 0) right_count = -right_count;

    distance_count = (left_count + right_count) / 2;
    if (app_state_get_mode() == RUN_MODE_PROBLEM56) {
        return distance_count >= P56_FINISH_SLOWDOWN_COUNTS;
    }

    return distance_count >= P2_FINISH_SLOWDOWN_COUNTS;
#else
    return 0;
#endif
}

static int chassis_p4_ab_slowdown_active(void)
{
#if CHASSIS_USE_ENCODER
    int left_count;
    int right_count;
    int distance_count;

    encoder_get_snapshot(&left_count, &right_count);
    if (left_count < 0) left_count = -left_count;
    if (right_count < 0) right_count = -right_count;

    distance_count = (left_count + right_count) / 2;
    return distance_count >= P4_AB_SLOWDOWN_COUNTS;
#else
    return 0;
#endif
}

static void chassis_load_line_pid_for_mode(void)
{
    if (app_state_get_mode() == RUN_MODE_PROBLEM4) {
        pid_init(&line_pid,
                 P4_LINE_PID_KP,
                 P4_LINE_PID_KI,
                 P4_LINE_PID_KD,
                 P4_TURN_LIMIT,
                 1000);
    } else if (app_state_get_mode() == RUN_MODE_PROBLEM56) {
        pid_init(&line_pid,
                 P56_LINE_PID_KP,
                 P56_LINE_PID_KI,
                 P56_LINE_PID_KD,
                 P56_TURN_LIMIT,
                 1000);
    } else {
        pid_init(&line_pid,
                 P2_LINE_PID_KP,
                 P2_LINE_PID_KI,
                 P2_LINE_PID_KD,
                 P2_TURN_LIMIT,
                 1000);
    }
}

void GROUP1_IRQHandler(void)
{
#if CHASSIS_USE_ENCODER
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(
        GPIOB, GPIO_ENCODER_LEFT_A_PIN | GPIO_ENCODER_RIGHT_A_PIN);

    if ((pending & GPIO_ENCODER_LEFT_A_PIN) != 0U) {
        int a = DL_GPIO_readPins(GPIO_ENCODER_LEFT_A_PORT,
                                 GPIO_ENCODER_LEFT_A_PIN) != 0U;
        int b = DL_GPIO_readPins(GPIO_ENCODER_LEFT_B_PORT,
                                 GPIO_ENCODER_LEFT_B_PIN) != 0U;

        encoder_left_count += ((a == b) ? 1 : -1) * ENCODER_LEFT_POLARITY;
        DL_GPIO_clearInterruptStatus(GPIOB, GPIO_ENCODER_LEFT_A_PIN);
    }

    if ((pending & GPIO_ENCODER_RIGHT_A_PIN) != 0U) {
        int a = DL_GPIO_readPins(GPIO_ENCODER_RIGHT_A_PORT,
                                 GPIO_ENCODER_RIGHT_A_PIN) != 0U;
        int b = DL_GPIO_readPins(GPIO_ENCODER_RIGHT_B_PORT,
                                 GPIO_ENCODER_RIGHT_B_PIN) != 0U;

        encoder_right_count += ((a == b) ? 1 : -1) * ENCODER_RIGHT_POLARITY;
        DL_GPIO_clearInterruptStatus(GPIOB, GPIO_ENCODER_RIGHT_A_PIN);
    }
#endif
}

void chassis_init(void)
{
    motor_init();
    track_bridge_init();
    track_bridge_mark_stale();
    chassis_load_line_pid_for_mode();
#if CHASSIS_USE_ENCODER && CHASSIS_USE_SPEED_PID
    pid_init(&speed_pid_left,
             CHASSIS_SPEED_PID_KP,
             CHASSIS_SPEED_PID_KI,
             CHASSIS_SPEED_PID_KD,
             CHASSIS_SPEED_PID_CORRECTION_LIMIT,
             CHASSIS_SPEED_PID_INTEGRAL_LIMIT);
    pid_init(&speed_pid_right,
             CHASSIS_SPEED_PID_KP,
             CHASSIS_SPEED_PID_KI,
             CHASSIS_SPEED_PID_KD,
             CHASSIS_SPEED_PID_CORRECTION_LIMIT,
             CHASSIS_SPEED_PID_INTEGRAL_LIMIT);
#endif
    chassis_state = CHASSIS_IDLE;
    chassis_needs_pid_reset = 0;
    encoder_reset_counts();
#if CHASSIS_USE_ENCODER
    NVIC_ClearPendingIRQ(GPIO_ENCODER_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_INT_IRQN);
#endif
    speed_pid_reset();
    chassis_reset_tracking();
}

void chassis_set_target(int target)
{
    if (target < (int)CHASSIS_TARGET_A || target > (int)CHASSIS_TARGET_D) {
        chassis_target = CHASSIS_TARGET_C;
    } else {
        chassis_target = (chassis_target_t)target;
    }

    encoder_reset_counts();
    speed_pid_reset();
    chassis_reset_tracking();
}

void chassis_unlock(void)
{
    motor_unlock();
    if (chassis_state == CHASSIS_LOCKED) {
        chassis_state = CHASSIS_IDLE;
    }
}

void chassis_lock(void)
{
    speed_pid_reset();
    motor_lock();
    chassis_state = CHASSIS_LOCKED;
    chassis_needs_pid_reset = 1;
}

void chassis_emergency_stop(void)
{
    speed_pid_reset();
    motor_lock();
    chassis_state = CHASSIS_LOCKED;
    chassis_needs_pid_reset = 1;
    chassis_reset_tracking();
}

void chassis_brake(void)
{
    speed_pid_reset();
    motor_brake();
    chassis_needs_pid_reset = 1;
    last_line_correction = 0;
}

void chassis_stop(void)
{
    speed_pid_reset();
    motor_coast();
    chassis_state = CHASSIS_IDLE;
    chassis_needs_pid_reset = 1;
    chassis_reset_tracking();
}

void chassis_drive(int forward, int turn)
{
    int left;
    int right;

    if (motor_is_locked()) {
        return;
    }

    left = clamp_speed(forward + turn);
    right = clamp_speed(forward - turn);
    chassis_apply_speed(left, right);
    chassis_state = CHASSIS_IDLE;
}

void chassis_forward(int speed)
{
    chassis_drive(clamp_speed(speed), 0);
}

void chassis_backward(int speed)
{
    if (speed < 0) speed = -speed;
    chassis_drive(-clamp_speed(speed), 0);
}

void chassis_turn_left(int speed)
{
    if (speed < 0) speed = -speed;
    chassis_drive(speed / 2, -speed / 2);
}

void chassis_turn_right(int speed)
{
    if (speed < 0) speed = -speed;
    chassis_drive(speed / 2, speed / 2);
}

void chassis_rotate_left(int speed)
{
    if (speed < 0) speed = -speed;
    chassis_drive(0, -speed);
}

void chassis_rotate_right(int speed)
{
    if (speed < 0) speed = -speed;
    chassis_drive(0, speed);
}

void chassis_find_line(void)
{
    if (motor_is_locked()) {
        return;
    }

    if (last_line_error < 0) {
        RunMode_t mode = app_state_get_mode();
        int find_speed = P2_FIND_SPEED;
        if (mode == RUN_MODE_PROBLEM4) {
            find_speed = P4_FIND_SPEED;
        } else if (mode == RUN_MODE_PROBLEM56) {
            find_speed = P56_FIND_SPEED;
        }
        chassis_rotate_left(find_speed);
    } else {
        RunMode_t mode = app_state_get_mode();
        int find_speed = P2_FIND_SPEED;
        if (mode == RUN_MODE_PROBLEM4) {
            find_speed = P4_FIND_SPEED;
        } else if (mode == RUN_MODE_PROBLEM56) {
            find_speed = P56_FIND_SPEED;
        }
        chassis_rotate_right(find_speed);
    }
    chassis_state = CHASSIS_FIND_LINE;
}

void chassis_follow_target(TargetPoint_t target)
{
    if (target == TARGET_NONE) {
        chassis_state = CHASSIS_ERROR;
        return;
    }

    external_target = target;
    chassis_target = chassis_convert_target(target);
    chassis_load_line_pid_for_mode();
    encoder_reset_counts();
    speed_pid_reset();
    chassis_reset_tracking();
    chassis_needs_pid_reset = 1;

    if (!motor_is_locked()) {
        chassis_state = CHASSIS_LINE;
    }
}

void chassis_run_line(void)
{
    chassis_line_input_t line = chassis_read_line_input();
    int correction;
    int base_speed;
    int finish_slowdown;
    int p4_slowdown;
    int turn_limit;
    int accel_step;
    int decel_step;
    int finish_slowdown_speed;
    int left;
    int right;

    if (motor_is_locked() || chassis_state == CHASSIS_FINISHED) {
        return;
    }

    if (chassis_needs_pid_reset) {
        chassis_needs_pid_reset = 0;
        pid_reset(&line_pid);
    }

    if (line.stale) {
        speed_pid_reset();
        motor_brake();
        chassis_state = CHASSIS_ERROR;
        chassis_debug_log(&line);
        return;
    }

    /* A wide black area on the left represents a 90-degree left corner.
     * Latch the manoeuvre so a short all-white gap during the pivot does not
     * make the chassis brake before the new line enters the centre sensors. */
    if (CHASSIS_ENABLE_SHARP_TURN &&
        !sharp_left_active && !sharp_right_active &&
        line.line_detected && !line.all_white) {
        if (line.turn_hint == LINE_TRACK_TURN_HINT_LEFT) {
            sharp_left_confirm++;
            sharp_right_confirm = 0;
            if (sharp_left_confirm >= CHASSIS_SHARP_TURN_CONFIRM_TICKS) {
                sharp_left_active = 1;
                sharp_turn_ticks = 0;
                sharp_left_confirm = 0;
                pid_reset(&line_pid);
                last_line_correction = 0;
            }
        } else if (line.turn_hint == LINE_TRACK_TURN_HINT_RIGHT) {
            sharp_right_confirm++;
            sharp_left_confirm = 0;
            if (sharp_right_confirm >= CHASSIS_SHARP_TURN_CONFIRM_TICKS) {
                sharp_right_active = 1;
                sharp_turn_ticks = 0;
                sharp_right_confirm = 0;
                pid_reset(&line_pid);
                last_line_correction = 0;
            }
        } else {
            sharp_left_confirm = 0;
            sharp_right_confirm = 0;
        }
    } else if (!sharp_left_active && !sharp_right_active) {
        sharp_left_confirm = 0;
        sharp_right_confirm = 0;
    }

    if (sharp_left_active) {
        if (sharp_turn_ticks > 0 &&
            line.line_detected && !line.all_white &&
            line.center_hit && abs_int(line.error) <= LINE_TRACK_CENTER_HIT_THRESHOLD) {
            sharp_left_active = 0;
            sharp_turn_ticks = 0;
            lost_count = 0;
            pid_reset(&line_pid);
        } else if (sharp_turn_ticks < CHASSIS_SHARP_TURN_TIMEOUT_TICKS) {
            if (line.line_detected && !line.all_white &&
                abs_int(line.error) <= CHASSIS_SHARP_APPROACH_ERROR) {
                chassis_apply_speed(CHASSIS_SHARP_APPROACH_INNER_SPEED,
                                    CHASSIS_SHARP_APPROACH_OUTER_SPEED);
            } else {
                chassis_apply_speed(CHASSIS_SHARP_LEFT_INNER_SPEED,
                                    CHASSIS_SHARP_LEFT_OUTER_SPEED);
            }
            sharp_turn_ticks++;
            chassis_state = CHASSIS_LINE;
            chassis_debug_log(&line);
            return;
        } else {
            sharp_left_active = 0;
            sharp_turn_ticks = 0;
        }
    }


    if (sharp_right_active) {
        if (sharp_turn_ticks > 0 &&
            line.line_detected && !line.all_white &&
            line.center_hit && abs_int(line.error) <= LINE_TRACK_CENTER_HIT_THRESHOLD) {
            sharp_right_active = 0;
            sharp_turn_ticks = 0;
            lost_count = 0;
            pid_reset(&line_pid);
        } else if (sharp_turn_ticks < CHASSIS_SHARP_TURN_TIMEOUT_TICKS) {
            if (line.line_detected && !line.all_white &&
                abs_int(line.error) <= CHASSIS_SHARP_APPROACH_ERROR) {
                chassis_apply_speed(CHASSIS_SHARP_APPROACH_OUTER_SPEED,
                                    CHASSIS_SHARP_APPROACH_INNER_SPEED);
            } else {
                chassis_apply_speed(CHASSIS_SHARP_RIGHT_OUTER_SPEED,
                                    CHASSIS_SHARP_RIGHT_INNER_SPEED);
            }
            sharp_turn_ticks++;
            chassis_state = CHASSIS_LINE;
            chassis_debug_log(&line);
            return;
        } else {
            sharp_right_active = 0;
            sharp_turn_ticks = 0;
        }
    }

    if (!line.line_detected || line.all_white) {
        lost_count++;
        if (lost_count == 1) {
            pid_reset(&line_pid);
            speed_pid_reset();
            last_line_correction = 0;
        }

        /* Keep searching indefinitely in the direction of the last observed
         * line error.  Reacquisition below resets the tracking PID and
         * resumes normal line following.  A second start-key press remains
         * the operator stop mechanism. */
        chassis_find_line();
        chassis_debug_log(&line);
        return;
    }

    lost_count = 0;
    if (chassis_state == CHASSIS_FIND_LINE) {
        pid_reset(&line_pid);
    }

    correction = pid_update(&line_pid, line.error);

    turn_limit = P2_TURN_LIMIT;
    accel_step = P2_LINE_ACCEL_STEP;
    decel_step = P2_LINE_DECEL_STEP;
    finish_slowdown_speed = P2_FINISH_SLOWDOWN_SPEED;
    if (app_state_get_mode() == RUN_MODE_PROBLEM4) {
        turn_limit = P4_TURN_LIMIT;
        accel_step = P4_LINE_ACCEL_STEP;
        decel_step = P4_LINE_DECEL_STEP;
    } else if (app_state_get_mode() == RUN_MODE_PROBLEM56) {
        turn_limit = P56_TURN_LIMIT;
        accel_step = P56_LINE_ACCEL_STEP;
        decel_step = P56_LINE_DECEL_STEP;
        finish_slowdown_speed = P56_FINISH_SLOWDOWN_SPEED;
    }

    correction = clamp_symmetric(correction, turn_limit);

    /* A centred, ordinary line segment does not need aggressive steering.
     * Sensor noise and the PID derivative term can otherwise make the car
     * hunt from side to side even though the line is already under the
     * centre probes.  Keep this dead-zone deliberately tight; turn hints
     * and larger errors still use the normal turn limit. */
    if (line.turn_hint == LINE_TRACK_TURN_HINT_NONE &&
        line.center_hit && line.active_count <= 4U) {
        correction = clamp_symmetric(correction, 3);
    }

    /* Keep enough damping for sensor noise without adding a long steering
     * delay, which otherwise makes the chassis repeatedly overshoot centre. */
    correction = (last_line_correction * 3 + correction) / 4;
    if (line.turn_hint == LINE_TRACK_TURN_HINT_NONE &&
        line.center_hit && line.active_count <= 4U) {
        correction = clamp_symmetric(correction, 3);
    }

    last_line_error = line.error;
    last_line_correction = correction;

    base_speed = chassis_select_base_speed(&line);
    finish_slowdown = chassis_finish_slowdown_active();
    p4_slowdown = (app_state_get_mode() == RUN_MODE_PROBLEM4) ?
                  chassis_p4_ab_slowdown_active() : 0;
    if (app_state_get_mode() != RUN_MODE_PROBLEM4 &&
        finish_slowdown && base_speed > finish_slowdown_speed) {
        base_speed = finish_slowdown_speed;
    } else if (p4_slowdown && base_speed > P4_AB_SLOWDOWN_SPEED) {
        base_speed = P4_AB_SLOWDOWN_SPEED;
    }
    if (last_base_speed != 0 &&
        base_speed > last_base_speed + accel_step) {
        base_speed = last_base_speed + accel_step;
    } else if (!finish_slowdown && !p4_slowdown &&
               last_base_speed != 0 &&
               base_speed < last_base_speed - decel_step) {
        base_speed = last_base_speed - decel_step;
    }
    last_base_speed = base_speed;

    left = base_speed + correction;
    right = base_speed - correction;
    chassis_apply_speed(left, right);
    chassis_state = CHASSIS_LINE;
    chassis_debug_log(&line);
}

void chassis_encoder_poll(void)
{
    /* Encoder edges are captured by GROUP1_IRQHandler(). */
}

void chassis_encoder_reset(void)
{
    encoder_reset_counts();
    speed_pid_reset();
}

void chassis_encoder_get_counts(int *left, int *right)
{
#if CHASSIS_USE_ENCODER
    encoder_get_snapshot(left, right);
#else
    if (left) *left = 0;
    if (right) *right = 0;
#endif
}

int chassis_encoder_get_left_count(void)
{
#if CHASSIS_USE_ENCODER
    int left;
    encoder_get_snapshot(&left, 0);
    return left;
#else
    return 0;
#endif
}

int chassis_encoder_get_right_count(void)
{
#if CHASSIS_USE_ENCODER
    int right;
    encoder_get_snapshot(0, &right);
    return right;
#else
    return 0;
#endif
}

void chassis_speed_pid_get_debug(int *target_left_x100,
                                 int *target_right_x100,
                                 int *measured_left_x100,
                                 int *measured_right_x100,
                                 int *pwm_left,
                                 int *pwm_right)
{
#if CHASSIS_USE_ENCODER && CHASSIS_USE_SPEED_PID
    if (target_left_x100) *target_left_x100 = speed_target_left_x100;
    if (target_right_x100) *target_right_x100 = speed_target_right_x100;
    if (measured_left_x100) *measured_left_x100 = speed_measured_left_x100;
    if (measured_right_x100) *measured_right_x100 = speed_measured_right_x100;
    if (pwm_left) *pwm_left = speed_pwm_left;
    if (pwm_right) *pwm_right = speed_pwm_right;
#else
    if (target_left_x100) *target_left_x100 = 0;
    if (target_right_x100) *target_right_x100 = 0;
    if (measured_left_x100) *measured_left_x100 = 0;
    if (measured_right_x100) *measured_right_x100 = 0;
    if (pwm_left) *pwm_left = 0;
    if (pwm_right) *pwm_right = 0;
#endif
}

void chassis_tick(void)
{
    chassis_run_line();
}

ChassisStatus_t chassis_get_status(void)
{
    return chassis_map_state_to_status(chassis_state);
}

TargetPoint_t chassis_get_target(void)
{
    return external_target;
}

int chassis_get_state(void)
{
    return (int)chassis_state;
}

int chassis_is_finished(void)
{
    return chassis_state == CHASSIS_FINISHED;
}

void chassis_follow_target_ex(int target)
{
    chassis_set_target(target);
    chassis_run_line();
}

void chassis_debug_simulate_line_lost(void)
{
    if (chassis_state == CHASSIS_FIND_LINE ||
        chassis_state == CHASSIS_LINE) {
        chassis_state = CHASSIS_LINE_LOST;
    }
}

void chassis_debug_simulate_error(void)
{
    chassis_state = CHASSIS_ERROR;
}
