#include "chassis_iface.h"
#include "app_state.h"
#include "line_track.h"
#include "motor.h"
#include "pid.h"
#include "ti_msp_dl_config.h"
#include "track_bridge.h"
#include <stdio.h>

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

#define CHASSIS_BASE_SPEED                  16
#define CHASSIS_CURVE_SPEED                 10
#define CHASSIS_FIND_SPEED                  30
#define CHASSIS_MAX_SPEED                   45
#define CHASSIS_TURN_LIMIT                  30
#define CHASSIS_CURVE_ERROR                 45
#define CHASSIS_CROSS_CONFIRM                4
#define CHASSIS_ENABLE_TARGET_STOP           0
#define CHASSIS_TURN_HINT_CONFIRM_TICKS      1
#define CHASSIS_TURN_PRIORITY_TICKS         42
#define CHASSIS_TURN_PRIORITY_MIN_TICKS     25
#define CHASSIS_TURN_PRIORITY_SPEED         36
#define CHASSIS_TURN_COOLDOWN_TICKS         100

typedef enum {
    CHASSIS_LOCKED = 0,
    CHASSIS_IDLE,
    CHASSIS_LINE,
    CHASSIS_FIND_LINE,
    CHASSIS_CROSS,
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

typedef enum {
    ROUTE_ACTION_RIGHT = 0,
    ROUTE_ACTION_LEFT,
    ROUTE_ACTION_STRAIGHT,
    ROUTE_ACTION_STOP
} route_action_t;

/* DRV route: A to C (basic drive, field-verified).
   Keep separate from NAV/ADV routes to avoid mode cross-talk. */
static const route_action_t route_drv_a_to_c[] = {
    ROUTE_ACTION_RIGHT,     /* node 1 */
    ROUTE_ACTION_LEFT,      /* node 2 */
    ROUTE_ACTION_STRAIGHT,  /* node 3 */
    ROUTE_ACTION_RIGHT,     /* node 4 */
    ROUTE_ACTION_LEFT,      /* node 5 */
    ROUTE_ACTION_RIGHT,     /* node 6 */
    ROUTE_ACTION_STOP       /* node 7: arrive at C */
};

/* NAV/ADV route: center to A (field-verified).
   Shared by BASIC_NAV and ADVANCED_NAV when target is A.
   Keep separate from DRV A-to-C route to avoid mode cross-talk. */
static const route_action_t route_nav_center_to_a[] = {
    ROUTE_ACTION_RIGHT,     /* node 1 */
    ROUTE_ACTION_LEFT,      /* node 2 */
    ROUTE_ACTION_LEFT,      /* node 3 */
    ROUTE_ACTION_RIGHT,     /* node 4 */
    ROUTE_ACTION_LEFT,      /* node 5 */
    ROUTE_ACTION_STOP       /* node 6: arrive at A */
};

/* NAV/ADV route: center to D (field-verified).
   First four actions match center-to-A; node 5 turns right to D. */
static const route_action_t route_nav_center_to_d[] = {
    ROUTE_ACTION_RIGHT,     /* node 1 */
    ROUTE_ACTION_LEFT,      /* node 2 */
    ROUTE_ACTION_LEFT,      /* node 3 */
    ROUTE_ACTION_RIGHT,     /* node 4 */
    ROUTE_ACTION_RIGHT,     /* node 5 */
    ROUTE_ACTION_STOP       /* node 6: arrive at D */
};

/*
 * NAV/ADV center to B, C: NOT YET CONFIRMED.
 * No route tables exist for these targets.
 * Calling code MUST treat the absence of a route as an error,
 * not fall back to free line following.
 */

#define ARRAY_LEN(a) ((int)(sizeof(a) / sizeof((a)[0])))

/* Select route table by (mode, target).
   Returns NULL with *length=0 when no route is defined.
   Only confirmed combinations:
     DRV + TARGET_C           -> route_drv_a_to_c
     BASIC_NAV/ADVANCED_NAV + TARGET_A -> route_nav_center_to_a
     BASIC_NAV/ADVANCED_NAV + TARGET_D -> route_nav_center_to_d */
static const route_action_t *chassis_route_lookup(RunMode_t mode,
                                                  TargetPoint_t target,
                                                  int *length)
{
    const route_action_t *table = NULL;
    int len = 0;

    if (mode == RUN_MODE_BASIC_DRIVE) {
        if (target == TARGET_C) {
            table = route_drv_a_to_c;
            len   = ARRAY_LEN(route_drv_a_to_c);
        }
    } else if (mode == RUN_MODE_BASIC_NAV ||
               mode == RUN_MODE_ADVANCED_NAV) {
        if (target == TARGET_A) {
            table = route_nav_center_to_a;
            len   = ARRAY_LEN(route_nav_center_to_a);
        } else if (target == TARGET_D) {
            table = route_nav_center_to_d;
            len   = ARRAY_LEN(route_nav_center_to_d);
        }
        /* B/C not confirmed -- table stays NULL */
    }

    if (length != NULL) {
        *length = len;
    }
    return table;
}

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
static int              lost_count = 0;
static int              cross_count = 0;
static int              cross_latched = 0;
static int              node_count = 0;
static int              turn_hint_count = 0;
static int              turn_priority_ticks = 0;
static int              turn_priority_dir = 0;
static int              turn_cooldown_ticks = 0;

static const route_action_t *route_table = 0;
static int              route_length = 0;
static int              route_index = 0;
static int              route_active = 0;
static int              route_straight_ticks = 0;

static int clamp_speed(int speed)
{
    if (speed > CHASSIS_MAX_SPEED) return CHASSIS_MAX_SPEED;
    if (speed < -CHASSIS_MAX_SPEED) return -CHASSIS_MAX_SPEED;
    return speed;
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
        case CHASSIS_LINE:
        case CHASSIS_CROSS:     return CHASSIS_STATUS_FOLLOWING;
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
        case CHASSIS_CROSS:     return "CROSS";
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
        default:                   return "?";
    }
}

static void chassis_debug_log(const chassis_line_input_t *line)
{
    static chassis_state_t last_state = (chassis_state_t)0xFF;
    static ChassisStatus_t last_status = (ChassisStatus_t)0xFF;
    static int last_route_index = -1;
    RunMode_t mode = app_state_get_mode();
    ChassisStatus_t status = chassis_map_state_to_status(chassis_state);
    char buf[160];

    if (!(chassis_state != last_state || status != last_status ||
          line->stale || status == CHASSIS_STATUS_LINE_LOST ||
          status == CHASSIS_STATUS_ERROR ||
          (route_active && route_index != last_route_index))) {
        return;
    }

    last_state = chassis_state;
    last_status = status;
    last_route_index = route_index;

    (void)snprintf(buf, sizeof(buf),
        "CH: st=%s ext=%s mode=%s err=%d line=%d white=%d active=%d stale=%d lost=%d node=%d tgt=%d ri=%d/%d lock=%d\r\n",
        chassis_state_name(chassis_state),
        chassis_status_name(status),
        chassis_mode_name(mode),
        line->error,
        line->line_detected,
        line->all_white,
        line->active_count,
        line->stale,
        lost_count,
        node_count,
        (int)chassis_target,
        route_index,
        route_length,
        motor_is_locked());
    debug_uart_send_str(buf);
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

static void chassis_apply_speed(int left, int right)
{
    motor_set_speed(clamp_speed(left), clamp_speed(right));
}

static void chassis_reset_tracking(void)
{
    pid_reset(&line_pid);
    lost_count = 0;
    cross_count = 0;
    cross_latched = 0;
    turn_hint_count = 0;
    turn_priority_ticks = 0;
    turn_priority_dir = 0;
    turn_cooldown_ticks = 0;
    route_straight_ticks = 0;
    last_line_error = 0;
    last_line_correction = 0;
}

static void chassis_start_turn_priority(int dir)
{
    turn_priority_dir = (dir < 0) ? -1 : 1;
    turn_priority_ticks = CHASSIS_TURN_PRIORITY_TICKS;
    turn_hint_count = 0;
    pid_reset(&line_pid);

    track_bridge_set_turn_mask((int8_t)turn_priority_dir);

    last_line_error = turn_priority_dir * LINE_TRACK_ERROR_MAX;
    last_line_correction = turn_priority_dir * CHASSIS_TURN_LIMIT;
}

static void chassis_apply_turn_priority(void)
{
    if (turn_priority_dir < 0) {
        chassis_apply_speed(-CHASSIS_TURN_PRIORITY_SPEED,
                             CHASSIS_TURN_PRIORITY_SPEED);
    } else {
        chassis_apply_speed( CHASSIS_TURN_PRIORITY_SPEED,
                            -CHASSIS_TURN_PRIORITY_SPEED);
    }
}

static void encoder_reset_counts(void) {}

void chassis_init(void)
{
    motor_init();
    track_bridge_init();
    track_bridge_mark_stale();
    pid_init(&line_pid, 30, 0, 16, CHASSIS_TURN_LIMIT, 1000);
    chassis_state = CHASSIS_IDLE;
    chassis_needs_pid_reset = 0;
    node_count = 0;
    chassis_reset_tracking();
}

void chassis_set_target(int target)
{
    if (target < (int)CHASSIS_TARGET_A || target > (int)CHASSIS_TARGET_D) {
        chassis_target = CHASSIS_TARGET_C;
    } else {
        chassis_target = (chassis_target_t)target;
    }

    node_count = 0;
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
    motor_lock();
    chassis_state = CHASSIS_LOCKED;
    chassis_needs_pid_reset = 1;
}

void chassis_emergency_stop(void)
{
    motor_lock();
    chassis_state = CHASSIS_LOCKED;
    chassis_needs_pid_reset = 1;
    chassis_reset_tracking();
}

void chassis_brake(void)
{
    motor_brake();
    chassis_needs_pid_reset = 1;
    last_line_correction = 0;
}

void chassis_stop(void)
{
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
        chassis_rotate_left(CHASSIS_FIND_SPEED);
    } else {
        chassis_rotate_right(CHASSIS_FIND_SPEED);
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
    node_count = 0;
    chassis_reset_tracking();
    chassis_needs_pid_reset = 1;

    route_index = 0;
    /* Select route by (mode, target).
       Confirmed: DRV+TARGET_C, NAV/ADV+TARGET_A/D.
       Unmatched combos (e.g. NAV+TARGET_B) -> ERROR, not free driving. */
    route_table = chassis_route_lookup(app_state_get_mode(), target, &route_length);
    route_active = (route_table != 0 && route_length > 0) ? 1 : 0;

    if (!route_active) {
        /* No route for this mode/target -- fail safe.
           B/C routes are not yet confirmed and must not
           fall back to pure line following. */
        chassis_state = CHASSIS_ERROR;
        return;
    }

    if (!motor_is_locked()) {
        chassis_state = CHASSIS_LINE;
    }
}

void chassis_run_line(void)
{
    chassis_line_input_t line = chassis_read_line_input();
    int correction;
    int base_speed;
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
        motor_brake();
        chassis_state = CHASSIS_ERROR;
        chassis_debug_log(&line);
        return;
    }

    if (!line.line_detected || line.all_white) {
        lost_count++;
        if (lost_count >= CHASSIS_LOST_LINE_SHORT_TICKS) {
            chassis_find_line();
            chassis_state = CHASSIS_FIND_LINE;
        } else {
            chassis_apply_speed(CHASSIS_BASE_SPEED + last_line_correction,
                               CHASSIS_BASE_SPEED - last_line_correction);
        }
        chassis_debug_log(&line);
        return;
    }

    lost_count = 0;
    if (chassis_state == CHASSIS_FIND_LINE) {
        pid_reset(&line_pid);
    }

    if (line.turn_hint != LINE_TRACK_TURN_HINT_NONE && turn_priority_ticks == 0) {
        if (turn_cooldown_ticks == 0 && route_straight_ticks == 0) {
            turn_hint_count++;
            if (turn_hint_count >= CHASSIS_TURN_HINT_CONFIRM_TICKS) {
                if (!route_active) {
                    chassis_start_turn_priority(line.turn_hint);
                }
            }
        } else {
            turn_hint_count = 0;
        }
    } else if (turn_priority_ticks == 0) {
        turn_hint_count = 0;
    }

    if (turn_priority_ticks > 0) {
        int elapsed = CHASSIS_TURN_PRIORITY_TICKS - turn_priority_ticks;

        if (elapsed >= CHASSIS_TURN_PRIORITY_MIN_TICKS &&
            line.center_hit &&
            line.turn_hint == LINE_TRACK_TURN_HINT_NONE) {
            turn_priority_ticks = 0;
            turn_priority_dir = 0;
            track_bridge_clear_turn_mask();
            turn_cooldown_ticks = CHASSIS_TURN_COOLDOWN_TICKS;
            pid_reset(&line_pid);
        } else {
            chassis_apply_turn_priority();
            turn_priority_ticks--;
            if (turn_priority_ticks == 0) {
                track_bridge_clear_turn_mask();
                turn_cooldown_ticks = CHASSIS_TURN_COOLDOWN_TICKS;
            }
            chassis_state = CHASSIS_LINE;
            chassis_debug_log(&line);
            return;
        }
    }

    if (turn_priority_ticks == 0 && turn_cooldown_ticks > 0) {
        turn_cooldown_ticks--;
    }

    if (route_straight_ticks > 0) {
        route_straight_ticks--;
    }

    if (route_active && turn_priority_ticks == 0) {
        if (line.active_count >= CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD) {
            cross_count++;
            if (!cross_latched && cross_count >= CHASSIS_CROSS_CONFIRM) {
                cross_latched = 1;
                node_count++;

                if (route_index < route_length) {
                    route_action_t action = route_table[route_index];
                    route_index++;

                    switch (action) {
                        case ROUTE_ACTION_RIGHT:
                            chassis_start_turn_priority(1);
                            chassis_state = CHASSIS_CROSS;
                            chassis_debug_log(&line);
                            return;
                        case ROUTE_ACTION_LEFT:
                            chassis_start_turn_priority(-1);
                            chassis_state = CHASSIS_CROSS;
                            chassis_debug_log(&line);
                            return;
                        case ROUTE_ACTION_STRAIGHT:
                            route_straight_ticks = CHASSIS_TURN_COOLDOWN_TICKS;
                            break;
                        case ROUTE_ACTION_STOP:
                            motor_brake();
                            chassis_state = CHASSIS_FINISHED;
                            chassis_debug_log(&line);
                            return;
                    }
                }
            }
        } else {
            cross_count = 0;
            cross_latched = 0;
        }
    } else if (!route_active) {
        cross_count = 0;
        cross_latched = 0;
    }

    correction = pid_update(&line_pid, line.error);

    if (turn_cooldown_ticks == 0 && route_straight_ticks == 0) {
        if (line.turn_hint == LINE_TRACK_TURN_HINT_LEFT && correction > -CHASSIS_TURN_LIMIT / 2) {
            correction -= CHASSIS_TURN_LIMIT / 2;
        } else if (line.turn_hint == LINE_TRACK_TURN_HINT_RIGHT &&
                   correction < CHASSIS_TURN_LIMIT / 2) {
            correction += CHASSIS_TURN_LIMIT / 2;
        }
    }

    correction = clamp_symmetric(correction, CHASSIS_TURN_LIMIT);

    correction = (last_line_correction * 3 + correction) / 4;

    last_line_error = line.error;
    last_line_correction = correction;

    base_speed = (abs_int(line.error) >= CHASSIS_CURVE_ERROR) ?
                 CHASSIS_CURVE_SPEED : CHASSIS_BASE_SPEED;

    left = base_speed + correction;
    right = base_speed - correction;
    chassis_apply_speed(left, right);
    chassis_state = CHASSIS_LINE;
    chassis_debug_log(&line);
}

void chassis_encoder_reset(void)
{
    encoder_reset_counts();
}

void chassis_encoder_get_counts(int *e1, int *e2, int *e3, int *e4)
{
    if (e1) *e1 = 0;
    if (e2) *e2 = 0;
    if (e3) *e3 = 0;
    if (e4) *e4 = 0;
}

int chassis_encoder_get_left_count(void)
{
    return 0;
}

int chassis_encoder_get_right_count(void)
{
    return 0;
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

int chassis_get_node_count(void)
{
    return node_count;
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
        chassis_state == CHASSIS_LINE ||
        chassis_state == CHASSIS_CROSS) {
        chassis_state = CHASSIS_LINE_LOST;
    }
}

void chassis_debug_simulate_error(void)
{
    chassis_state = CHASSIS_ERROR;
}
