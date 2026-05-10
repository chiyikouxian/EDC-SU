/*
 * Chassis interface — Hardware A real-chassis implementation (Phase 1).
 *
 * Migrated from EDC-SHEN-Car chassis_iface.c.  Phase 1 scope:
 *   open-loop motor + track_bridge line following, encoder OFF.
 *
 * Data flow (RUNNING):
 *   handle_running() → track_bridge_update() → chassis_tick()
 *   chassis_tick() → chassis_read_line_input() → track_bridge_get()
 *     → stale? → CHASSIS_ERROR
 *     → FIND_LINE? → rotate recovery / timeout
 *     → lost? → short FLIN / long LOST
 *     → node? → CROSS → RCHD
 *     → PID(line.error) → motor_set_speed(L,R)
 *
 * Excluded (Phase 2+):
 *   encoder closed-loop, legacy track fallback, GROUP1_IRQHandler.
 */

#include "chassis_iface.h"
#include "motor.h"
#include "pid.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include "track_bridge.h"

/* ---- tunable macros ---- */
#define CHASSIS_BASE_SPEED                 12
#define CHASSIS_MAX_SPEED                  28
#define CHASSIS_CURVE_SPEED                4
#define CHASSIS_FIND_SPEED                 16
#define CHASSIS_LOST_COAST_SPEED           7
#define CHASSIS_LOST_COAST_TURN_LIMIT      6
#define CHASSIS_TURN_HINT_CONFIRM_TICKS    1
#define CHASSIS_CROSS_CONFIRM               4
#define CHASSIS_CURVE_ERROR               300
#define CHASSIS_ENABLE_TARGET_STOP          0

#define CHASSIS_CORNER_ERROR              150
#define CHASSIS_CORNER_LOST_CONFIRM_TICKS   2
#define CHASSIS_CORNER_BRAKE_TICKS          4
#define CHASSIS_CORNER_TURN_TICKS         60
#define CHASSIS_CORNER_TURN_SPEED         20

/* ---- internal types ---- */

typedef enum {
    CORNER_NONE = 0,
    CORNER_BRAKE,
    CORNER_TURN
} corner_phase_t;

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

typedef struct {
    int error;
    int line_detected;
    int all_white;
    int active_count;
    int turn_hint;
    int stale;
} chassis_line_input_t;

/* ---- static state ---- */

static pid_t           line_pid;
static chassis_state_t chassis_state         = CHASSIS_LOCKED;
static chassis_target_t chassis_target       = CHASSIS_TARGET_C;
static TargetPoint_t   external_target       = TARGET_NONE;
static int             last_line_error       = 0;
static int             last_line_correction  = 0;
static int             chassis_needs_pid_reset = 0;
static int             lost_count            = 0;
static int             turn_hint_count       = 0;
static int             cross_count           = 0;
static int             cross_latched         = 0;
static int             node_count            = 0;
static corner_phase_t  corner_phase          = CORNER_NONE;
static int             corner_ticks          = 0;
static int             corner_dir            = 0;  /* -1 left, +1 right */

#if CHASSIS_ENABLE_TARGET_STOP
static const int target_finish_nodes[] = {
    1,  /* Target A */
    1,  /* Target B */
    2,  /* Target C */
    1   /* Target D */
};
#endif

/* ---- helpers ---- */

static void corner_reset(void)
{
    corner_phase = CORNER_NONE;
    corner_ticks = 0;
    corner_dir   = 0;
    turn_hint_count = 0;
}

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

static ChassisStatus_t chassis_map_state_to_status(chassis_state_t state)
{
    switch (state) {
        case CHASSIS_LOCKED:
        case CHASSIS_IDLE:      return CHASSIS_STATUS_IDLE;
        case CHASSIS_FIND_LINE: return CHASSIS_STATUS_FINDING_LINE;
        case CHASSIS_LINE:
        case CHASSIS_CROSS:     return CHASSIS_STATUS_FOLLOWING;
        case CHASSIS_FINISHED:  return CHASSIS_STATUS_TARGET_REACHED;
        case CHASSIS_ERROR:     return CHASSIS_STATUS_ERROR;
        case CHASSIS_LINE_LOST: return CHASSIS_STATUS_LINE_LOST;
        default:                return CHASSIS_STATUS_IDLE;
    }
}

static chassis_target_t chassis_convert_target(TargetPoint_t target)
{
    switch (target) {
        case TARGET_A:  return CHASSIS_TARGET_A;
        case TARGET_B:  return CHASSIS_TARGET_B;
        case TARGET_C:  return CHASSIS_TARGET_C;
        case TARGET_D:  return CHASSIS_TARGET_D;
        case TARGET_NONE:
        default:        return CHASSIS_TARGET_C;
    }
}

/* ---- uart debug (shared DEBUG_UART with main.c) ---- */

static void debug_uart_send_char(char c)
{
    while (DL_UART_isBusy(DEBUG_UART_INST)) {}
    DL_UART_transmitData(DEBUG_UART_INST, (uint8_t)c);
}

static void debug_uart_send_str(const char *s)
{
    while (*s) { debug_uart_send_char(*s++); }
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

static void chassis_debug_log(const chassis_line_input_t *line)
{
    static chassis_state_t last_state  = (chassis_state_t)0xFF;
    static ChassisStatus_t last_status = (ChassisStatus_t)0xFF;
    ChassisStatus_t status = chassis_map_state_to_status(chassis_state);
    char buf[160];

    if (!(chassis_state != last_state || status != last_status ||
          line->stale || status == CHASSIS_STATUS_LINE_LOST ||
          status == CHASSIS_STATUS_ERROR))
    {
        return;
    }

    last_state  = chassis_state;
    last_status = status;

    (void)snprintf(buf, sizeof(buf),
        "CH: st=%s ext=%s err=%d line=%d white=%d active=%d stale=%d lost=%d node=%d target=%d lock=%d\r\n",
        chassis_state_name(chassis_state),
        chassis_status_name(status),
        line->error, line->line_detected, line->all_white,
        line->active_count, line->stale,
        lost_count, node_count,
        (int)chassis_target, motor_is_locked());
    debug_uart_send_str(buf);
}

/* ---- bridge input ---- */

static chassis_line_input_t chassis_read_line_input(void)
{
    chassis_line_input_t line = {0};
    const TrackBridgeData_t *bridge = track_bridge_get();

    if (bridge == 0) {
        line.stale    = 1;
        line.all_white = 1;
        return line;
    }

    line.error        = bridge->error * CHASSIS_TRACK_ERROR_SCALE;
    line.line_detected = bridge->line_detected ? 1 : 0;
    line.all_white    = bridge->all_white     ? 1 : 0;
    line.active_count = bridge->active_count;
    line.turn_hint    = bridge->turn_hint;
    line.stale        = bridge->stale         ? 1 : 0;
    return line;
}

/* ---- motor output (open-loop, encoder=0) ---- */

static void chassis_apply_speed(int left, int right)
{
    motor_set_speed(clamp_speed(left), clamp_speed(right));
}

static void chassis_coast_through_lost_line(void)
{
    int correction = clamp_symmetric(last_line_correction,
                                     CHASSIS_LOST_COAST_TURN_LIMIT);
    chassis_apply_speed(CHASSIS_LOST_COAST_SPEED + correction,
                        CHASSIS_LOST_COAST_SPEED - correction);
}

static void chassis_start_corner_turn(int dir)
{
    corner_dir   = (dir < 0) ? -1 : 1;
    corner_phase = CORNER_BRAKE;
    corner_ticks = CHASSIS_CORNER_BRAKE_TICKS;
    turn_hint_count = 0;
    lost_count = 0;
    last_line_correction = 0;
    pid_reset(&line_pid);
    motor_brake();
    chassis_state = CHASSIS_LINE;
}

static void encoder_reset_counts(void) {}

/* ---- contract API: init ---- */

void chassis_init(void)
{
    motor_init();
    track_bridge_init();
    track_bridge_mark_stale();
    pid_init(&line_pid, 12, 0, 6, 18, 1200);
    chassis_state = CHASSIS_IDLE;
    chassis_needs_pid_reset = 0;
    corner_reset();
    lost_count    = 0;
    cross_count   = 0;
    cross_latched = 0;
    node_count    = 0;
    last_line_error = 0;
    last_line_correction = 0;
}

/* ---- contract API: target setting ---- */

void chassis_set_target(int target)
{
    chassis_target_t ct;

    if (target < (int)CHASSIS_TARGET_A || target > (int)CHASSIS_TARGET_D) {
        ct = CHASSIS_TARGET_C;
    } else {
        ct = (chassis_target_t)target;
    }

    chassis_target    = ct;
    node_count        = 0;
    cross_count       = 0;
    cross_latched     = 0;
    lost_count        = 0;
    last_line_correction = 0;
    corner_reset();
    pid_reset(&line_pid);
}

/* ---- contract API: lock / unlock ---- */

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
    corner_reset();
}

/* ---- extended: emergency / brake / stop ---- */

void chassis_emergency_stop(void)
{
    motor_lock();
    pid_reset(&line_pid);
    last_line_correction = 0;
    chassis_state = CHASSIS_LOCKED;
    corner_reset();
}

void chassis_brake(void)
{
    motor_brake();
    pid_reset(&line_pid);
    last_line_correction = 0;
}

void chassis_stop(void)
{
    motor_coast();
    pid_reset(&line_pid);
    chassis_state = CHASSIS_IDLE;
    last_line_correction = 0;
    corner_reset();
}

/* ---- extended: drive commands ---- */

void chassis_drive(int forward, int turn)
{
    int left, right;

    if (motor_is_locked()) return;

    left  = clamp_speed(forward + turn);
    right = clamp_speed(forward - turn);
    chassis_apply_speed(left, right);
    chassis_state = CHASSIS_IDLE;
}

void chassis_forward(int speed)
{
    speed = clamp_speed(speed);
    chassis_drive(speed, 0);
}

void chassis_backward(int speed)
{
    speed = clamp_speed(speed);
    if (speed < 0) speed = -speed;
    chassis_drive(-speed, 0);
}

void chassis_turn_left(int speed)
{
    speed = clamp_speed(speed);
    if (speed < 0) speed = -speed;
    chassis_drive(speed / 2, -speed / 2);
}

void chassis_turn_right(int speed)
{
    speed = clamp_speed(speed);
    if (speed < 0) speed = -speed;
    chassis_drive(speed / 2, speed / 2);
}

void chassis_rotate_left(int speed)
{
    speed = clamp_speed(speed);
    if (speed < 0) speed = -speed;
    chassis_drive(0, -speed);
}

void chassis_rotate_right(int speed)
{
    speed = clamp_speed(speed);
    if (speed < 0) speed = -speed;
    chassis_drive(0, speed);
}

/* ---- contract API: find_line ---- */

void chassis_find_line(void)
{
    if (motor_is_locked()) return;

    if (last_line_error >= 0) {
        chassis_rotate_right(CHASSIS_FIND_SPEED);
    } else {
        chassis_rotate_left(CHASSIS_FIND_SPEED);
    }

    chassis_state = CHASSIS_FIND_LINE;
}

/* ---- contract API: follow_target ---- */

void chassis_follow_target(TargetPoint_t target)
{
    chassis_target_t ctarget;

    if (target == TARGET_NONE) {
        chassis_state = CHASSIS_ERROR;
        return;
    }

    ctarget = chassis_convert_target(target);
    if (ctarget < CHASSIS_TARGET_A || ctarget > CHASSIS_TARGET_D) {
        ctarget = CHASSIS_TARGET_C;
    }

    external_target         = target;
    chassis_target          = ctarget;
    node_count              = 0;
    cross_count             = 0;
    cross_latched           = 0;
    lost_count              = 0;
    last_line_correction    = 0;
    chassis_needs_pid_reset = 1;
    corner_reset();

    if (!motor_is_locked()) {
        chassis_state = CHASSIS_LINE;
    }
}

/* ---- core: line-following tick ---- */

void chassis_run_line(void)
{
    chassis_line_input_t line = chassis_read_line_input();
    int correction, left, right;

    if (motor_is_locked() || chassis_state == CHASSIS_FINISHED) {
        return;
    }

    if (chassis_needs_pid_reset) {
        chassis_needs_pid_reset = 0;
        pid_reset(&line_pid);
    }

    /* 1. stale — unconditional, highest priority */
    if (line.stale) {
        motor_brake();
        chassis_state = CHASSIS_ERROR;
        corner_reset();
        chassis_debug_log(&line);
        return;
    }

    /* 2. FIND_LINE — rotate recovery (non-stale data only) */
    if (chassis_state == CHASSIS_FIND_LINE) {
        if (line.line_detected && !line.all_white) {
            lost_count = 0;
            /* fall through to normal tracking */
        } else {
            lost_count++;
            chassis_find_line();
            chassis_debug_log(&line);
            return;
        }
    }

    /* 3. lost-line: short → FLIN, long → LOST */
    /* The first Short ticks are a grace window for insensitive/intermittent sensors. */
    if (corner_phase == CORNER_NONE && (!line.line_detected || line.all_white)) {
        lost_count++;
        if (lost_count >= CHASSIS_LOST_LINE_LONG_TICKS) {
            motor_brake();
            chassis_state = CHASSIS_LINE_LOST;
            pid_reset(&line_pid);
            corner_reset();
            chassis_debug_log(&line);
            return;
        }
        if (lost_count >= CHASSIS_LOST_LINE_SHORT_TICKS) {
            chassis_find_line();
            chassis_debug_log(&line);
            return;
        }
        if (lost_count >= CHASSIS_CORNER_LOST_CONFIRM_TICKS) {
            if (last_line_error <= -CHASSIS_CORNER_ERROR) {
                chassis_start_corner_turn(-1);
                chassis_debug_log(&line);
                return;
            }
            if (last_line_error >= CHASSIS_CORNER_ERROR) {
                chassis_start_corner_turn(1);
                chassis_debug_log(&line);
                return;
            }
        }
        chassis_coast_through_lost_line();
        chassis_state = CHASSIS_LINE;
        chassis_debug_log(&line);
        return;
    } else {
        lost_count = 0;
    }

    if (corner_phase == CORNER_NONE && line.turn_hint != LINE_TRACK_TURN_HINT_NONE) {
        turn_hint_count++;
        if (turn_hint_count >= CHASSIS_TURN_HINT_CONFIRM_TICKS) {
            last_line_error = (line.turn_hint < 0) ? -CHASSIS_CORNER_ERROR
                                                   :  CHASSIS_CORNER_ERROR;
            chassis_start_corner_turn(line.turn_hint);
            chassis_debug_log(&line);
            return;
        }
    } else {
        turn_hint_count = 0;
    }

    /* 4. node / intersection detection */
#if CHASSIS_ENABLE_TARGET_STOP
    if (line.active_count >= CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD) {
        cross_count++;
        if (!cross_latched && cross_count >= CHASSIS_CROSS_CONFIRM) {
            cross_latched = 1;
            node_count++;
            chassis_state = CHASSIS_CROSS;

            if (node_count >= target_finish_nodes[chassis_target]) {
                motor_brake();
                chassis_state = CHASSIS_FINISHED;
                corner_reset();
                chassis_debug_log(&line);
                return;
            }
        }
    } else {
        cross_count   = 0;
        cross_latched = 0;
    }
#else
    cross_count   = 0;
    cross_latched = 0;
#endif

    /* 4a. corner brake-turn assist — brake then pivot to shed forward inertia */

    if (corner_phase == CORNER_BRAKE) {
        motor_brake();
        corner_ticks--;
        chassis_state = CHASSIS_LINE;
        chassis_debug_log(&line);
        if (corner_ticks <= 0) {
            corner_phase = CORNER_TURN;
            corner_ticks = CHASSIS_CORNER_TURN_TICKS;
        }
        return;
    }

    if (corner_phase == CORNER_TURN) {
        if (corner_dir < 0) {
            chassis_apply_speed(clamp_speed(-CHASSIS_CORNER_TURN_SPEED),
                                clamp_speed( CHASSIS_CORNER_TURN_SPEED));
        } else {
            chassis_apply_speed(clamp_speed( CHASSIS_CORNER_TURN_SPEED),
                                clamp_speed(-CHASSIS_CORNER_TURN_SPEED));
        }
        if (corner_ticks > 0) {
            corner_ticks--;
        }
        chassis_state = CHASSIS_LINE;
        chassis_debug_log(&line);
        if (corner_ticks <= 0 && line.line_detected && !line.all_white) {
            lost_count = 0;
            corner_reset();
            pid_reset(&line_pid);
        }
        return;
    }

    if (line.error <= -CHASSIS_CORNER_ERROR) {
        chassis_start_corner_turn(-1);
        chassis_debug_log(&line);
        return;
    }

    if (line.error >= CHASSIS_CORNER_ERROR) {
        chassis_start_corner_turn(1);
        chassis_debug_log(&line);
        return;
    }

    /* 5. PID tracking */
    last_line_error = line.error;
    correction = pid_update(&line_pid, line.error);
    last_line_correction = correction;

    if (line.error > CHASSIS_CURVE_ERROR || line.error < -CHASSIS_CURVE_ERROR) {
        left  = CHASSIS_CURVE_SPEED + correction;
        right = CHASSIS_CURVE_SPEED - correction;
    } else {
        left  = CHASSIS_BASE_SPEED + correction;
        right = CHASSIS_BASE_SPEED - correction;
    }

    chassis_apply_speed(clamp_speed(left), clamp_speed(right));
    chassis_state = CHASSIS_LINE;
    chassis_debug_log(&line);
}

/* ---- encoder stubs (Phase 1 — encoder=0, return 0) ---- */

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

/* ---- contract API: tick / status / target ---- */

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

/* ---- extended: query ---- */

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

/* ---- extended: legacy follow_target_ex ---- */

void chassis_follow_target_ex(int target)
{
    chassis_set_target(target);
    chassis_run_line();
}

/* ---- debug helpers (from dry-run stub, bench test only) ---- */

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
