#ifndef CHASSIS_IFACE_H
#define CHASSIS_IFACE_H

#include "app_common.h"

/* ========================================================================
 * Chassis parameter macros — tunable by Hardware A/B during integration.
 * ======================================================================== */

/* Primary input path: 1 = track_bridge_get(), 0 = local track_read() fallback.
   Production / competition builds MUST set this to 1. */
#define CHASSIS_USE_TRACK_BRIDGE           1

/* Scale bridge error [-100,+100] → chassis PID range.  Default 4 maps to
   [-400,+400].  This is a chassis-side concern; the bridge contract stays at
   [-100,+100] permanently.  Tune during vehicle calibration. */
#define CHASSIS_TRACK_ERROR_SCALE          1

/* 1 = the side carrying the tracking module is the logical front.
 * This rotates the chassis coordinate system by 180 degrees: logical left
 * and right wheels map to the opposite physical wheels, and forward is the
 * former reverse direction.  Set to 0 to return to the original layout. */
#ifndef CHASSIS_REVERSE_FORWARD_DIRECTION
#define CHASSIS_REVERSE_FORWARD_DIRECTION  1
#endif

/* Lost-line tick thresholds (logical ticks, currently 10 ms each).
   Short: begin find-line rotation.  Long: brake protection stop.
   Physical time = ticks × TICK_MS (currently 10 ms). */
/* During the first Short ticks, chassis_iface.c coasts using the last
   steering correction instead of feeding error=0 into PID. */
#define CHASSIS_LOST_LINE_SHORT_TICKS      18   /* 180 ms grace/coast, then find */
#define CHASSIS_LOST_LINE_LONG_TICKS       180  /* 1800 ms protection stop */

/* Encoder feedback: 0 = off, 1 = count GPIO_ENCODER quadrature pulses. */
#ifndef CHASSIS_USE_ENCODER
#define CHASSIS_USE_ENCODER                1
#endif

/* Wheel-speed feedback: 1 = encoder PI/PID correction, 0 = direct open loop.
   The loop runs whenever the chassis receives its 10 ms wheel command. */
#ifndef CHASSIS_USE_SPEED_PID
#define CHASSIS_USE_SPEED_PID              1
#endif

/* Encoder transitions expected in 10 ms for one motor-command unit, x100.
   The raised-wheel test measured about 6.1 transitions at command 8, so the
   calibrated value is 6.1 / 8 = 0.76 transitions per command unit. */
#define CHASSIS_SPEED_COUNTS_PER_CMD_X100  42

/* pid.c gains use a scale of 100.  Error is encoder transitions x100 and
   output is motor command units.  PI control for faster speed response. */
#define CHASSIS_SPEED_PID_KP               5
#define CHASSIS_SPEED_PID_KI               2
#define CHASSIS_SPEED_PID_KD               0
#define CHASSIS_SPEED_PID_CORRECTION_LIMIT 12
#define CHASSIS_SPEED_PID_INTEGRAL_LIMIT   1500

/* ========================================================================
 * Contract types — stable API between app_state and chassis.
 * ======================================================================== */

typedef enum {
    CHASSIS_STATUS_IDLE = 0,
    CHASSIS_STATUS_FINDING_LINE,
    CHASSIS_STATUS_FOLLOWING,
    CHASSIS_STATUS_TARGET_REACHED,
    CHASSIS_STATUS_LINE_LOST,
    CHASSIS_STATUS_ERROR
} ChassisStatus_t;

/* ========================================================================
 * Contract API — called by app_state.  DO NOT change signatures.
 * ======================================================================== */

void            chassis_init(void);
void            chassis_tick(void);
void            chassis_lock(void);
void            chassis_unlock(void);
void            chassis_find_line(void);
void            chassis_follow_target(TargetPoint_t target);
void            chassis_stop(void);
ChassisStatus_t chassis_get_status(void);
TargetPoint_t   chassis_get_target(void);

/* ---- debug / dry-run helpers (do NOT call from production flow) ---- */
void chassis_debug_simulate_line_lost(void);
void chassis_debug_simulate_error(void);

/* ========================================================================
 * Hardware A extended interface — available after real-chassis merge.
 * Called internally by chassis_iface.c; also available to app_state if
 * needed (e.g. emergency stop from error handler).
 * ======================================================================== */

void chassis_brake(void);
void chassis_emergency_stop(void);

void chassis_drive(int forward, int turn);
void chassis_forward(int speed);
void chassis_backward(int speed);
void chassis_turn_left(int speed);
void chassis_turn_right(int speed);
void chassis_rotate_left(int speed);
void chassis_rotate_right(int speed);

void  chassis_set_target(int target);
void  chassis_follow_target_ex(int target);
void  chassis_run_line(void);

int   chassis_is_finished(void);

/* Encoder interface. chassis_encoder_poll() is called from the 1 ms main loop. */
void chassis_encoder_poll(void);
void chassis_encoder_reset(void);
void chassis_encoder_get_counts(int *left, int *right);
int  chassis_encoder_get_left_count(void);
int  chassis_encoder_get_right_count(void);
void chassis_speed_pid_get_debug(int *target_left_x100,
                                 int *target_right_x100,
                                 int *measured_left_x100,
                                 int *measured_right_x100,
                                 int *pwm_left,
                                 int *pwm_right);

/* Internal state query (debug only — returns chassis_state_t cast to int) */
int  chassis_get_state(void);

#endif /* CHASSIS_IFACE_H */
