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

/* Lost-line tick thresholds (logical ticks, currently 10 ms each).
   Short: begin find-line rotation.  Long: brake protection stop.
   Physical time = ticks × TICK_MS (currently 10 ms). */
/* During the first Short ticks, chassis_iface.c coasts using the last
   steering correction instead of feeding error=0 into PID. */
#define CHASSIS_LOST_LINE_SHORT_TICKS      18   /* 180 ms grace/coast, then find */
#define CHASSIS_LOST_LINE_LONG_TICKS       180  /* 1800 ms protection stop */

/* active_count >= this value → intersection / node region.  Empirical
   threshold; tune on real track. */
#define CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD 4

/* Encoder closed-loop: 0 = off (first-round default), 1 = on.
   Owned by Hardware A; keep off until encoder hardware is stable. */
#define CHASSIS_USE_ENCODER                0

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

int   chassis_get_node_count(void);
int   chassis_is_finished(void);

/* Encoder interface — Phase 1 stubs (encoder=0, always return 0).
   GPIO_ENCODER not configured in syscfg. */
void chassis_encoder_reset(void);
void chassis_encoder_get_counts(int *e1, int *e2, int *e3, int *e4);
int  chassis_encoder_get_left_count(void);
int  chassis_encoder_get_right_count(void);

/* Internal state query (debug only — returns chassis_state_t cast to int) */
int  chassis_get_state(void);

#endif /* CHASSIS_IFACE_H */
