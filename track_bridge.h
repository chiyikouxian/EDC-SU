#ifndef TRACK_BRIDGE_H
#define TRACK_BRIDGE_H

#include "app_common.h"
#include "line_track.h"

/*
 * Track Bridge — preparation layer between line_track and chassis.
 *
 * Purpose:
 *   Holds the latest LineTrackState_t result as a stable, zero-copy data
 *   contract that Hardware A (chassis) can consume without coupling to
 *   gray_sensor or line_track internals.
 *
 * Update discipline:
 *   - track_bridge_update() is called ONLY from handle_running() in
 *     app_state.c, once per 10 ms main-loop tick, before chassis_tick().
 *   - In all other states (IDLE, FDET, GSCAN, SBEEP, TSTOP, FBEEP, ERROR)
 *     the bridge is NOT updated and remains marked stale.
 *
 * Freshness:
 *   - `stale` is the primary validity indicator for downstream code.
 *   - `tick` is a logical monotonic counter incremented by the caller;
 *     it represents main-control tick sequence ordering and does NOT
 *     permanently guarantee a fixed physical duration (e.g. 10 ms).
 *     Downstream SHALL treat `stale` as the gate and `tick` as auxiliary.
 *
 * Hardware A/B boundary:
 *   - Main control (B) provides: error, line_detected, all_white,
 *     center_hit, active_count, tick, stale.
 *   - Main control does NOT provide: motor PWM, differential targets,
 *     PID parameters, intersection strategy, path action tables.
 *   - Chassis (A) responsibility: read track_bridge_get() → PID/PD →
 *     drive motors.
 */

typedef struct {
    int16_t  error;          /* signed bounded tracking error [-100, +100]  */
    uint8_t  active_count;   /* probes seeing the line (0..8)               */
    bool     line_detected;  /* at least one probe sees the line            */
    bool     all_white;      /* no probe sees the line (possible lost)      */
    bool     center_hit;     /* |error| <= threshold AND active_count >= 2  */
    int8_t   turn_hint;      /* -1 left, +1 right when straight+turn seen  */
    int8_t   turn_mask;      /* +1 right-turn masks L, -1 left-turn masks R,
                                0 = no mask active                         */
    uint32_t tick;           /* logical tick count at last update           */
    bool     stale;          /* true = data is NOT from the current RUN
                                cycle; false = updated this RUNNING tick    */
} TrackBridgeData_t;

void track_bridge_init(void);

/* Mark bridge data as stale.  Called on every state transition so that
   only RUNNING ticks (via track_bridge_update) can clear the flag. */
void track_bridge_mark_stale(void);

/*
 * Store a new tracking snapshot.  Called once per tick from handle_running().
 * `tick` should be a logical monotonic counter (e.g. state_ticks).
 */
void track_bridge_update(const LineTrackState_t *state, uint32_t tick);

/* Zero-copy read.  Returns pointer to internal static data.  Never blocks. */
const TrackBridgeData_t *track_bridge_get(void);

/*
 * Compact debug string (no "B=" prefix — main.c adds it).
 *   "E:+035 HIT"  — line right, centre hit
 *   "E:-042"      — line left, not centred
 *   "---"         — running, no line detected
 *   "STALE"       — bridge has not been updated in the current RUN cycle
 */
/* Turn-mask control — chassis sets direction; app_state applies to raw sensors. */
void  track_bridge_set_turn_mask(int8_t mask);
void  track_bridge_clear_turn_mask(void);
int8_t track_bridge_get_turn_mask(void);

const char *track_bridge_debug_string(void);

#endif /* TRACK_BRIDGE_H */
