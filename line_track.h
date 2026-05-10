#ifndef LINE_TRACK_H
#define LINE_TRACK_H

#include "app_common.h"
#include "gray_sensor.h"

/*
 * Line-tracking abstraction — converts 8-channel grayscale sensor readings
 * into a signed bounded error and high-level line state for PID or rule-based
 * steering layers.
 *
 * Error convention:
 *   Positive (+) = line is RIGHT of centre  (robot should steer right)
 *   Negative (−) = line is LEFT  of centre  (robot should steer left)
 *   Zero         = line is centred
 *
 * The error is bounded to [−100, +100]:
 *   −100 = line under X1 only (hard left)
 *   +100 = line under X8 only (hard right)
 *      0 = centroid at centre (between X4 / X5)
 *
 * Channel ordering (X1 leftmost .. X8 rightmost) is defined by the grayscale
 * sensor module (gray_sensor.h) and is the only valid input order.
 */

#define LINE_TRACK_ERROR_MAX           100
#define LINE_TRACK_CENTER_HIT_THRESHOLD  15
#define LINE_TRACK_TURN_HINT_LEFT       -1
#define LINE_TRACK_TURN_HINT_NONE        0
#define LINE_TRACK_TURN_HINT_RIGHT       1

typedef struct {
    int16_t error;        /* signed bounded tracking error [-100, +100] */
    uint8_t active_count; /* number of probes seeing the line (0..8)   */
    bool    line_detected; /* at least one probe sees the line          */
    bool    all_white;     /* no probe sees the line (possible lost)    */
    bool    center_hit;    /* line near centre AND ≥2 probes active     */
    int8_t  turn_hint;     /* -1 left, +1 right when straight+turn seen */
} LineTrackState_t;

/*
 * Compute tracking state from one 8-channel grayscale frame.
 * `values` must hold exactly GRAY_SENSOR_CHANNELS elements and follow the
 * X1 (index 0) .. X8 (index 7) ordering.
 */
void line_track_compute(const uint8_t values[GRAY_SENSOR_CHANNELS],
                        LineTrackState_t *out);

/*
 * Format tracking state as a compact debug string:
 *   "E:+035 HIT"  — line right, centre hit
 *   "E:-042"      — line left, not centred
 *   "---"         — all white / no line detected
 * Returns a pointer to a static buffer valid until the next call.
 */
const char *line_track_debug_string(const LineTrackState_t *state);

#endif /* LINE_TRACK_H */
