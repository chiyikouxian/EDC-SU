#ifndef LINE_TRACK_H
#define LINE_TRACK_H

#include "app_common.h"
#include "gray_sensor.h"

/*
 * Convert one 8-channel grayscale frame into a signed line error.
 *
 * Error convention:
 *   Positive (+) = line is right of centre, steer right.
 *   Negative (-) = line is left of centre, steer left.
 *   Zero         = line is centred between X4 and X5.
 *
 * This follows the reference CCD idea at 8-channel resolution: find the black
 * band edges, use their midpoint as the line centre, then run PD steering in
 * the chassis layer.  If there are several active bands, the longest band is
 * used; ties prefer the previous valid centre to avoid sudden jumps.
 */

#define LINE_TRACK_ERROR_MAX              100
#define LINE_TRACK_CENTER_HIT_THRESHOLD    15
#define LINE_TRACK_TURN_HINT_LEFT          -1
#define LINE_TRACK_TURN_HINT_NONE           0
#define LINE_TRACK_TURN_HINT_RIGHT          1

typedef struct {
    int16_t error;        /* signed bounded tracking error [-100, +100] */
    uint8_t active_count; /* number of probes seeing the line (0..8)    */
    bool    line_detected;
    bool    all_white;
    bool    center_hit;   /* line near centre and at least 2 probes     */
    int8_t  turn_hint;    /* -1 left, +1 right when a branch is likely  */
} LineTrackState_t;

void line_track_reset(void);
void line_track_compute(const uint8_t values[GRAY_SENSOR_CHANNELS],
                        LineTrackState_t *out);
const char *line_track_debug_string(const LineTrackState_t *state);

#endif /* LINE_TRACK_H */
