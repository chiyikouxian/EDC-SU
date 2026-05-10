#include "line_track.h"
#include "gray_sensor.h"

/*
 * Centroid-based error calculation with pure integer arithmetic.
 *
 * Each probe i (0 = X1 .. 7 = X8) has position i.
 * The physical centre lies between probes 3 and 4, at 3.5.
 *
 *   centroid  = Σ(pos × active) / active_count
 *   error_raw = (centroid − 3.5) / 3.5 × 100
 *             = Σ(pos) × 200 / (active_count × 7) − 100
 *
 * This maps:
 *   all probes at X1 (pos 0) → −100
 *   centroid at 3.5         →    0
 *   all probes at X8 (pos 7) → +100
 */
void line_track_compute(const uint8_t values[GRAY_SENSOR_CHANNELS],
                        LineTrackState_t *out)
{
    uint8_t  i;
    uint8_t  active_count = 0;
    uint16_t sum_pos      = 0;
    uint8_t  left_edge_active;
    uint8_t  right_edge_active;
    uint8_t  center_active;

    for (i = 0; i < GRAY_SENSOR_CHANNELS; i++) {
        if (values[i] == GRAY_ACTIVE_LEVEL) {
            active_count++;
            sum_pos += i;
        }
    }

    out->active_count  = active_count;
    out->line_detected = (active_count > 0);
    out->all_white     = (active_count == 0);
    out->turn_hint     = LINE_TRACK_TURN_HINT_NONE;

    if (active_count == 0) {
        out->error      = 0;
        out->center_hit = false;
        return;
    }

    /* Integer centroid → signed bounded error, rounded to nearest. */
    {
        int32_t num = (int32_t)sum_pos * 200;
        int32_t den = (int32_t)active_count * 7;
        int32_t div = num / den;
        int32_t rem = num % den;
        /* round half-up */
        if (rem < 0) rem = -rem;
        if (rem * 2 >= den) {
            div += (num >= 0) ? 1 : -1;
        }
        out->error = (int16_t)(div - 100);
    }

    /* Clamp to guard against any rounding overshoot. */
    if (out->error > LINE_TRACK_ERROR_MAX) {
        out->error = LINE_TRACK_ERROR_MAX;
    } else if (out->error < -LINE_TRACK_ERROR_MAX) {
        out->error = -LINE_TRACK_ERROR_MAX;
    }

    /*
     * Centre hit: error within threshold AND at least two probes active
     * (a single probe near centre still implies uncertain alignment).
     */
    {
        int16_t abs_err = out->error;
        if (abs_err < 0) abs_err = -abs_err;
        out->center_hit = (abs_err <= LINE_TRACK_CENTER_HIT_THRESHOLD)
                       && (active_count >= 2);
    }

    left_edge_active  = (values[0] == GRAY_ACTIVE_LEVEL)
                     || (values[1] == GRAY_ACTIVE_LEVEL);
    right_edge_active = (values[6] == GRAY_ACTIVE_LEVEL)
                     || (values[7] == GRAY_ACTIVE_LEVEL);
    center_active     = (values[3] == GRAY_ACTIVE_LEVEL)
                     || (values[4] == GRAY_ACTIVE_LEVEL);

    if (center_active && active_count >= 3) {
        if (left_edge_active && !right_edge_active) {
            out->turn_hint = LINE_TRACK_TURN_HINT_LEFT;
        } else if (right_edge_active && !left_edge_active) {
            out->turn_hint = LINE_TRACK_TURN_HINT_RIGHT;
        }
    }
}

const char *line_track_debug_string(const LineTrackState_t *state)
{
    static char buf[16];

    if (!state->line_detected) {
        return "---";
    }

    {
        int16_t e  = state->error;
        char    sign;
        uint8_t mag;

        if (e >= 0) {
            sign = '+';
            mag  = (uint8_t)e;
        } else {
            sign = '-';
            mag  = (uint8_t)(-e);
        }

        /*
         * Format: "E:sMMM" optionally " HIT"
         * buf[16] is ample for "E:+100 HIT\0" (12 chars).
         */
        {
            uint8_t pos = 0;
            buf[pos++] = 'E';
            buf[pos++] = ':';
            buf[pos++] = sign;

            /* hundreds digit */
            if (mag >= 100) {
                buf[pos++] = '0' + (mag / 100);
                mag = mag % 100;
            } else {
                buf[pos++] = '0';
            }

            /* tens digit */
            buf[pos++] = '0' + (mag / 10);
            /* units digit */
            buf[pos++] = '0' + (mag % 10);

            if (state->center_hit) {
                buf[pos++] = ' ';
                buf[pos++] = 'H';
                buf[pos++] = 'I';
                buf[pos++] = 'T';
            }
            buf[pos] = '\0';
        }
    }

    return buf;
}
