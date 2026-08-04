#include "line_track.h"
#include "gray_sensor.h"

#define LINE_CENTER_X2  5   /* 2 * 2.5, centre between X3 and X4 */
#define LINE_ERROR_DEADBAND  5
#define LINE_LOST_HOLD_FRAMES  5
#define LINE_SMOOTH_DELTA      20

static int8_t last_center_x2 = LINE_CENTER_X2;
static int16_t last_error = 0;
static uint8_t lost_hold_count = 0;
static bool   has_last_center = false;

static int16_t abs_i16(int16_t value)
{
    return (value < 0) ? (int16_t)(-value) : value;
}

static int16_t clamp_error(int32_t error)
{
    if (error > LINE_TRACK_ERROR_MAX) {
        return LINE_TRACK_ERROR_MAX;
    }
    if (error < -LINE_TRACK_ERROR_MAX) {
        return -LINE_TRACK_ERROR_MAX;
    }
    return (int16_t)error;
}

static int16_t rounded_div_i32(int32_t num, int32_t den)
{
    if (num >= 0) {
        return (int16_t)((num + den / 2) / den);
    }
    return (int16_t)((num - den / 2) / den);
}

static uint8_t choose_active_band(const uint8_t values[GRAY_SENSOR_CHANNELS],
                                  uint8_t *best_start,
                                  uint8_t *best_end)
{
    uint8_t i = 0;
    uint8_t found = 0;
    uint8_t chosen_start = 0;
    uint8_t chosen_end = 0;
    uint8_t chosen_len = 0;
    int16_t chosen_dist = 32767;

    while (i < GRAY_SENSOR_CHANNELS) {
        uint8_t start;
        uint8_t end;
        uint8_t len;
        int16_t center_x2;
        int16_t dist;

        while (i < GRAY_SENSOR_CHANNELS &&
               values[i] != GRAY_ACTIVE_LEVEL) {
            i++;
        }
        if (i >= GRAY_SENSOR_CHANNELS) {
            break;
        }

        start = i;
        while (i < GRAY_SENSOR_CHANNELS &&
               values[i] == GRAY_ACTIVE_LEVEL) {
            i++;
        }
        end = (uint8_t)(i - 1);
        len = (uint8_t)(end - start + 1);
        center_x2 = (int16_t)(start + end);
        dist = abs_i16((int16_t)(center_x2 -
                       (has_last_center ? last_center_x2 : LINE_CENTER_X2)));

        if (!found || len > chosen_len ||
            (len == chosen_len && dist < chosen_dist)) {
            found = 1;
            chosen_start = start;
            chosen_end = end;
            chosen_len = len;
            chosen_dist = dist;
        }
    }

    *best_start = chosen_start;
    *best_end = chosen_end;
    return found;
}

void line_track_reset(void)
{
    last_center_x2 = LINE_CENTER_X2;
    last_error = 0;
    lost_hold_count = 0;
    has_last_center = false;
}

void line_track_compute(const uint8_t values[GRAY_SENSOR_CHANNELS],
                        LineTrackState_t *out)
{
    uint8_t i;
    uint8_t active_count = 0;
    uint8_t best_start = 0;
    uint8_t best_end = 0;
    uint8_t left_edge_active;
    uint8_t right_edge_active;
    uint8_t center_active;
    int16_t center_x2;
    int16_t raw_error;

    for (i = 0; i < GRAY_SENSOR_CHANNELS; i++) {
        if (values[i] == GRAY_ACTIVE_LEVEL) {
            active_count++;
        }
    }

    out->active_count = active_count;
    out->line_detected = (active_count > 0);
    out->all_white = (active_count == 0);
    out->turn_hint = LINE_TRACK_TURN_HINT_NONE;

    if (active_count == 0 || !choose_active_band(values, &best_start, &best_end)) {
        if (has_last_center && lost_hold_count < LINE_LOST_HOLD_FRAMES) {
            lost_hold_count++;
            out->active_count = 0;
            out->line_detected = true;
            out->all_white = false;
            out->error = last_error;
            out->center_hit = false;
            return;
        }

        out->error = last_error;
        out->center_hit = false;
        return;
    }

    lost_hold_count = 0;

    if (active_count == GRAY_SENSOR_CHANNELS) {
        center_x2 = LINE_CENTER_X2;
    } else {
        center_x2 = (int16_t)(best_start + best_end);
    }

    raw_error = clamp_error(
        rounded_div_i32(((int32_t)center_x2 - LINE_CENTER_X2) * 100, LINE_CENTER_X2));

    if (abs_i16(raw_error) <= LINE_ERROR_DEADBAND) {
        raw_error = 0;
    }

    last_center_x2 = (int8_t)center_x2;

    left_edge_active = (values[0] == GRAY_ACTIVE_LEVEL) ||
                       (values[1] == GRAY_ACTIVE_LEVEL);
    right_edge_active = (values[4] == GRAY_ACTIVE_LEVEL) ||
                        (values[5] == GRAY_ACTIVE_LEVEL);
    center_active = (values[2] == GRAY_ACTIVE_LEVEL) ||
                    (values[3] == GRAY_ACTIVE_LEVEL);

    /* Two outer black probes identify a sharp-corner candidate early.  The
     * chassis layer confirms it across consecutive frames before pivoting. */
    if (values[0] == GRAY_ACTIVE_LEVEL &&
        values[1] == GRAY_ACTIVE_LEVEL &&
        !right_edge_active) {
        out->turn_hint = LINE_TRACK_TURN_HINT_LEFT;
    } else if (values[4] == GRAY_ACTIVE_LEVEL &&
               values[5] == GRAY_ACTIVE_LEVEL &&
               !left_edge_active) {
        out->turn_hint = LINE_TRACK_TURN_HINT_RIGHT;
    } else if (center_active && active_count >= 3) {
        if (left_edge_active && !right_edge_active) {
            out->turn_hint = LINE_TRACK_TURN_HINT_LEFT;
        } else if (right_edge_active && !left_edge_active) {
            out->turn_hint = LINE_TRACK_TURN_HINT_RIGHT;
        }
    }

    if (out->turn_hint == LINE_TRACK_TURN_HINT_NONE && has_last_center) {
        int16_t delta = (int16_t)(raw_error - last_error);
        if (delta < 0) {
            delta = (int16_t)(-delta);
        }

        if (delta <= LINE_SMOOTH_DELTA) {
            out->error = (int16_t)((last_error * 3 + raw_error) / 4);
        } else {
            out->error = (int16_t)((last_error + raw_error) / 2);
        }
    } else {
        out->error = raw_error;
    }

    last_error = out->error;
    has_last_center = true;

    out->center_hit = (abs_i16(out->error) <= LINE_TRACK_CENTER_HIT_THRESHOLD) &&
                      (active_count >= 2);
}

const char *line_track_debug_string(const LineTrackState_t *state)
{
    static char buf[16];
    int16_t e;
    char sign;
    uint8_t mag;
    uint8_t pos = 0;

    if (!state->line_detected) {
        return "---";
    }

    e = state->error;
    if (e >= 0) {
        sign = '+';
        mag = (uint8_t)e;
    } else {
        sign = '-';
        mag = (uint8_t)(-e);
    }

    buf[pos++] = 'E';
    buf[pos++] = ':';
    buf[pos++] = sign;
    buf[pos++] = (mag >= 100) ? (char)('0' + (mag / 100)) : '0';
    mag %= 100;
    buf[pos++] = (char)('0' + (mag / 10));
    buf[pos++] = (char)('0' + (mag % 10));

    if (state->center_hit) {
        buf[pos++] = ' ';
        buf[pos++] = 'H';
        buf[pos++] = 'I';
        buf[pos++] = 'T';
    }
    buf[pos] = '\0';

    return buf;
}
