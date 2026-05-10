#include "track_bridge.h"
#include "line_track.h"

/*
 * Single static instance — the entire bridge state.
 * track_bridge_get() returns a const pointer to this, so callers never
 * allocate, copy, or block.
 */
static TrackBridgeData_t bridge;

void track_bridge_init(void)
{
    bridge.error         = 0;
    bridge.active_count  = 0;
    bridge.line_detected = false;
    bridge.all_white     = true;
    bridge.center_hit    = false;
    bridge.turn_hint     = LINE_TRACK_TURN_HINT_NONE;
    bridge.tick          = 0;
    bridge.stale         = true;
}

void track_bridge_mark_stale(void)
{
    bridge.stale = true;
}

void track_bridge_update(const LineTrackState_t *state, uint32_t tick)
{
    bridge.error         = state->error;
    bridge.active_count  = state->active_count;
    bridge.line_detected = state->line_detected;
    bridge.all_white     = state->all_white;
    bridge.center_hit    = state->center_hit;
    bridge.turn_hint     = state->turn_hint;
    bridge.tick          = tick;
    bridge.stale         = false;
}

const TrackBridgeData_t *track_bridge_get(void)
{
    return &bridge;
}

const char *track_bridge_debug_string(void)
{
    static char buf[16];

    if (bridge.stale) {
        return "STALE";
    }

    if (!bridge.line_detected) {
        return "---";
    }

    /* Format: "E:sMMM" optionally " HIT" — identical to L= convention. */
    {
        int16_t e  = bridge.error;
        char    sign;
        uint8_t mag;

        if (e >= 0) {
            sign = '+';
            mag  = (uint8_t)e;
        } else {
            sign = '-';
            mag  = (uint8_t)(-e);
        }

        {
            uint8_t pos = 0;
            buf[pos++] = 'E';
            buf[pos++] = ':';
            buf[pos++] = sign;

            if (mag >= 100) {
                buf[pos++] = '0' + (mag / 100);
                mag = mag % 100;
            } else {
                buf[pos++] = '0';
            }

            buf[pos++] = '0' + (mag / 10);
            buf[pos++] = '0' + (mag % 10);

            if (bridge.center_hit) {
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
