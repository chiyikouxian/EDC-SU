#include "start_key.h"
#include "ti_msp_dl_config.h"

#define START_KEY_SCAN_MS         1
#define START_KEY_DEBOUNCE_TICKS  (30 / START_KEY_SCAN_MS)

static uint16_t hold_ticks;
static bool     triggered;
static bool     armed;
static bool     released_once;

void start_key_init(void)
{
    hold_ticks    = 0;
    triggered     = false;
    armed         = false;
    released_once = false;
}

void start_key_scan(void)
{
    bool pressed = !DL_GPIO_readPins(START_KEY_PORT, START_KEY_BTN_PIN);

    if (!pressed) {
        hold_ticks    = 0;
        armed         = false;
        released_once = true;
        return;
    }

    if (!released_once) {
        return;
    }

    if (!armed) {
        hold_ticks++;
        if (hold_ticks >= START_KEY_DEBOUNCE_TICKS) {
            triggered = true;
            armed     = true;
        }
    }
}

bool start_key_triggered(void)
{
    if (triggered) {
        triggered = false;
        return true;
    }
    return false;
}
