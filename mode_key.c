#include "mode_key.h"
#include "ti_msp_dl_config.h"

/* Trigger on the first sampled press; release-to-rearm prevents repeats. */
#define MODE_KEY_DEBOUNCE_TICKS 1U

static uint16_t hold_ticks;
static bool triggered;
static bool armed;
static bool released_once;
static bool released_level;

void mode_key_init(void)
{
    hold_ticks = 0U;
    triggered = false;
    armed = false;
    released_level =
        (DL_GPIO_readPins(MODE_KEY_PORT, MODE_KEY_KEY_PIN) != 0U);
    released_once = true;
}

void mode_key_scan(void)
{
    bool pin_level =
        (DL_GPIO_readPins(MODE_KEY_PORT, MODE_KEY_KEY_PIN) != 0U);
    bool pressed = (pin_level != released_level);

    if (!pressed) {
        hold_ticks = 0U;
        armed = false;
        released_once = true;
        return;
    }

    if (!released_once) {
        return;
    }

    if (!armed) {
        if (hold_ticks < MODE_KEY_DEBOUNCE_TICKS) {
            hold_ticks++;
        }
        if (hold_ticks >= MODE_KEY_DEBOUNCE_TICKS) {
            triggered = true;
            armed = true;
        }
    }
}

bool mode_key_triggered(void)
{
    if (triggered) {
        triggered = false;
        return true;
    }
    return false;
}

RunMode_t mode_key_get_mode(void)
{
    return RUN_MODE_BASIC_DRIVE;
}
