#include "mode_key.h"
#include "ti_msp_dl_config.h"

#define DEBOUNCE_TICKS  (30 / TICK_MS)

static RunMode_t current_mode;
static uint16_t  press_ticks;
static bool      was_pressed;

void mode_key_init(void)
{
    current_mode = RUN_MODE_BASIC_DRIVE;
    press_ticks  = 0;
    was_pressed  = false;
}

void mode_key_scan(void)
{
    bool pressed = !DL_GPIO_readPins(MODE_KEY_PORT, MODE_KEY_KEY_PIN);

    if (pressed) {
        if (press_ticks < DEBOUNCE_TICKS) {
            press_ticks++;
        }
    } else {
        if (was_pressed && press_ticks >= DEBOUNCE_TICKS) {
            current_mode = (RunMode_t)((current_mode + 1) % RUN_MODE_COUNT);
        }
        press_ticks = 0;
    }

    was_pressed = pressed && (press_ticks >= DEBOUNCE_TICKS);
}

RunMode_t mode_key_get_mode(void)
{
    return current_mode;
}
