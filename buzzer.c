#include "buzzer.h"
#include "ti_msp_dl_config.h"

#define BEEP_ON_TICKS   (200 / TICK_MS)
#define BEEP_OFF_TICKS  (200 / TICK_MS)

static uint8_t beep_total;
static uint8_t beep_count;
static uint16_t tick_counter;
static bool     is_on;
static bool     sequence_done;
static bool     last_beep_started;

static void buzzer_hw_on(void)
{
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BUZZ_PIN);
}

static void buzzer_hw_off(void)
{
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_BUZZ_PIN);
}

void buzzer_init(void)
{
    buzzer_hw_off();
    beep_total     = 0;
    beep_count     = 0;
    tick_counter   = 0;
    is_on          = false;
    sequence_done  = true;
    last_beep_started = false;
}

void buzzer_start_sequence(uint8_t count)
{
    if (count == 0) {
        sequence_done = true;
        return;
    }
    beep_total     = count;
    beep_count     = 0;
    tick_counter   = 0;
    is_on          = true;
    sequence_done  = false;
    last_beep_started = false;

    if (count == 1) {
        last_beep_started = true;
    }
    buzzer_hw_on();
}

void buzzer_tick(void)
{
    if (sequence_done) return;

    tick_counter++;

    if (is_on) {
        if (tick_counter >= BEEP_ON_TICKS) {
            buzzer_hw_off();
            is_on = false;
            tick_counter = 0;
            beep_count++;
            if (beep_count >= beep_total) {
                sequence_done = true;
            }
        }
    } else {
        if (tick_counter >= BEEP_OFF_TICKS) {
            if (beep_count + 1 == beep_total) {
                last_beep_started = true;
            }
            buzzer_hw_on();
            is_on = true;
            tick_counter = 0;
        }
    }
}

bool buzzer_is_done(void)
{
    return sequence_done;
}

bool buzzer_on_last_beep_start(void)
{
    return last_beep_started;
}
