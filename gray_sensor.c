#include "gray_sensor.h"
#include "ti_msp_dl_config.h"

/* Select one of 8 channels via AD0/AD1/AD2.
   ch: 0..7, maps to X1..X8.
   AD0 = bit 0, AD1 = bit 1, AD2 = bit 2.
   Pins come from ti_msp_dl_config.h (GRAY_SENSOR_ADx_PIN). */
static void gray_select_channel(uint8_t ch)
{
    if (ch & 0x01) DL_GPIO_setPins(  GRAY_SENSOR_PORT, GRAY_SENSOR_AD0_PIN);
    else           DL_GPIO_clearPins(GRAY_SENSOR_PORT, GRAY_SENSOR_AD0_PIN);

    if (ch & 0x02) DL_GPIO_setPins(  GRAY_SENSOR_PORT, GRAY_SENSOR_AD1_PIN);
    else           DL_GPIO_clearPins(GRAY_SENSOR_PORT, GRAY_SENSOR_AD1_PIN);

    if (ch & 0x04) DL_GPIO_setPins(  GRAY_SENSOR_PORT, GRAY_SENSOR_AD2_PIN);
    else           DL_GPIO_clearPins(GRAY_SENSOR_PORT, GRAY_SENSOR_AD2_PIN);
}

static uint8_t gray_read_out(void)
{
    return (DL_GPIO_readPins(GRAY_SENSOR_PORT, GRAY_SENSOR_OUT_PIN) != 0) ? 1 : 0;
}

void gray_sensor_init(void)
{
    DL_GPIO_setPins(GRAY_SENSOR_PORT, GRAY_AD_MASK);
    DL_GPIO_clearPins(GRAY_SENSOR_PORT, GRAY_AD_MASK);
}

void gray_sensor_read_all(uint8_t values[GRAY_SENSOR_CHANNELS])
{
    uint8_t ch;
    for (ch = 0; ch < GRAY_SENSOR_CHANNELS; ch++) {
        gray_select_channel(ch);
        delay_cycles(CPUCLK_FREQ / 1000000 * GRAY_SETTLE_US);
        values[ch] = gray_read_out();
    }
    /* leave AD2/AD1/AD0 driven low after scan */
    DL_GPIO_clearPins(GRAY_SENSOR_PORT, GRAY_AD_MASK);
}

bool gray_sensor_has_line(const uint8_t values[GRAY_SENSOR_CHANNELS])
{
    uint8_t ch;
    for (ch = 0; ch < GRAY_SENSOR_CHANNELS; ch++) {
        if (values[ch] == GRAY_ACTIVE_LEVEL) return true;
    }
    return false;
}

bool gray_sensor_is_all_white(const uint8_t values[GRAY_SENSOR_CHANNELS])
{
    return !gray_sensor_has_line(values);
}

const char *gray_sensor_debug_string(const uint8_t values[GRAY_SENSOR_CHANNELS])
{
    static char buf[64];
    uint8_t ch;
    uint16_t pos = 0;

    for (ch = 0; ch < GRAY_SENSOR_CHANNELS; ch++) {
        /* Format: "X1:1 X2:0 ..." */
        buf[pos++] = 'X';
        buf[pos++] = '1' + ch;
        buf[pos++] = ':';
        buf[pos++] = values[ch] ? '1' : '0';
        if (ch < GRAY_SENSOR_CHANNELS - 1) {
            buf[pos++] = ' ';
        }
    }
    buf[pos] = '\0';
    return buf;
}
