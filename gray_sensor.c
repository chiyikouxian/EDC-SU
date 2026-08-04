#include "gray_sensor.h"
#include "ti_msp_dl_config.h"

#define IR_I2C_ADDRESS        0x5CU
#define IR_DIGITAL_REG        5U
#define IR_I2C_SCL_PORT       GRAY_SENSOR_PORT
#define IR_I2C_SCL_PIN        GRAY_SENSOR_AD1_PIN
#define IR_I2C_SDA_PORT       GRAY_SENSOR_PORT
#define IR_I2C_SDA_PIN        GRAY_SENSOR_AD2_PIN

static void i2c_delay(void)
{
    delay_cycles((CPUCLK_FREQ / 1000000U) * GRAY_I2C_DELAY_US);
}

static void line_low(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_clearPins(port, pin);
    DL_GPIO_enableOutput(port, pin);
    i2c_delay();
}

static void line_release(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_disableOutput(port, pin);
    i2c_delay();
}

static uint8_t line_read(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U) ? 1U : 0U;
}

static void scl_low(void)
{
    line_low(IR_I2C_SCL_PORT, IR_I2C_SCL_PIN);
}

static void scl_release(void)
{
    line_release(IR_I2C_SCL_PORT, IR_I2C_SCL_PIN);
}

static void sda_low(void)
{
    line_low(IR_I2C_SDA_PORT, IR_I2C_SDA_PIN);
}

static void sda_release(void)
{
    line_release(IR_I2C_SDA_PORT, IR_I2C_SDA_PIN);
}

static uint8_t sda_read(void)
{
    return line_read(IR_I2C_SDA_PORT, IR_I2C_SDA_PIN);
}

static void i2c_start(void)
{
    sda_release();
    scl_release();
    sda_low();
    scl_low();
}

static void i2c_stop(void)
{
    sda_low();
    scl_release();
    sda_release();
}

static uint8_t i2c_write_byte(uint8_t value)
{
    uint8_t bit;
    uint8_t ack;

    for (bit = 0U; bit < 8U; bit++) {
        if ((value & 0x80U) != 0U) {
            sda_release();
        } else {
            sda_low();
        }
        scl_release();
        scl_low();
        value <<= 1;
    }

    sda_release();
    scl_release();
    ack = (sda_read() == 0U) ? 1U : 0U;
    scl_low();
    return ack;
}

static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t bit;
    uint8_t value = 0U;

    sda_release();
    for (bit = 0U; bit < 8U; bit++) {
        value <<= 1;
        scl_release();
        if (sda_read() != 0U) {
            value |= 1U;
        }
        scl_low();
    }

    if (ack) {
        sda_low();
    } else {
        sda_release();
    }
    scl_release();
    scl_low();
    sda_release();

    return value;
}

static uint8_t ir_read_digital_byte(uint8_t *value)
{
    i2c_start();
    if (!i2c_write_byte((uint8_t)(IR_I2C_ADDRESS << 1))) {
        i2c_stop();
        return 0U;
    }
    if (!i2c_write_byte(IR_DIGITAL_REG)) {
        i2c_stop();
        return 0U;
    }

    i2c_start();
    if (!i2c_write_byte((uint8_t)((IR_I2C_ADDRESS << 1) | 1U))) {
        i2c_stop();
        return 0U;
    }

    *value = i2c_read_byte(0U);
    i2c_stop();
    return 1U;
}

static bool last_read_ok;

void gray_sensor_init(void)
{
    /* Keep both software-I2C lines as released, pulled-up inputs until the
     * transaction code temporarily enables the output driver for a low bit. */
    DL_GPIO_initDigitalInputFeatures(GRAY_SENSOR_AD1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GRAY_SENSOR_AD2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    sda_release();
    scl_release();
    last_read_ok = false;
}

void gray_sensor_read_all(uint8_t values[GRAY_SENSOR_CHANNELS])
{
    uint8_t ch;
    uint8_t digital = 0U;

    last_read_ok = ir_read_digital_byte(&digital) != 0U;
    if (!last_read_ok) {
        for (ch = 0U; ch < GRAY_SENSOR_CHANNELS; ch++) {
            values[ch] = 0U;
        }
        return;
    }

    for (ch = 0U; ch < GRAY_SENSOR_CHANNELS; ch++) {
        values[ch] = ((digital & (uint8_t)(1U << ch)) != 0U) ? 1U : 0U;
    }
}

bool gray_sensor_read_ok(void)
{
    return last_read_ok;
}

bool gray_sensor_has_line(const uint8_t values[GRAY_SENSOR_CHANNELS])
{
    uint8_t ch;
    for (ch = 0U; ch < GRAY_SENSOR_CHANNELS; ch++) {
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
    static char buf[48];
    uint8_t ch;
    uint16_t pos = 0U;

    for (ch = 0U; ch < GRAY_SENSOR_CHANNELS; ch++) {
        buf[pos++] = 'X';
        buf[pos++] = (char)('1' + ch);
        buf[pos++] = ':';
        buf[pos++] = values[ch] ? '1' : '0';
        if (ch < GRAY_SENSOR_CHANNELS - 1U) {
            buf[pos++] = ' ';
        }
    }
    buf[pos] = '\0';
    return buf;
}
