#include "oled_stopwatch.h"

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define OLED_WIDTH                 128U
#define OLED_PAGES                   8U
#define OLED_I2C_WRITE_ADDRESS     0x78U
#define OLED_COLUMN_OFFSET           2U
#define OLED_I2C_DELAY_US            2U
#define OLED_POWER_ON_DELAY_MS     100U
#define STOPWATCH_REFRESH_MS       100U
#define STOPWATCH_MAX_MS       5999999UL

/* Software I2C costs roughly 54 us per byte at OLED_I2C_DELAY_US = 2, so a
   blocking full-frame push (~1064 bytes) would stall the 1 ms main loop for
   ~58 ms and stretch the 10 ms control tick that drives line tracking and
   the speed PID. Each oled_stopwatch_process() call therefore emits at most
   this many payload bytes and returns. */
#define OLED_TX_BYTE_BUDGET           3U

typedef enum {
    OLED_TX_IDLE = 0,
    OLED_TX_CURSOR,
    OLED_TX_HEADER,
    OLED_TX_DATA
} OledTxState_t;

static uint8_t framebuffer[OLED_PAGES][OLED_WIDTH];
/* Last bytes actually shown by the panel: only differing columns are sent. */
static uint8_t shadow[OLED_PAGES][OLED_WIDTH];
static volatile uint32_t system_ms;
static volatile uint32_t elapsed_ms;
static volatile bool stopwatch_running;
static volatile bool has_measurement;
static uint32_t last_refresh_ms;
static volatile bool refresh_requested;
static bool frame_pending;

static OledTxState_t tx_state;
static uint8_t tx_queue[5];
static uint8_t tx_queue_length;
static uint8_t tx_queue_index;
static uint8_t tx_page;
static uint8_t tx_column;
static uint8_t tx_span_end;

static void i2c_delay(void)
{
    delay_cycles((CPUCLK_FREQ / 1000000U) * OLED_I2C_DELAY_US);
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

static void scl_low(void)
{
    line_low(OLED_I2C_SCL_PORT, OLED_I2C_SCL_PIN);
}

static void scl_release(void)
{
    line_release(OLED_I2C_SCL_PORT, OLED_I2C_SCL_PIN);
}

static void sda_low(void)
{
    line_low(OLED_I2C_SDA_PORT, OLED_I2C_SDA_PIN);
}

static void sda_release(void)
{
    line_release(OLED_I2C_SDA_PORT, OLED_I2C_SDA_PIN);
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

static void i2c_write_byte(uint8_t value)
{
    uint8_t bit;

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

    /* Release SDA for the slave ACK clock. */
    sda_release();
    scl_release();
    scl_low();
}

static void oled_write_command(uint8_t command)
{
    i2c_start();
    i2c_write_byte(OLED_I2C_WRITE_ADDRESS);
    i2c_write_byte(0x00U);
    i2c_write_byte(command);
    i2c_stop();
}

static void oled_set_cursor(uint8_t page, uint8_t x)
{
    x = (uint8_t)(x + OLED_COLUMN_OFFSET);

    i2c_start();
    i2c_write_byte(OLED_I2C_WRITE_ADDRESS);
    i2c_write_byte(0x00U);
    i2c_write_byte((uint8_t)(0xB0U | page));
    i2c_write_byte((uint8_t)(0x10U | (x >> 4)));
    i2c_write_byte((uint8_t)(x & 0x0FU));
    i2c_stop();
}

/* Blocking full-frame push. Used only from oled_stopwatch_init(), before the
   control loop is running; the steady-state path is oled_flush_step(). */
static void oled_update_blocking(void)
{
    uint8_t page;
    uint8_t x;

    for (page = 0U; page < OLED_PAGES; page++) {
        oled_set_cursor(page, 0U);
        i2c_start();
        i2c_write_byte(OLED_I2C_WRITE_ADDRESS);
        i2c_write_byte(0x40U);
        for (x = 0U; x < OLED_WIDTH; x++) {
            i2c_write_byte(framebuffer[page][x]);
        }
        i2c_stop();
    }
    (void)memcpy(shadow, framebuffer, sizeof(shadow));
}

/* Finds the next page holding changed columns, starting at tx_page. Returns
   false once the whole frame matches the panel contents. */
static bool oled_find_dirty_span(void)
{
    while (tx_page < OLED_PAGES) {
        uint8_t first = 0U;
        uint8_t last = 0U;
        bool found = false;
        uint8_t x;

        for (x = 0U; x < OLED_WIDTH; x++) {
            if (framebuffer[tx_page][x] != shadow[tx_page][x]) {
                if (!found) {
                    first = x;
                    found = true;
                }
                last = x;
            }
        }
        if (found) {
            tx_column = first;
            tx_span_end = last;
            return true;
        }
        tx_page++;
    }
    return false;
}

static void oled_queue_cursor(uint8_t page, uint8_t x)
{
    x = (uint8_t)(x + OLED_COLUMN_OFFSET);

    tx_queue[0] = OLED_I2C_WRITE_ADDRESS;
    tx_queue[1] = 0x00U;
    tx_queue[2] = (uint8_t)(0xB0U | page);
    tx_queue[3] = (uint8_t)(0x10U | (x >> 4));
    tx_queue[4] = (uint8_t)(x & 0x0FU);
    tx_queue_length = 5U;
    tx_queue_index = 0U;
}

/* Emits a bounded slice of the pending frame. The I2C transaction is left
   open across calls: the SSD1306 is a passive slave clocked entirely by this
   master, and no other device shares OLED_I2C_SCL/SDA, so pausing between
   bytes is safe. */
static void oled_flush_step(void)
{
    uint8_t budget = OLED_TX_BYTE_BUDGET;

    while (budget > 0U) {
        switch (tx_state) {
            case OLED_TX_IDLE:
                if (!frame_pending) {
                    return;
                }
                if (!oled_find_dirty_span()) {
                    frame_pending = false;
                    tx_page = 0U;
                    return;
                }
                i2c_start();
                oled_queue_cursor(tx_page, tx_column);
                tx_state = OLED_TX_CURSOR;
                break;

            case OLED_TX_CURSOR:
                while ((budget > 0U) && (tx_queue_index < tx_queue_length)) {
                    i2c_write_byte(tx_queue[tx_queue_index++]);
                    budget--;
                }
                if (tx_queue_index >= tx_queue_length) {
                    i2c_stop();
                    i2c_start();
                    tx_queue[0] = OLED_I2C_WRITE_ADDRESS;
                    tx_queue[1] = 0x40U;
                    tx_queue_length = 2U;
                    tx_queue_index = 0U;
                    tx_state = OLED_TX_HEADER;
                }
                break;

            case OLED_TX_HEADER:
                while ((budget > 0U) && (tx_queue_index < tx_queue_length)) {
                    i2c_write_byte(tx_queue[tx_queue_index++]);
                    budget--;
                }
                if (tx_queue_index >= tx_queue_length) {
                    tx_state = OLED_TX_DATA;
                }
                break;

            case OLED_TX_DATA:
            default:
                while ((budget > 0U) && (tx_column <= tx_span_end)) {
                    uint8_t value = framebuffer[tx_page][tx_column];
                    i2c_write_byte(value);
                    shadow[tx_page][tx_column] = value;
                    tx_column++;
                    budget--;
                }
                if (tx_column > tx_span_end) {
                    i2c_stop();
                    tx_page++;
                    tx_state = OLED_TX_IDLE;
                }
                break;
        }
    }
}

static void oled_init(void)
{
    static const uint8_t init_commands[] = {
        0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
        0xA1U, 0xC8U, 0xDAU, 0x12U, 0x81U, 0xCFU, 0xD9U, 0xF1U,
        0xDBU, 0x30U, 0xA4U, 0xA6U, 0x8DU, 0x14U, 0xAFU
    };
    uint8_t i;

    DL_GPIO_clearPins(OLED_I2C_SCL_PORT, OLED_I2C_SCL_PIN);
    DL_GPIO_clearPins(OLED_I2C_SDA_PORT, OLED_I2C_SDA_PIN);
    scl_release();
    sda_release();
    delay_cycles((CPUCLK_FREQ / 1000U) * OLED_POWER_ON_DELAY_MS);

    for (i = 0U; i < sizeof(init_commands); i++) {
        oled_write_command(init_commands[i]);
    }
}

static void draw_pixel(uint8_t x, uint8_t y)
{
    if (x < OLED_WIDTH && y < (OLED_PAGES * 8U)) {
        framebuffer[y >> 3][x] |= (uint8_t)(1U << (y & 7U));
    }
}

static const uint8_t *glyph_for(char character)
{
    static const uint8_t digits[10][5] = {
        {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
        {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
        {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
        {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
        {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
        {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
        {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
        {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
        {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
        {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}
    };
    static const uint8_t colon[5] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
    static const uint8_t period[5] = {0x00U, 0x60U, 0x60U, 0x00U, 0x00U};
    static const uint8_t blank[5] = {0U, 0U, 0U, 0U, 0U};

    if (character >= '0' && character <= '9') {
        return digits[(uint8_t)(character - '0')];
    }
    if (character == ':') {
        return colon;
    }
    if (character == '.') {
        return period;
    }
    return blank;
}

static const uint8_t *status_glyph_for(char character)
{
    static const uint8_t a[5] = {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU};
    static const uint8_t d[5] = {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU};
    static const uint8_t e[5] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U};
    static const uint8_t n[5] = {0x7FU, 0x02U, 0x0CU, 0x10U, 0x7FU};
    static const uint8_t o[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU};
    static const uint8_t p[5] = {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U};
    static const uint8_t r[5] = {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U};
    static const uint8_t s[5] = {0x46U, 0x49U, 0x49U, 0x49U, 0x31U};
    static const uint8_t t[5] = {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U};
    static const uint8_t u[5] = {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU};
    static const uint8_t y[5] = {0x07U, 0x08U, 0x70U, 0x08U, 0x07U};
    static const uint8_t blank[5] = {0U, 0U, 0U, 0U, 0U};

    switch (character) {
        case 'A': return a;
        case 'D': return d;
        case 'E': return e;
        case 'N': return n;
        case 'O': return o;
        case 'P': return p;
        case 'R': return r;
        case 'S': return s;
        case 'T': return t;
        case 'U': return u;
        case 'Y': return y;
        default: return blank;
    }
}

static void draw_glyph(uint8_t x, uint8_t y, const uint8_t *glyph,
                       uint8_t scale)
{
    uint8_t column;
    uint8_t row;
    uint8_t dx;
    uint8_t dy;

    for (column = 0U; column < 5U; column++) {
        for (row = 0U; row < 7U; row++) {
            if ((glyph[column] & (1U << row)) == 0U) {
                continue;
            }
            for (dx = 0U; dx < scale; dx++) {
                for (dy = 0U; dy < scale; dy++) {
                    draw_pixel((uint8_t)(x + column * scale + dx),
                               (uint8_t)(y + row * scale + dy));
                }
            }
        }
    }
}

static void draw_status(const char *text)
{
    uint8_t length = (uint8_t)strlen(text);
    uint8_t x = (uint8_t)((OLED_WIDTH - length * 6U) / 2U);

    while (*text != '\0') {
        draw_glyph(x, 2U, status_glyph_for(*text++), 1U);
        x = (uint8_t)(x + 6U);
    }
}

static void render_stopwatch(void)
{
    uint32_t shown_ms = elapsed_ms;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t hundredths;
    char time_text[9];
    uint8_t i;

    if (shown_ms > STOPWATCH_MAX_MS) {
        shown_ms = STOPWATCH_MAX_MS;
    }
    minutes = (uint8_t)(shown_ms / 60000UL);
    seconds = (uint8_t)((shown_ms / 1000UL) % 60UL);
    hundredths = (uint8_t)((shown_ms / 10UL) % 100UL);

    time_text[0] = (char)('0' + minutes / 10U);
    time_text[1] = (char)('0' + minutes % 10U);
    time_text[2] = ':';
    time_text[3] = (char)('0' + seconds / 10U);
    time_text[4] = (char)('0' + seconds % 10U);
    time_text[5] = '.';
    time_text[6] = (char)('0' + hundredths / 10U);
    time_text[7] = (char)('0' + hundredths % 10U);
    time_text[8] = '\0';

    (void)memset(framebuffer, 0, sizeof(framebuffer));
    if (stopwatch_running) {
        draw_status("RUN");
    } else if (has_measurement) {
        draw_status("STOP");
    } else {
        draw_status("READY");
    }

    for (i = 0U; i < 8U; i++) {
        draw_glyph((uint8_t)(i * 16U), 23U, glyph_for(time_text[i]), 3U);
    }
    /* Hand the frame to the chunked transmitter instead of blocking here. */
    frame_pending = true;
    tx_page = 0U;
}

void oled_stopwatch_init(void)
{
    system_ms = 0U;
    elapsed_ms = 0U;
    stopwatch_running = false;
    has_measurement = false;
    last_refresh_ms = 0U;
    refresh_requested = false;
    frame_pending = false;
    tx_state = OLED_TX_IDLE;
    tx_queue_length = 0U;
    tx_queue_index = 0U;
    tx_page = 0U;
    tx_column = 0U;
    tx_span_end = 0U;

    oled_init();
    /* Paint the initial READY frame while blocking is still harmless, and
       seed the shadow buffer so later frames only send real differences. */
    (void)memset(shadow, 0xFFU, sizeof(shadow));
    render_stopwatch();
    frame_pending = false;
    oled_update_blocking();
}

void oled_stopwatch_process(void)
{
    uint32_t now = system_ms;

    /* Only start a new frame once the previous one is fully on the panel, so
       a partially transmitted frame is never overwritten mid-span. */
    if ((tx_state == OLED_TX_IDLE) && !frame_pending) {
        if (refresh_requested ||
            (stopwatch_running &&
             ((uint32_t)(now - last_refresh_ms) >= STOPWATCH_REFRESH_MS))) {
            refresh_requested = false;
            last_refresh_ms = now;
            render_stopwatch();
        }
    }

    oled_flush_step();
}

bool oled_stopwatch_is_running(void)
{
    return stopwatch_running;
}

uint32_t oled_stopwatch_elapsed_ms(void)
{
    return elapsed_ms;
}

void oled_stopwatch_reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    elapsed_ms = 0U;
    stopwatch_running = false;
    has_measurement = false;
    refresh_requested = true;
    if (primask == 0U) {
        __enable_irq();
    }
}

void oled_stopwatch_start(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    elapsed_ms = 0U;
    has_measurement = false;
    stopwatch_running = true;
    refresh_requested = true;
    if (primask == 0U) {
        __enable_irq();
    }
}

void oled_stopwatch_stop(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    stopwatch_running = false;
    has_measurement = true;
    refresh_requested = true;
    if (primask == 0U) {
        __enable_irq();
    }
}

void STOPWATCH_TIMER_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(STOPWATCH_TIMER_INST) ==
        DL_TIMER_IIDX_ZERO) {
        system_ms++;

        if (stopwatch_running) {
            if (elapsed_ms < STOPWATCH_MAX_MS) {
                elapsed_ms++;
            } else {
                stopwatch_running = false;
                has_measurement = true;
                refresh_requested = true;
            }
        }
    }
}
