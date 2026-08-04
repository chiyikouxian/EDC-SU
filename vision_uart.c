#include "vision_uart.h"

#include <stddef.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

/*
 * Ball-position telemetry receiver. The ISR only moves bytes into a ring
 * buffer; line assembly and parsing run in vision_uart_process() on the main
 * loop, so a malformed or overlong line cannot stretch interrupt latency.
 */

#define VISION_RX_BUFFER_SIZE       512U
#define VISION_RX_BUFFER_MASK       (VISION_RX_BUFFER_SIZE - 1U)
#define VISION_LINE_BUFFER_SIZE     192U
#define VISION_TELEMETRY_TIMEOUT_MS 250U

static volatile uint8_t rx_buffer[VISION_RX_BUFFER_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_drop_count;
static volatile bool rx_discard_requested;

static char line_buffer[VISION_LINE_BUFFER_SIZE];
static uint16_t line_length;
static bool line_overflow;

volatile VisionTelemetry_t g_vision_telemetry;

static bool text_equals(const char *a, const char *b)
{
    while ((*a != '\0') && (*b != '\0')) {
        if (*a != *b) {
            return false;
        }
        a++;
        b++;
    }
    return *a == *b;
}

static bool parse_u64(const char *text, uint64_t *out)
{
    uint64_t value = 0U;
    uint8_t digits = 0U;

    if ((text == NULL) || (out == NULL) || (*text == '\0')) {
        return false;
    }
    while (*text != '\0') {
        uint8_t digit;
        if ((*text < '0') || (*text > '9')) {
            return false;
        }
        digit = (uint8_t)(*text - '0');
        if (value > (UINT64_MAX - digit) / 10U) {
            return false;
        }
        value = value * 10U + digit;
        digits++;
        text++;
    }
    if (digits == 0U) {
        return false;
    }
    *out = value;
    return true;
}

static bool parse_i32(const char *text, int32_t *out)
{
    bool negative = false;
    uint32_t value = 0U;
    uint8_t digits = 0U;
    uint32_t limit;

    if ((text == NULL) || (out == NULL) || (*text == '\0')) {
        return false;
    }
    if (*text == '-') {
        negative = true;
        text++;
    } else if (*text == '+') {
        text++;
    }
    if (*text == '\0') {
        return false;
    }
    limit = negative ? 2147483648U : 2147483647U;
    while (*text != '\0') {
        uint8_t digit;
        if ((*text < '0') || (*text > '9')) {
            return false;
        }
        digit = (uint8_t)(*text - '0');
        if (value > (limit - digit) / 10U) {
            return false;
        }
        value = value * 10U + digit;
        digits++;
        text++;
    }
    if (digits == 0U) {
        return false;
    }
    if (negative) {
        if (value == 2147483648U) {
            *out = INT32_MIN;
        } else {
            *out = -(int32_t)value;
        }
    } else {
        *out = (int32_t)value;
    }
    return true;
}

/* Accepts "123" or "123.4" and returns the value scaled by 10. A second
   decimal digit is rejected rather than truncated, so a sender misconfigured
   to more precision fails loudly instead of silently losing resolution. */
static bool parse_fixed_x10(const char *text, int32_t *out)
{
    bool negative = false;
    uint32_t integer_part = 0U;
    uint8_t fraction = 0U;
    uint8_t fraction_digits = 0U;
    uint8_t integer_digits = 0U;
    uint32_t limit;

    if ((text == NULL) || (out == NULL) || (*text == '\0')) {
        return false;
    }
    if (*text == '-') {
        negative = true;
        text++;
    } else if (*text == '+') {
        text++;
    }
    if (*text == '\0') {
        return false;
    }
    limit = negative ? 2147483648U / 10U : 2147483647U / 10U;
    while ((*text >= '0') && (*text <= '9')) {
        uint8_t digit = (uint8_t)(*text - '0');
        if (integer_part > (limit - digit) / 10U) {
            return false;
        }
        integer_part = integer_part * 10U + digit;
        integer_digits++;
        text++;
    }
    if (integer_digits == 0U) {
        return false;
    }
    if (*text == '.') {
        text++;
        if ((*text < '0') || (*text > '9')) {
            return false;
        }
        fraction = (uint8_t)(*text - '0');
        fraction_digits = 1U;
        text++;
        if (*text != '\0') {
            return false;
        }
    } else if (*text != '\0') {
        return false;
    }
    if (fraction_digits == 0U) {
        fraction = 0U;
    }
    if (integer_part > limit) {
        return false;
    }
    {
        uint32_t scaled = integer_part * 10U + fraction;
        if (negative) {
            if (scaled > 2147483648U) {
                return false;
            }
            *out = (scaled == 2147483648U) ? INT32_MIN : -(int32_t)scaled;
        } else {
            if (scaled > 2147483647U) {
                return false;
            }
            *out = (int32_t)scaled;
        }
    }
    return true;
}

static bool parse_bool01(const char *text, bool *out)
{
    if ((text == NULL) || (out == NULL) || (text[0] == '\0') ||
        (text[1] != '\0') || ((text[0] != '0') && (text[0] != '1'))) {
        return false;
    }
    *out = text[0] == '1';
    return true;
}

/* Splits "key=value,key=value,..." in place. All seven keys are mandatory and
   duplicates are rejected, so a partially-formed line can never be mistaken
   for a complete sample. Destroys the input buffer. */
static bool parse_telemetry_line(char *line, VisionTelemetry_t *out)
{
    bool seen_timestamp = false;
    bool seen_valid = false;
    bool seen_position = false;
    bool seen_velocity = false;
    bool seen_x = false;
    bool seen_y = false;
    bool seen_recording = false;
    char *field = line;
    VisionTelemetry_t parsed = {0};

    while (*field != '\0') {
        char *key = field;
        char *next = field;
        char *equals;
        char *value;
        int32_t coordinate;

        while ((*next != '\0') && (*next != ',')) {
            next++;
        }
        if (*next == ',') {
            *next = '\0';
            next++;
        } else {
            next = NULL;
        }
        equals = key;
        while ((*equals != '\0') && (*equals != '=')) {
            equals++;
        }
        if ((*equals != '=') || (equals == key) || (equals[1] == '\0')) {
            return false;
        }
        *equals = '\0';
        value = equals + 1;
        if (text_equals(key, "timestamp")) {
            if (seen_timestamp ||
                !parse_u64(value, &parsed.remote_timestamp_ms)) {
                return false;
            }
            seen_timestamp = true;
        } else if (text_equals(key, "valid")) {
            if (seen_valid || !parse_bool01(value, &parsed.ball_valid)) {
                return false;
            }
            seen_valid = true;
        } else if (text_equals(key, "position_mm")) {
            if (seen_position ||
                !parse_fixed_x10(value, &parsed.position_mm_x10)) {
                return false;
            }
            seen_position = true;
        } else if (text_equals(key, "velocity_mm_s")) {
            if (seen_velocity ||
                !parse_fixed_x10(value, &parsed.velocity_mm_s_x10)) {
                return false;
            }
            seen_velocity = true;
        } else if (text_equals(key, "x")) {
            if (seen_x || !parse_i32(value, &coordinate) ||
                (coordinate < INT16_MIN) || (coordinate > INT16_MAX)) {
                return false;
            }
            parsed.image_x = (int16_t)coordinate;
            seen_x = true;
        } else if (text_equals(key, "y")) {
            if (seen_y || !parse_i32(value, &coordinate) ||
                (coordinate < INT16_MIN) || (coordinate > INT16_MAX)) {
                return false;
            }
            parsed.image_y = (int16_t)coordinate;
            seen_y = true;
        } else if (text_equals(key, "recording")) {
            if (seen_recording || !parse_bool01(value, &parsed.recording)) {
                return false;
            }
            seen_recording = true;
        } else {
            return false;
        }
        if (next == NULL) {
            break;
        }
        field = next;
    }
    if (!(seen_timestamp && seen_valid && seen_position && seen_velocity &&
          seen_x && seen_y && seen_recording)) {
        return false;
    }
    *out = parsed;
    return true;
}

static void process_line(uint32_t now_ms)
{
    VisionTelemetry_t parsed;

    if (parse_telemetry_line(line_buffer, &parsed)) {
        parsed.local_receive_ms = now_ms;
        parsed.sample_count = g_vision_telemetry.sample_count + 1U;
        parsed.link_fresh = true;
        parsed.malformed_count = g_vision_telemetry.malformed_count;
        parsed.overlength_count = g_vision_telemetry.overlength_count;
        parsed.rx_drop_count = rx_drop_count;
        g_vision_telemetry = parsed;
    } else {
        g_vision_telemetry.malformed_count++;
    }
}

void vision_uart_init(void)
{
    rx_head = 0U;
    rx_tail = 0U;
    rx_drop_count = 0U;
    rx_discard_requested = false;
    line_length = 0U;
    line_overflow = false;
    g_vision_telemetry.remote_timestamp_ms = 0U;
    g_vision_telemetry.position_mm_x10 = 0;
    g_vision_telemetry.velocity_mm_s_x10 = 0;
    g_vision_telemetry.image_x = -1;
    g_vision_telemetry.image_y = -1;
    g_vision_telemetry.ball_valid = false;
    g_vision_telemetry.recording = false;
    g_vision_telemetry.link_fresh = false;
    g_vision_telemetry.local_receive_ms = 0U;
    g_vision_telemetry.sample_count = 0U;
    g_vision_telemetry.malformed_count = 0U;
    g_vision_telemetry.overlength_count = 0U;
    g_vision_telemetry.rx_drop_count = 0U;
    DL_UART_Main_disableInterrupt(VISION_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    DL_UART_Main_enableInterrupt(VISION_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(VISION_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(VISION_UART_INST_INT_IRQN);
}

void vision_uart_process(uint32_t now_ms)
{
    while (rx_tail != rx_head) {
        uint8_t byte = rx_buffer[rx_tail];

        rx_tail = (uint16_t)((rx_tail + 1U) & VISION_RX_BUFFER_MASK);
        /* A ring-buffer overrun happened somewhere inside the current line, so
           the line is already corrupt: drop it rather than parse a splice. */
        if (rx_discard_requested) {
            line_overflow = true;
            rx_discard_requested = false;
        }
        if (byte == '\r') {
            continue;
        }
        if (byte == '\n') {
            if (line_overflow) {
                g_vision_telemetry.overlength_count++;
            } else {
                line_buffer[line_length] = '\0';
                process_line(now_ms);
            }
            line_length = 0U;
            line_overflow = false;
        } else if (!line_overflow) {
            if (line_length < (VISION_LINE_BUFFER_SIZE - 1U)) {
                line_buffer[line_length++] = (char)byte;
            } else {
                line_overflow = true;
            }
        }
    }
    if (g_vision_telemetry.link_fresh &&
        ((uint32_t)(now_ms - g_vision_telemetry.local_receive_ms) >
         VISION_TELEMETRY_TIMEOUT_MS)) {
        g_vision_telemetry.link_fresh = false;
    }
    g_vision_telemetry.rx_drop_count = rx_drop_count;
}

bool vision_uart_get_telemetry(VisionTelemetry_t *out)
{
    uint32_t primask;

    if (out == NULL) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *out = (VisionTelemetry_t)g_vision_telemetry;
    if (primask == 0U) {
        __enable_irq();
    }
    return out->link_fresh;
}

void VISION_UART_INST_IRQHandler(void)
{
    DL_UART_IIDX pending = DL_UART_Main_getPendingInterrupt(VISION_UART_INST);

    if (pending == DL_UART_MAIN_IIDX_RX) {
        while (!DL_UART_isRXFIFOEmpty(VISION_UART_INST)) {
            uint16_t next = (uint16_t)((rx_head + 1U) & VISION_RX_BUFFER_MASK);
            uint8_t byte = DL_UART_receiveData(VISION_UART_INST);

            if (next == rx_tail) {
                rx_drop_count++;
                rx_discard_requested = true;
            } else {
                rx_buffer[rx_head] = byte;
                rx_head = next;
            }
        }
    }
}
