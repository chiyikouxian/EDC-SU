#include "vision_uart.h"
#include "ti_msp_dl_config.h"
#include <string.h>

/*
 * Pure interrupt reception architecture:
 * VISION_UART_INST_IRQHandler is the SOLE byte feeder into the parser.
 * There is no polling path - app_state only consumes parsed results.
 * Parser state stays ISR-private, and result handoff uses a short IRQ mask
 * to avoid torn reads or clearing a frame while a new one is being published.
 */

#define FRAME_BUF_SIZE  40

typedef enum {
    RX_WAIT_START,
    RX_COLLECT
} RxState_t;

/* volatile: shared between ISR (writer) and main loop (reader) */
static volatile VisionResult_t last_result;
static volatile bool           has_result;

/* ISR-private: accessed ONLY from VISION_UART_INST_IRQHandler */
static RxState_t rx_state;
static char      rx_buf[FRAME_BUF_SIZE];
static uint8_t   rx_len;

static void vision_uart_result_lock(void)
{
    NVIC_DisableIRQ(VISION_UART_INST_INT_IRQN);
}

static void vision_uart_result_unlock(void)
{
    NVIC_EnableIRQ(VISION_UART_INST_INT_IRQN);
}

/* ---- string compare helper (no stdlib dependency beyond string.h) ---- */

static bool str_eq(const char *a, const char *b)
{
    while (*a && *b) {
        if (*a != *b) return false;
        a++; b++;
    }
    return *a == *b;
}

/* ---- field splitter: finds up to max_fields comma-separated tokens ---- */

static int split_fields(char *line, char *fields[], int max_fields)
{
    int count = 0;
    char *p = line;
    while (*p && count < max_fields) {
        fields[count++] = p;
        while (*p && *p != ',') p++;
        if (*p == ',') { *p = '\0'; p++; }
    }
    return count;
}

/* ---- mapping tables ---- */

typedef struct {
    const char   *name;
    Shape_t       shape;
    TargetPoint_t expected_target;
    bool          expected_valid;
} ShapeEntry_t;

static const ShapeEntry_t shape_table[] = {
    { "CIRCLE",   SHAPE_CIRCLE,   TARGET_A,    true  },
    { "TRIANGLE", SHAPE_TRIANGLE, TARGET_B,    true  },
    { "RECT",     SHAPE_RECT,     TARGET_C,    true  },
    { "PENTAGON", SHAPE_PENTAGON, TARGET_D,    true  },
    { "NONE",     SHAPE_NONE,     TARGET_NONE, false },
};

#define SHAPE_TABLE_LEN  (sizeof(shape_table) / sizeof(shape_table[0]))

static TargetPoint_t parse_target_char(char c)
{
    switch (c) {
        case 'A': return TARGET_A;
        case 'B': return TARGET_B;
        case 'C': return TARGET_C;
        case 'D': return TARGET_D;
        case 'X': return TARGET_NONE;
        default:  return (TargetPoint_t)0xFF;
    }
}

/* ---- frame parser ---- */

static void process_frame(void)
{
    char *fields[3];
    int n = split_fields(rx_buf, fields, 3);
    if (n != 3) return;

    /* field 1: shape name */
    const ShapeEntry_t *entry = NULL;
    uint8_t i;
    for (i = 0; i < SHAPE_TABLE_LEN; i++) {
        if (str_eq(fields[0], shape_table[i].name)) {
            entry = &shape_table[i];
            break;
        }
    }
    if (entry == NULL) return;

    /* field 2: target letter (must be single char) */
    if (fields[1][0] == '\0' || fields[1][1] != '\0') return;
    TargetPoint_t tgt = parse_target_char(fields[1][0]);
    if ((uint8_t)tgt == 0xFF) return;

    /* field 3: valid flag (must be '0' or '1', single char) */
    if (fields[2][0] == '\0' || fields[2][1] != '\0') return;
    bool frame_valid;
    if (fields[2][0] == '1')      frame_valid = true;
    else if (fields[2][0] == '0') frame_valid = false;
    else return;

    /* strict cross-check: shape/target/valid must match mapping table */
    if (tgt != entry->expected_target) return;
    if (frame_valid != entry->expected_valid) return;

    /* all checks passed */
    last_result.shape  = entry->shape;
    last_result.target = tgt;
    last_result.valid  = frame_valid;
    has_result = true;
}

/* ---- public API ---- */

void vision_uart_init(void)
{
    last_result.shape  = SHAPE_NONE;
    last_result.target = TARGET_NONE;
    last_result.valid  = false;
    has_result = false;
    rx_state   = RX_WAIT_START;
    rx_len     = 0;

    DL_UART_Main_enableInterrupt(VISION_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(VISION_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(VISION_UART_INST_INT_IRQN);
}

/* VISION_UART RX interrupt handler - drains RX FIFO and feeds parser */
void VISION_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(VISION_UART_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_isRXFIFOEmpty(VISION_UART_INST)) {
            uint8_t byte = DL_UART_receiveData(VISION_UART_INST);
            vision_uart_feed_byte(byte);
        }
        break;
    default:
        break;
    }
}

void vision_uart_feed_byte(uint8_t byte)
{
    char c = (char)byte;

    switch (rx_state) {
    case RX_WAIT_START:
        if (c == '$') {
            rx_len   = 0;
            rx_state = RX_COLLECT;
        }
        break;

    case RX_COLLECT:
        if (c == '$') {
            rx_len   = 0;
            break;
        }
        if (c == '\r') {
            break;
        }
        if (c == '\n') {
            rx_buf[rx_len] = '\0';
            process_frame();
            rx_state = RX_WAIT_START;
            rx_len   = 0;
            break;
        }
        if (rx_len < FRAME_BUF_SIZE - 1) {
            rx_buf[rx_len++] = c;
        } else {
            rx_state = RX_WAIT_START;
            rx_len   = 0;
        }
        break;
    }
}

bool vision_uart_has_result(void)
{
    bool result_ready;

    vision_uart_result_lock();
    result_ready = has_result;
    vision_uart_result_unlock();

    return result_ready;
}

VisionResult_t vision_uart_get_result(void)
{
    VisionResult_t result;

    vision_uart_result_lock();
    result = (VisionResult_t)last_result;
    vision_uart_result_unlock();

    return result;
}

bool vision_uart_take_result(VisionResult_t *out)
{
    bool taken = false;

    vision_uart_result_lock();
    if (has_result) {
        if (out != NULL) {
            *out = (VisionResult_t)last_result;
        }
        has_result = false;
        taken = true;
    }
    vision_uart_result_unlock();

    return taken;
}

void vision_uart_clear(void)
{
    vision_uart_result_lock();
    has_result = false;
    last_result.shape  = SHAPE_NONE;
    last_result.target = TARGET_NONE;
    last_result.valid  = false;
    vision_uart_result_unlock();
}
