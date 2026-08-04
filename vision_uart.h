#ifndef VISION_UART_H
#define VISION_UART_H

#include <stdbool.h>
#include <stdint.h>

/* Ball-position telemetry from the vision module (Raspberry Pi + camera) on
   UART1 at 115200 8N1. One newline-terminated text line per sample:

     timestamp=<ms>,valid=<0|1>,position_mm=<f.f>,velocity_mm_s=<f.f>,
     x=<int>,y=<int>,recording=<0|1>

   All seven keys are required and each may appear only once; anything else is
   counted as malformed and discarded. Positions and velocities arrive with one
   optional decimal place and are stored scaled by 10 (mm_x10 / mm_s_x10), per
   the project convention of integer-only control paths. */

typedef struct {
    uint64_t remote_timestamp_ms;  /* sender's clock, not comparable to ours */
    int32_t position_mm_x10;
    int32_t velocity_mm_s_x10;
    int16_t image_x;
    int16_t image_y;
    bool ball_valid;               /* sender saw the ball this frame */
    bool recording;
    bool link_fresh;               /* a line arrived within the timeout */
    uint32_t local_receive_ms;     /* our system_time_ms() at line end */
    uint32_t sample_count;         /* increments per accepted line */
    uint32_t malformed_count;
    uint32_t overlength_count;
    uint32_t rx_drop_count;
} VisionTelemetry_t;

/* CCS Watch symbol. Application code should read through the getter below. */
extern volatile VisionTelemetry_t g_vision_telemetry;

void vision_uart_init(void);
void vision_uart_process(uint32_t now_ms);

/* Copies a consistent snapshot; returns the link_fresh flag. */
bool vision_uart_get_telemetry(VisionTelemetry_t *out);

#endif /* VISION_UART_H */
