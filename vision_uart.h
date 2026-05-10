#ifndef VISION_UART_H
#define VISION_UART_H

#include "app_common.h"

typedef struct {
    Shape_t       shape;
    TargetPoint_t target;
    bool          valid;
} VisionResult_t;

void vision_uart_init(void);
void vision_uart_feed_byte(uint8_t byte);
bool vision_uart_has_result(void);
VisionResult_t vision_uart_get_result(void);
bool vision_uart_take_result(VisionResult_t *out);
void vision_uart_clear(void);

#endif /* VISION_UART_H */
