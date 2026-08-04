#ifndef OLED_STOPWATCH_H
#define OLED_STOPWATCH_H

#include <stdbool.h>
#include <stdint.h>

/* 说明 5: the vehicle must carry a start key and a display no larger than
   2 inches; timing starts when the run starts and the elapsed time is shown
   while driving. The application state machine owns start/stop -- start_key.c
   owns the key, so this module never reads it. */

void oled_stopwatch_init(void);

/* Call from the 1 ms main loop. Non-blocking: each call pushes at most a
   small bounded number of software-I2C bytes to the panel. */
void oled_stopwatch_process(void);

void oled_stopwatch_reset(void);
void oled_stopwatch_start(void);
void oled_stopwatch_stop(void);

bool     oled_stopwatch_is_running(void);
uint32_t oled_stopwatch_elapsed_ms(void);

#endif /* OLED_STOPWATCH_H */
