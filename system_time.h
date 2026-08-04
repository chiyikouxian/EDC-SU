#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H

#include <stdint.h>

/* Free-running 1 ms tick from SysTick, used as the timestamp source for
   sensor sampling. Independent of the TIMG12 run timer in oled_stopwatch.c:
   that one measures a single scored run, this one never resets. */
void system_time_init(void);
uint32_t system_time_ms(void);

#endif /* SYSTEM_TIME_H */
