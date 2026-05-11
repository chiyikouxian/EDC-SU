#ifndef BUZZER_H
#define BUZZER_H

#include "app_common.h"

void buzzer_init(void);
void buzzer_start_sequence(uint8_t count);
void buzzer_tick(void);
bool buzzer_is_done(void);
bool buzzer_on_last_beep_start(void);

#endif /* BUZZER_H */
