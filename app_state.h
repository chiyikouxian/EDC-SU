#ifndef APP_STATE_H
#define APP_STATE_H

#include "app_common.h"

void app_state_init(void);
void app_state_tick(void);
AppState_t app_state_get(void);
RunMode_t  app_state_get_mode(void);

#endif /* APP_STATE_H */
