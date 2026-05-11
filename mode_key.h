#ifndef MODE_KEY_H
#define MODE_KEY_H

#include "app_common.h"

void mode_key_init(void);
void mode_key_scan(void);
RunMode_t mode_key_get_mode(void);

#endif /* MODE_KEY_H */
