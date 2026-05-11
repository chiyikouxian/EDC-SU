#ifndef GIMBAL_H
#define GIMBAL_H

#include "app_common.h"

void gimbal_init(void);
void gimbal_center(void);
void gimbal_scan_start(void);
void gimbal_scan_step(void);
void gimbal_stop(void);
bool gimbal_is_scanning(void);

#endif /* GIMBAL_H */
