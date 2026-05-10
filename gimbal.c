#include "gimbal.h"

/* TODO: Replace with real servo PWM control when hardware is connected */

static bool scanning;

void gimbal_init(void)
{
    scanning = false;
}

void gimbal_center(void)
{
    /* TODO: Set servo to center position (facing CD segment) */
    scanning = false;
}

void gimbal_scan_start(void)
{
    /* TODO: Begin sweeping servo from center toward AB side */
    scanning = true;
}

void gimbal_scan_step(void)
{
    /* TODO: Advance servo by one step, pause for vision frame capture */
}

void gimbal_stop(void)
{
    scanning = false;
}

bool gimbal_is_scanning(void)
{
    return scanning;
}
