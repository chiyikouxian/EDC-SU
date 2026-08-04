#ifndef STEPPER_H
#define STEPPER_H

#include <stdbool.h>
#include <stdint.h>

#define STEPPER_MIN_FREQUENCY_HZ 20U
#define STEPPER_MAX_FREQUENCY_HZ 5000U

typedef enum {
    STEPPER_STATE_DISABLED = 0,
    STEPPER_STATE_IDLE,
    STEPPER_STATE_CONTINUOUS,
    STEPPER_STATE_JOG,
    STEPPER_STATE_FAULT
} StepperState_t;

typedef struct {
    StepperState_t state;
    int32_t commanded_frequency_hz;
    uint32_t remaining_pulses;
    bool enabled;
} StepperStatus_t;

void stepper_init(void);
bool stepper_enable(void);
void stepper_stop(void);
void stepper_disable(void);
void stepper_emergency_stop(void);
bool stepper_set_speed(int32_t frequency_hz);
bool stepper_jog(int32_t pulses, uint32_t frequency_hz);
StepperStatus_t stepper_get_status(void);
bool stepper_is_busy(void);

#endif /* STEPPER_H */
