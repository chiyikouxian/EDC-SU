#ifndef STEPPER_FEEDBACK_H
#define STEPPER_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

/* Pendulum-rod feedback: TIMG8 quadrature count, PA18 Z index, and the PA26
   absolute-angle PWM captured on TIMG7. abs_pwm_angle_cdeg is the value the
   ball-control loop treats as rod angle truth. */
typedef struct {
    int32_t position;
    int32_t commanded_steps;
    int32_t expected_encoder_count;
    int32_t command_error;
    int32_t last_z_position;
    uint32_t z_count;
    uint32_t abs_pwm_period_ticks;
    uint32_t abs_pwm_high_ticks;
    uint32_t abs_pwm_frequency_hz;
    uint32_t abs_pwm_capture_count;
    uint32_t abs_pwm_timestamp_ms;
    uint32_t timestamp_ms;
    uint16_t hardware_count;
    uint16_t abs_pwm_duty_permyriad;
    uint16_t abs_pwm_angle_cdeg;   /* 0..35999, 100 = 1 degree */
    int8_t direction;
    bool valid;
    bool abs_pwm_valid;
    bool scale_configured;
} StepperFeedbackSnapshot_t;

void stepper_feedback_init(void);
void stepper_feedback_process(uint32_t now_ms);

/* Call from GROUP1_IRQHandler(): the Z index shares that vector with the
   wheel encoders. */
void stepper_feedback_handle_gpio_irq(void);

void stepper_feedback_record_command_step(bool positive);
bool stepper_feedback_set_scale(
    uint32_t encoder_counts, uint32_t command_steps);
StepperFeedbackSnapshot_t stepper_feedback_get_snapshot(void);

#endif /* STEPPER_FEEDBACK_H */
