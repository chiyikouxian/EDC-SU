#include "stepper_feedback.h"

#include <limits.h>

#include "system_time.h"
#include "ti_msp_dl_config.h"

#define ABS_PWM_TIMER_HZ           1000000U
#define ABS_PWM_SAMPLE_PERIOD_MS  20U
#define ABS_PWM_INTERRUPT_MASK    (DL_TIMERG_INTERRUPT_CC1_DN_EVENT | \
                                   DL_TIMERG_INTERRUPT_ZERO_EVENT)

static volatile int32_t feedback_position;
static volatile int32_t commanded_steps;
static volatile int32_t last_z_position;
static volatile uint32_t z_count;
static volatile uint32_t feedback_timestamp_ms;
static volatile uint16_t last_hardware_count;
static volatile int8_t feedback_direction;
static volatile bool feedback_valid;
static volatile uint32_t scale_encoder_counts;
static volatile uint32_t scale_command_steps;
static volatile uint32_t abs_pwm_period_ticks;
static volatile uint32_t abs_pwm_high_ticks;
static volatile uint32_t abs_pwm_frequency_hz;
static volatile uint32_t abs_pwm_capture_count;
static volatile uint32_t abs_pwm_timestamp_ms;
static volatile uint16_t abs_pwm_duty_permyriad;
static volatile uint16_t abs_pwm_angle_cdeg;
static volatile bool abs_pwm_synced;
static volatile bool abs_pwm_valid;
static volatile bool abs_pwm_capture_active;
static volatile uint32_t next_abs_pwm_capture_ms;
static uint32_t abs_pwm_load_value;

static uint32_t enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void exit_critical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/* Accumulates the 16-bit hardware counter into a 32-bit position, treating a
   delta above INT16_MAX as a backward wrap. Must be called often enough that
   the rod cannot travel more than 32767 counts between calls. */
static void update_position(uint32_t now_ms)
{
    uint16_t current = (uint16_t)DL_TimerG_getTimerCount(STEPPER_QEI_INST);
    uint16_t forward_delta = (uint16_t)(current - last_hardware_count);
    int32_t delta;

    if (forward_delta <= INT16_MAX) {
        delta = (int32_t)forward_delta;
    } else {
        delta = (int32_t)forward_delta - 65536L;
    }
    feedback_position += delta;
    last_hardware_count = current;
    feedback_timestamp_ms = now_ms;
    if (delta > 0) {
        feedback_direction = 1;
    } else if (delta < 0) {
        feedback_direction = -1;
    }
}

/* Arms one PA26 capture window. The first CC1_DN only establishes sync; the
   window after it carries usable period/high tick counts. */
static void start_abs_pwm_capture(uint32_t now_ms)
{
    uint32_t primask = enter_critical();

    if (!abs_pwm_capture_active) {
        abs_pwm_synced = false;
        abs_pwm_capture_active = true;
        next_abs_pwm_capture_ms = now_ms + ABS_PWM_SAMPLE_PERIOD_MS;
        DL_TimerG_setTimerCount(STEPPER_ABS_PWM_INST, abs_pwm_load_value);
        DL_TimerG_clearInterruptStatus(
            STEPPER_ABS_PWM_INST, ABS_PWM_INTERRUPT_MASK);
        DL_TimerG_enableInterrupt(
            STEPPER_ABS_PWM_INST, ABS_PWM_INTERRUPT_MASK);
        NVIC_ClearPendingIRQ(STEPPER_ABS_PWM_INST_INT_IRQN);
        DL_TimerG_startCounter(STEPPER_ABS_PWM_INST);
    }
    exit_critical(primask);
}

void stepper_feedback_init(void)
{
    feedback_position = 0;
    commanded_steps = 0;
    last_z_position = 0;
    z_count = 0U;
    feedback_timestamp_ms = system_time_ms();
    feedback_direction = 0;
    feedback_valid = true;
    scale_encoder_counts = 0U;
    scale_command_steps = 0U;
    abs_pwm_period_ticks = 0U;
    abs_pwm_high_ticks = 0U;
    abs_pwm_frequency_hz = 0U;
    abs_pwm_capture_count = 0U;
    abs_pwm_timestamp_ms = system_time_ms();
    abs_pwm_duty_permyriad = 0U;
    abs_pwm_angle_cdeg = 0U;
    abs_pwm_synced = false;
    abs_pwm_valid = false;
    abs_pwm_capture_active = false;
    next_abs_pwm_capture_ms = system_time_ms();

    DL_TimerG_startCounter(STEPPER_QEI_INST);
    last_hardware_count =
        (uint16_t)DL_TimerG_getTimerCount(STEPPER_QEI_INST);
    DL_GPIO_clearInterruptStatus(
        STEPPER_ENCODER_PORT, STEPPER_ENCODER_INDEX_Z_PIN);
    NVIC_ClearPendingIRQ(STEPPER_ENCODER_INT_IRQN);
    NVIC_EnableIRQ(STEPPER_ENCODER_INT_IRQN);

    abs_pwm_load_value = DL_TimerG_getLoadValue(STEPPER_ABS_PWM_INST);
    DL_TimerG_setCoreHaltBehavior(
        STEPPER_ABS_PWM_INST, DL_TIMER_CORE_HALT_IMMEDIATE);
    NVIC_ClearPendingIRQ(STEPPER_ABS_PWM_INST_INT_IRQN);
    NVIC_EnableIRQ(STEPPER_ABS_PWM_INST_INT_IRQN);
    start_abs_pwm_capture(system_time_ms());
}

void stepper_feedback_process(uint32_t now_ms)
{
    uint32_t primask = enter_critical();
    update_position(now_ms);
    exit_critical(primask);

    if (!abs_pwm_capture_active &&
        ((int32_t)(now_ms - next_abs_pwm_capture_ms) >= 0)) {
        start_abs_pwm_capture(now_ms);
    }
}

void stepper_feedback_handle_gpio_irq(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(
        STEPPER_ENCODER_PORT, STEPPER_ENCODER_INDEX_Z_PIN);

    if ((pending & STEPPER_ENCODER_INDEX_Z_PIN) != 0U) {
        update_position(system_time_ms());
        last_z_position = feedback_position;
        z_count++;
        DL_GPIO_clearInterruptStatus(
            STEPPER_ENCODER_PORT, STEPPER_ENCODER_INDEX_Z_PIN);
    }
}

void stepper_feedback_record_command_step(bool positive)
{
    commanded_steps += positive ? 1 : -1;
}

bool stepper_feedback_set_scale(
    uint32_t encoder_counts, uint32_t command_steps_count)
{
    uint32_t primask;

    if ((encoder_counts == 0U) || (command_steps_count == 0U)) {
        return false;
    }
    primask = enter_critical();
    scale_encoder_counts = encoder_counts;
    scale_command_steps = command_steps_count;
    exit_critical(primask);
    return true;
}

StepperFeedbackSnapshot_t stepper_feedback_get_snapshot(void)
{
    StepperFeedbackSnapshot_t snapshot;
    uint32_t encoder_counts;
    uint32_t command_scale;
    int64_t expected;
    uint32_t primask = enter_critical();

    snapshot.position = feedback_position;
    snapshot.commanded_steps = commanded_steps;
    snapshot.last_z_position = last_z_position;
    snapshot.z_count = z_count;
    snapshot.abs_pwm_period_ticks = abs_pwm_period_ticks;
    snapshot.abs_pwm_high_ticks = abs_pwm_high_ticks;
    snapshot.abs_pwm_frequency_hz = abs_pwm_frequency_hz;
    snapshot.abs_pwm_capture_count = abs_pwm_capture_count;
    snapshot.abs_pwm_timestamp_ms = abs_pwm_timestamp_ms;
    snapshot.timestamp_ms = feedback_timestamp_ms;
    snapshot.hardware_count = last_hardware_count;
    snapshot.abs_pwm_duty_permyriad = abs_pwm_duty_permyriad;
    snapshot.abs_pwm_angle_cdeg = abs_pwm_angle_cdeg;
    snapshot.direction = feedback_direction;
    snapshot.valid = feedback_valid;
    snapshot.abs_pwm_valid = abs_pwm_valid;
    encoder_counts = scale_encoder_counts;
    command_scale = scale_command_steps;
    exit_critical(primask);

    /* Open-loop command count vs measured encoder count, in encoder units.
       A growing command_error means lost steps. */
    snapshot.scale_configured =
        (encoder_counts != 0U) && (command_scale != 0U);
    if (!snapshot.scale_configured) {
        snapshot.expected_encoder_count = 0;
        snapshot.command_error = 0;
        return snapshot;
    }
    expected = ((int64_t)snapshot.commanded_steps * encoder_counts) /
               command_scale;
    if (expected > INT32_MAX) {
        expected = INT32_MAX;
    } else if (expected < INT32_MIN) {
        expected = INT32_MIN;
    }
    snapshot.expected_encoder_count = (int32_t)expected;
    snapshot.command_error = snapshot.position - (int32_t)expected;
    return snapshot;
}

void STEPPER_ABS_PWM_INST_IRQHandler(void)
{
    uint32_t period_ticks;
    uint32_t high_ticks;
    uint32_t now_ms;

    switch (DL_TimerG_getPendingInterrupt(STEPPER_ABS_PWM_INST)) {
        case DL_TIMERG_IIDX_CC1_DN:
            if (abs_pwm_synced) {
                period_ticks = abs_pwm_load_value -
                    DL_TimerG_getCaptureCompareValue(
                        STEPPER_ABS_PWM_INST, DL_TIMER_CC_1_INDEX);
                high_ticks = abs_pwm_load_value -
                    DL_TimerG_getCaptureCompareValue(
                        STEPPER_ABS_PWM_INST, DL_TIMER_CC_0_INDEX);
                DL_TimerG_stopCounter(STEPPER_ABS_PWM_INST);
                DL_TimerG_disableInterrupt(
                    STEPPER_ABS_PWM_INST, ABS_PWM_INTERRUPT_MASK);
                abs_pwm_capture_active = false;
                now_ms = system_time_ms();
                next_abs_pwm_capture_ms =
                    now_ms + ABS_PWM_SAMPLE_PERIOD_MS;
                if ((period_ticks > 0U) && (high_ticks <= period_ticks)) {
                    abs_pwm_period_ticks = period_ticks;
                    abs_pwm_high_ticks = high_ticks;
                    abs_pwm_frequency_hz =
                        (ABS_PWM_TIMER_HZ + (period_ticks / 2U)) /
                        period_ticks;
                    abs_pwm_duty_permyriad = (uint16_t)(
                        ((high_ticks * 10000U) + (period_ticks / 2U)) /
                        period_ticks);
                    abs_pwm_angle_cdeg = (uint16_t)(
                        ((high_ticks * 36000U) + (period_ticks / 2U)) /
                        period_ticks);
                    abs_pwm_capture_count++;
                    abs_pwm_timestamp_ms = now_ms;
                    abs_pwm_valid = true;
                } else {
                    abs_pwm_valid = false;
                }
            } else {
                abs_pwm_synced = true;
            }
            /* TIMER_ERR_01: combined capture requires a manual reload. */
            DL_TimerG_setTimerCount(STEPPER_ABS_PWM_INST, abs_pwm_load_value);
            break;
        case DL_TIMERG_IIDX_ZERO:
            /* Window expired with no second edge: the PA26 signal is gone.
               Invalidate so the control loop stops trusting the angle. */
            DL_TimerG_stopCounter(STEPPER_ABS_PWM_INST);
            DL_TimerG_disableInterrupt(
                STEPPER_ABS_PWM_INST, ABS_PWM_INTERRUPT_MASK);
            abs_pwm_synced = false;
            abs_pwm_valid = false;
            abs_pwm_capture_active = false;
            next_abs_pwm_capture_ms =
                system_time_ms() + ABS_PWM_SAMPLE_PERIOD_MS;
            break;
        default:
            break;
    }
}
