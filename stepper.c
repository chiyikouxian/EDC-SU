#include "stepper.h"

#include "ti_msp_dl_config.h"
#include "stepper_feedback.h"

#define STEPPER_EN_ACTIVE_HIGH 1
#define STEPPER_POSITIVE_DIR_HIGH 1

typedef struct {
    volatile StepperState_t state;
    volatile int32_t commanded_frequency_hz;
    volatile uint32_t remaining_pulses;
    volatile bool enabled;
    volatile bool direction_high;
} StepperRuntime_t;

static StepperRuntime_t stepper_runtime;

static uint32_t stepper_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void stepper_exit_critical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static void stepper_write_enable(bool enabled)
{
    bool drive_high = enabled == (STEPPER_EN_ACTIVE_HIGH != 0);

    if (drive_high) {
        DL_GPIO_setPins(STEPPER_CTRL_EN1_PORT, STEPPER_CTRL_EN1_PIN);
    } else {
        DL_GPIO_clearPins(STEPPER_CTRL_EN1_PORT, STEPPER_CTRL_EN1_PIN);
    }
}

static void stepper_write_direction(bool positive)
{
    bool drive_high = positive == (STEPPER_POSITIVE_DIR_HIGH != 0);

    if (drive_high) {
        DL_GPIO_setPins(STEPPER_CTRL_DIR1_PORT, STEPPER_CTRL_DIR1_PIN);
    } else {
        DL_GPIO_clearPins(STEPPER_CTRL_DIR1_PORT, STEPPER_CTRL_DIR1_PIN);
    }
    stepper_runtime.direction_high = drive_high;
}

static void stepper_stop_locked(StepperState_t next_state)
{
    DL_TimerA_stopCounter(STEPPER_PWM_INST);
    DL_TimerA_disableInterrupt(
        STEPPER_PWM_INST, DL_TIMER_INTERRUPT_CC0_DN_EVENT);
    DL_TimerA_setCCPOutputDisabled(STEPPER_PWM_INST,
        DL_TIMER_CCP_DIS_OUT_LOW, DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);
    DL_TimerA_clearInterruptStatus(
        STEPPER_PWM_INST, DL_TIMER_INTERRUPT_CC0_DN_EVENT);

    stepper_runtime.commanded_frequency_hz = 0;
    stepper_runtime.remaining_pulses = 0U;
    stepper_runtime.state = next_state;
}

static bool stepper_frequency_values(
    uint32_t frequency_hz, uint32_t *load_value, uint32_t *compare_value)
{
    uint32_t period_ticks;

    if ((frequency_hz < STEPPER_MIN_FREQUENCY_HZ) ||
        (frequency_hz > STEPPER_MAX_FREQUENCY_HZ)) {
        return false;
    }

    period_ticks =
        (STEPPER_PWM_INST_CLK_FREQ + (frequency_hz / 2U)) / frequency_hz;
    if ((period_ticks < 2U) || (period_ticks > 65536U)) {
        return false;
    }

    *load_value = period_ticks - 1U;
    *compare_value = (period_ticks / 2U) - 1U;
    return true;
}

static void stepper_start_locked(uint32_t load_value, uint32_t compare_value,
    bool positive, StepperState_t state, int32_t command_hz,
    uint32_t remaining_pulses)
{
    stepper_stop_locked(STEPPER_STATE_IDLE);
    stepper_write_direction(positive);

    DL_TimerA_disableShadowFeatures(STEPPER_PWM_INST);
    DL_TimerA_setLoadValue(STEPPER_PWM_INST, load_value);
    DL_TimerA_setCaptureCompareValue(
        STEPPER_PWM_INST, compare_value, GPIO_STEPPER_PWM_C0_IDX);
    DL_TimerA_setTimerCount(STEPPER_PWM_INST, load_value);
    DL_TimerA_enableShadowFeatures(STEPPER_PWM_INST);
    DL_TimerA_clearInterruptStatus(
        STEPPER_PWM_INST, DL_TIMER_INTERRUPT_CC0_DN_EVENT);
    DL_TimerA_enableInterrupt(
        STEPPER_PWM_INST, DL_TIMER_INTERRUPT_CC0_DN_EVENT);
    DL_TimerA_setCCPOutputDisabled(STEPPER_PWM_INST,
        DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL,
        DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);

    stepper_runtime.commanded_frequency_hz = command_hz;
    stepper_runtime.remaining_pulses = remaining_pulses;
    stepper_runtime.state = state;
    DL_TimerA_startCounter(STEPPER_PWM_INST);
}

void stepper_init(void)
{
    uint32_t primask = stepper_enter_critical();

    stepper_runtime.enabled = false;
    stepper_runtime.direction_high = false;
    stepper_write_direction(false);
    stepper_write_enable(false);
    stepper_stop_locked(STEPPER_STATE_DISABLED);

    NVIC_ClearPendingIRQ(STEPPER_PWM_INST_INT_IRQN);
    NVIC_EnableIRQ(STEPPER_PWM_INST_INT_IRQN);
    stepper_exit_critical(primask);
}

bool stepper_enable(void)
{
    uint32_t primask = stepper_enter_critical();

    stepper_stop_locked(STEPPER_STATE_IDLE);
    stepper_write_enable(true);
    stepper_runtime.enabled = true;
    stepper_exit_critical(primask);
    return true;
}

void stepper_stop(void)
{
    uint32_t primask = stepper_enter_critical();

    stepper_stop_locked(stepper_runtime.enabled ?
        STEPPER_STATE_IDLE : STEPPER_STATE_DISABLED);
    stepper_exit_critical(primask);
}

void stepper_disable(void)
{
    uint32_t primask = stepper_enter_critical();

    stepper_stop_locked(STEPPER_STATE_DISABLED);
    stepper_write_enable(false);
    stepper_runtime.enabled = false;
    stepper_exit_critical(primask);
}

void stepper_emergency_stop(void)
{
    uint32_t primask = stepper_enter_critical();

    stepper_stop_locked(STEPPER_STATE_FAULT);
    stepper_write_enable(false);
    stepper_runtime.enabled = false;
    stepper_exit_critical(primask);
}

bool stepper_set_speed(int32_t frequency_hz)
{
    uint32_t magnitude;
    uint32_t load_value;
    uint32_t compare_value;
    uint32_t primask;
    bool positive;

    if (frequency_hz == 0) {
        stepper_stop();
        return true;
    }

    positive = frequency_hz > 0;
    magnitude = positive ?
        (uint32_t)frequency_hz : (uint32_t)(-(int64_t)frequency_hz);
    if (!stepper_frequency_values(
            magnitude, &load_value, &compare_value)) {
        return false;
    }

    primask = stepper_enter_critical();
    if (!stepper_runtime.enabled) {
        stepper_exit_critical(primask);
        return false;
    }

    /* Same direction and already running: retune load/compare in place so the
       pulse train is not interrupted. Any direction change restarts it. */
    if ((stepper_runtime.state == STEPPER_STATE_CONTINUOUS) &&
        (stepper_runtime.direction_high ==
            (positive == (STEPPER_POSITIVE_DIR_HIGH != 0)))) {
        DL_TimerA_setLoadValue(STEPPER_PWM_INST, load_value);
        DL_TimerA_setCaptureCompareValue(
            STEPPER_PWM_INST, compare_value, GPIO_STEPPER_PWM_C0_IDX);
        stepper_runtime.commanded_frequency_hz = frequency_hz;
    } else {
        stepper_start_locked(load_value, compare_value, positive,
            STEPPER_STATE_CONTINUOUS, frequency_hz, 0U);
    }

    stepper_exit_critical(primask);
    return true;
}

bool stepper_jog(int32_t pulses, uint32_t frequency_hz)
{
    uint32_t pulse_count;
    uint32_t load_value;
    uint32_t compare_value;
    uint32_t primask;
    bool positive;
    int32_t command_hz;

    if (pulses == 0) {
        stepper_stop();
        return true;
    }
    if (!stepper_frequency_values(
            frequency_hz, &load_value, &compare_value)) {
        return false;
    }

    positive = pulses > 0;
    pulse_count = positive ?
        (uint32_t)pulses : (uint32_t)(-(int64_t)pulses);
    command_hz = positive ?
        (int32_t)frequency_hz : -(int32_t)frequency_hz;

    primask = stepper_enter_critical();
    if (!stepper_runtime.enabled) {
        stepper_exit_critical(primask);
        return false;
    }

    stepper_start_locked(load_value, compare_value, positive,
        STEPPER_STATE_JOG, command_hz, pulse_count);
    stepper_exit_critical(primask);
    return true;
}

StepperStatus_t stepper_get_status(void)
{
    StepperStatus_t status;
    uint32_t primask = stepper_enter_critical();

    status.state = stepper_runtime.state;
    status.commanded_frequency_hz = stepper_runtime.commanded_frequency_hz;
    status.remaining_pulses = stepper_runtime.remaining_pulses;
    status.enabled = stepper_runtime.enabled;
    stepper_exit_critical(primask);
    return status;
}

bool stepper_is_busy(void)
{
    StepperState_t state = stepper_get_status().state;

    return (state == STEPPER_STATE_CONTINUOUS) ||
           (state == STEPPER_STATE_JOG);
}

void STEPPER_PWM_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(STEPPER_PWM_INST)) {
        case DL_TIMERA_IIDX_CC0_DN:
            /* One emitted STEP edge: feed the open-loop command counter that
               stepper_feedback compares against the encoder. */
            stepper_feedback_record_command_step(
                stepper_runtime.commanded_frequency_hz > 0);
            if (stepper_runtime.state == STEPPER_STATE_JOG) {
                if (stepper_runtime.remaining_pulses > 0U) {
                    stepper_runtime.remaining_pulses--;
                }
                if (stepper_runtime.remaining_pulses == 0U) {
                    stepper_stop_locked(STEPPER_STATE_IDLE);
                }
            }
            break;

        default:
            break;
    }
}
