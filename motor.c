#include "motor.h"
#include "ti_msp_dl_config.h"

#define AIN1_PORT   GPIO_MOTOR_AIN1_PORT
#define AIN1_PIN    GPIO_MOTOR_AIN1_PIN

#define AIN2_PORT   GPIO_MOTOR_AIN2_PORT
#define AIN2_PIN    GPIO_MOTOR_AIN2_PIN

#define BIN1_PORT   GPIO_MOTOR_BIN1_PORT
#define BIN1_PIN    GPIO_MOTOR_BIN1_PIN

#define BIN2_PORT   GPIO_MOTOR_BIN2_PORT
#define BIN2_PIN    GPIO_MOTOR_BIN2_PIN

#define PWMA_PORT   GPIO_MOTOR_PWMA_PORT
#define PWMA_PIN    GPIO_MOTOR_PWMA_PIN

#define PWMB_PORT   GPIO_MOTOR_PWMB_PORT
#define PWMB_PIN    GPIO_MOTOR_PWMB_PIN

#define STBY_PORT   GPIO_MOTOR_STBY_PORT
#define STBY_PIN    GPIO_MOTOR_STBY_PIN

#define MOTOR_RAMP_STEP 2
#define MOTOR_RAMP_INTERVAL_TICKS 1
#define MOTOR_A_POLARITY 1
#define MOTOR_B_POLARITY 1

static int motor_locked = 1;
static int target_a = 0;
static int target_b = 0;
static int current_a = 0;
static int current_b = 0;
static int duty_a = 0;
static int duty_b = 0;
static int pwm_accumulator_a = 0;
static int pwm_accumulator_b = 0;
static int ramp_tick = 0;
static int brake_active = 0;

static int clamp_speed(int speed)
{
    if (speed > MOTOR_MAX_SPEED) return MOTOR_MAX_SPEED;
    if (speed < -MOTOR_MAX_SPEED) return -MOTOR_MAX_SPEED;
    return speed;
}

static void set_channel(GPIO_Regs *in1_port, uint32_t in1_pin,
                        GPIO_Regs *in2_port, uint32_t in2_pin,
                        int speed)
{
    if (speed > 0)
    {
        DL_GPIO_setPins(in1_port, in1_pin);
        DL_GPIO_clearPins(in2_port, in2_pin);
    }
    else if (speed < 0)
    {
        DL_GPIO_clearPins(in1_port, in1_pin);
        DL_GPIO_setPins(in2_port, in2_pin);
    }
    else
    {
        DL_GPIO_clearPins(in1_port, in1_pin);
        DL_GPIO_clearPins(in2_port, in2_pin);
    }
}

static void brake_channel(GPIO_Regs *in1_port, uint32_t in1_pin,
                          GPIO_Regs *in2_port, uint32_t in2_pin)
{
    DL_GPIO_setPins(in1_port, in1_pin);
    DL_GPIO_setPins(in2_port, in2_pin);
}

static void enable_driver(void)
{
    DL_GPIO_setPins(STBY_PORT, STBY_PIN);
    DL_GPIO_setPins(PWMA_PORT, PWMA_PIN);
    DL_GPIO_setPins(PWMB_PORT, PWMB_PIN);
}

static void disable_driver(void)
{
    DL_GPIO_clearPins(PWMA_PORT, PWMA_PIN);
    DL_GPIO_clearPins(PWMB_PORT, PWMB_PIN);
    DL_GPIO_clearPins(STBY_PORT, STBY_PIN);
}

static int speed_to_duty(int speed)
{
    int duty = clamp_speed(speed);

    if (duty < 0) duty = -duty;
    return duty;
}

static int ramp_toward(int current, int target)
{
    if (current < target) {
        current += MOTOR_RAMP_STEP;
        if (current > target) current = target;
    } else if (current > target) {
        current -= MOTOR_RAMP_STEP;
        if (current < target) current = target;
    }

    return current;
}

static void update_ramped_speeds(void)
{
    ramp_tick++;
    if (ramp_tick < MOTOR_RAMP_INTERVAL_TICKS) {
        return;
    }
    ramp_tick = 0;

    current_a = ramp_toward(current_a, target_a);
    current_b = ramp_toward(current_b, target_b);
}

/* Pulse-density modulation at the 1 ms motor_update_pwm() rate.
 * This preserves 1% speed resolution without stretching the PWM frame to
 * 100 ms, avoiding the large 10% output jumps of the previous 10-slot PWM. */
static void write_pwm(GPIO_Regs *port, uint32_t pin, int duty,
                      int *accumulator)
{
    *accumulator += duty;

    if (*accumulator >= MOTOR_MAX_SPEED)
    {
        *accumulator -= MOTOR_MAX_SPEED;
        DL_GPIO_setPins(port, pin);
    }
    else
    {
        DL_GPIO_clearPins(port, pin);
    }
}

void motor_init(void)
{
    enable_driver();
    motor_locked = 0;
    motor_stop();
}

void motor_set_raw(int left, int right)
{
    if (motor_locked)
    {
        return;
    }

    DL_GPIO_setPins(STBY_PORT, STBY_PIN);

    brake_active = 0;
    target_a = clamp_speed(left);
    target_b = clamp_speed(right);
}

void motor_set_speed(int left, int right)
{
    motor_set_raw(left, right);
}

void motor_update_pwm(void)
{
    if (motor_locked)
    {
        disable_driver();
        return;
    }

    DL_GPIO_setPins(STBY_PORT, STBY_PIN);

    if (brake_active) {
        DL_GPIO_setPins(PWMA_PORT, PWMA_PIN);
        DL_GPIO_setPins(PWMB_PORT, PWMB_PIN);
        return;
    }

    update_ramped_speeds();

    set_channel(AIN1_PORT, AIN1_PIN, AIN2_PORT, AIN2_PIN,
        current_a * MOTOR_A_POLARITY);
    set_channel(BIN1_PORT, BIN1_PIN, BIN2_PORT, BIN2_PIN,
        current_b * MOTOR_B_POLARITY);

    duty_a = speed_to_duty(current_a);
    duty_b = speed_to_duty(current_b);

    write_pwm(PWMA_PORT, PWMA_PIN, duty_a, &pwm_accumulator_a);
    write_pwm(PWMB_PORT, PWMB_PIN, duty_b, &pwm_accumulator_b);
}

void motor_force_forward_full(void)
{
    motor_unlock();
    target_a = MOTOR_MAX_SPEED;
    target_b = MOTOR_MAX_SPEED;
    current_a = MOTOR_MAX_SPEED;
    current_b = MOTOR_MAX_SPEED;
    brake_active = 0;
    motor_update_pwm();
}

void motor_coast(void)
{
    target_a = 0;
    target_b = 0;
    current_a = 0;
    current_b = 0;
    duty_a = 0;
    duty_b = 0;
    pwm_accumulator_a = 0;
    pwm_accumulator_b = 0;
    ramp_tick = 0;
    brake_active = 0;

    set_channel(AIN1_PORT, AIN1_PIN, AIN2_PORT, AIN2_PIN, 0);
    set_channel(BIN1_PORT, BIN1_PIN, BIN2_PORT, BIN2_PIN, 0);

    DL_GPIO_clearPins(PWMA_PORT, PWMA_PIN);
    DL_GPIO_clearPins(PWMB_PORT, PWMB_PIN);
}

void motor_brake(void)
{
    if (motor_locked)
    {
        return;
    }

    duty_a = MOTOR_MAX_SPEED;
    duty_b = MOTOR_MAX_SPEED;
    target_a = 0;
    target_b = 0;
    current_a = 0;
    current_b = 0;
    pwm_accumulator_a = 0;
    pwm_accumulator_b = 0;
    ramp_tick = 0;
    brake_active = 1;

    DL_GPIO_setPins(STBY_PORT, STBY_PIN);
    DL_GPIO_setPins(PWMA_PORT, PWMA_PIN);
    DL_GPIO_setPins(PWMB_PORT, PWMB_PIN);

    brake_channel(AIN1_PORT, AIN1_PIN, AIN2_PORT, AIN2_PIN);
    brake_channel(BIN1_PORT, BIN1_PIN, BIN2_PORT, BIN2_PIN);
}

void motor_stop(void)
{
    motor_coast();
}

void motor_lock(void)
{
    motor_brake();
    disable_driver();
    motor_locked = 1;
}

void motor_unlock(void)
{
    target_a = 0;
    target_b = 0;
    current_a = 0;
    current_b = 0;
    duty_a = 0;
    duty_b = 0;
    pwm_accumulator_a = 0;
    pwm_accumulator_b = 0;
    ramp_tick = 0;
    brake_active = 0;
    enable_driver();
    motor_locked = 0;
}

int motor_is_locked(void)
{
    return motor_locked;
}
