/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "app_state.h"
#include "start_key.h"
#include "vision_uart.h"
#include "gimbal.h"
#include "chassis_iface.h"
#include "gray_sensor.h"
#include "track_bridge.h"
#include "motor.h"
#include "oled_stopwatch.h"
#include "mode_key.h"
#include "system_time.h"
#include "imu_sensing.h"
#include "stepper.h"
#include "stepper_feedback.h"

#define TICK_10MS_DIVIDER      10
#define OLED_STOPWATCH_MODE       0
#define IR_SERIAL_TEST_MODE       0
#define PID_TUNING_MODE           0
#define PID_TUNING_WHEEL_COMMAND  15
#define PID_TUNING_RUN_TICKS      400U  /* 4 seconds at 10 ms per tick. */

/* PID tuning telemetry for the 115200-baud LoRa link: one frame every 100 ms. */
#define PID_TELEMETRY_INTERVAL_TICKS     10U
#define PID_TELEMETRY_HEADER_INTERVAL_FRAMES 20U  /* Repeat header every 2 s. */
#define PID_TELEMETRY_TX_BUFFER_SIZE    128U /* Must be a power of two. */
#define PID_TELEMETRY_TX_BUFFER_MASK    (PID_TELEMETRY_TX_BUFFER_SIZE - 1U)
#define PID_TELEMETRY_FRAME_MAX          96U

/*
 * Temporary chassis test mode.
 *
 * 1 = after power-on, keep commanding both sides forward for motor/wiring test.
 *     The start-key and state-machine code is kept below, but app_state_tick()
 *     is bypassed so it will not stop or reroute the test drive.
 * 0 = normal competition flow: PA25 start key -> app_state state machine.
 */
#define MOTOR_TEST_ALWAYS_RUN  0
#define MOTOR_TEST_LEFT_SPEED  20
#define MOTOR_TEST_RIGHT_SPEED 20
#define KEY_INPUTS_ENABLED     1

/* Six-channel infrared sensor bench test.  When enabled, the car stays
 * braked and UART0 prints one frame every 100 ms:
 *   IR:X1=0 X2=0 X3=1 X4=1 X5=0 X6=0
 * UART0 wiring: PA10=TX, PA11=RX, 115200 8N1. */
#if IR_SERIAL_TEST_MODE
static void ir_serial_test_send_char(char value)
{
    while (DL_UART_Main_isTXFIFOFull(DEBUG_UART_INST)) {}
    DL_UART_Main_transmitData(DEBUG_UART_INST, (uint8_t)value);
}

static void ir_serial_test_send_frame(
    const uint8_t values[GRAY_SENSOR_CHANNELS])
{
    uint8_t channel;

    ir_serial_test_send_char('I');
    ir_serial_test_send_char('R');
    ir_serial_test_send_char(':');
    for (channel = 0U; channel < GRAY_SENSOR_CHANNELS; channel++) {
        ir_serial_test_send_char('X');
        ir_serial_test_send_char((char)('1' + channel));
        ir_serial_test_send_char('=');
        ir_serial_test_send_char(values[channel] ? '1' : '0');
        if (channel + 1U < GRAY_SENSOR_CHANNELS) {
            ir_serial_test_send_char(' ');
        }
    }
    ir_serial_test_send_char('\r');
    ir_serial_test_send_char('\n');
}
#endif

/* UART0 TX queue: the main control loop is the only producer and the UART0
 * TX ISR is the only consumer.  A full queue discards an entire telemetry
 * frame rather than delaying motor PWM or the 10 ms speed-control update. */
static char             pid_telemetry_tx_buffer[PID_TELEMETRY_TX_BUFFER_SIZE];
static volatile uint8_t pid_telemetry_tx_head;
static volatile uint8_t pid_telemetry_tx_tail;
static uint8_t          pid_telemetry_header_frame_count;

static void pid_telemetry_uart_kick_locked(void)
{
    while (pid_telemetry_tx_tail != pid_telemetry_tx_head &&
           !DL_UART_Main_isTXFIFOFull(DEBUG_UART_INST)) {
        DL_UART_Main_transmitData(
            DEBUG_UART_INST,
            (uint8_t)pid_telemetry_tx_buffer[pid_telemetry_tx_tail]);
        pid_telemetry_tx_tail =
            (uint8_t)((pid_telemetry_tx_tail + 1U) & PID_TELEMETRY_TX_BUFFER_MASK);
    }

    if (pid_telemetry_tx_tail != pid_telemetry_tx_head) {
        DL_UART_Main_enableInterrupt(DEBUG_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    } else {
        DL_UART_Main_disableInterrupt(DEBUG_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    }
}

static void pid_telemetry_uart_init(void)
{
    pid_telemetry_tx_head = 0U;
    pid_telemetry_tx_tail = 0U;
    pid_telemetry_header_frame_count = 0U;

    DL_UART_Main_disableInterrupt(DEBUG_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    NVIC_ClearPendingIRQ(DEBUG_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(DEBUG_UART_INST_INT_IRQN);
}

static bool pid_telemetry_enqueue(const char *frame, uint8_t length)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t used;
    uint8_t free_bytes;
    uint8_t index;

    __disable_irq();
    used = (uint8_t)((pid_telemetry_tx_head - pid_telemetry_tx_tail) &
                     PID_TELEMETRY_TX_BUFFER_MASK);
    free_bytes = (uint8_t)(PID_TELEMETRY_TX_BUFFER_MASK - used);

    if (length > free_bytes) {
        if (primask == 0U) {
            __enable_irq();
        }
        return false;
    }

    for (index = 0U; index < length; index++) {
        pid_telemetry_tx_buffer[pid_telemetry_tx_head] = frame[index];
        pid_telemetry_tx_head =
            (uint8_t)((pid_telemetry_tx_head + 1U) & PID_TELEMETRY_TX_BUFFER_MASK);
    }
    pid_telemetry_uart_kick_locked();

    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}

static void pid_telemetry_append_char(char *frame, uint8_t *length, char value)
{
    if (*length < PID_TELEMETRY_FRAME_MAX) {
        frame[(*length)++] = value;
    }
}

static void pid_telemetry_append_int(char *frame, uint8_t *length, int value)
{
    char digits[12];
    uint8_t count = 0U;
    unsigned int magnitude;

    if (value < 0) {
        pid_telemetry_append_char(frame, length, '-');
        magnitude = (unsigned int)(-value);
    } else {
        magnitude = (unsigned int)value;
    }

    do {
        digits[count++] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while (magnitude != 0U && count < sizeof(digits));

    while (count > 0U) {
        pid_telemetry_append_char(frame, length, digits[--count]);
    }
}

static void pid_telemetry_emit(uint32_t control_tick)
{
    const TrackBridgeData_t *bridge;
    char frame[PID_TELEMETRY_FRAME_MAX];
    uint8_t length = 0U;
    int target_left;
    int target_right;
    int measured_left;
    int measured_right;
    int pwm_left;
    int pwm_right;
    int line_error = 0;

    chassis_speed_pid_get_debug(&target_left, &target_right,
                                &measured_left, &measured_right,
                                &pwm_left, &pwm_right);
    bridge = track_bridge_get();
    if (bridge != 0) {
        line_error = bridge->error * CHASSIS_TRACK_ERROR_SCALE;
    }

    if (pid_telemetry_header_frame_count == 0U) {
        static const char header[] =
            "#PID-LINE:t,tl,tr,ml,mr,el,er,pl,pr,le\r\n";

        (void)pid_telemetry_enqueue(header, (uint8_t)(sizeof(header) - 1U));
    }

    pid_telemetry_append_char(frame, &length, 'P');
    pid_telemetry_append_char(frame, &length, ',');
    pid_telemetry_append_int(frame, &length, (int)control_tick);
    pid_telemetry_append_char(frame, &length, ',');
    pid_telemetry_append_int(frame, &length, target_left);
    pid_telemetry_append_char(frame, &length, ',');
    pid_telemetry_append_int(frame, &length, target_right);
    pid_telemetry_append_char(frame, &length, ',');
    pid_telemetry_append_int(frame, &length, measured_left);
    pid_telemetry_append_char(frame, &length, ',');
    pid_telemetry_append_int(frame, &length, measured_right);
    pid_telemetry_append_char(frame, &length, ',');
    pid_telemetry_append_int(frame, &length, target_left - measured_left);
    pid_telemetry_append_char(frame, &length, ',');
    pid_telemetry_append_int(frame, &length, target_right - measured_right);
    pid_telemetry_append_char(frame, &length, ',');
    pid_telemetry_append_int(frame, &length, pwm_left);
    pid_telemetry_append_char(frame, &length, ',');
    pid_telemetry_append_int(frame, &length, pwm_right);
    pid_telemetry_append_char(frame, &length, ',');
    pid_telemetry_append_int(frame, &length, line_error);
    pid_telemetry_append_char(frame, &length, '\r');
    pid_telemetry_append_char(frame, &length, '\n');

    if (pid_telemetry_enqueue(frame, length)) {
        pid_telemetry_header_frame_count++;
        if (pid_telemetry_header_frame_count >=
            PID_TELEMETRY_HEADER_INTERVAL_FRAMES) {
            pid_telemetry_header_frame_count = 0U;
        }
    }
}

void DEBUG_UART_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(DEBUG_UART_INST) ==
        DL_UART_MAIN_IIDX_TX) {
        pid_telemetry_uart_kick_locked();
    }
}

int main(void)
{
#if OLED_STOPWATCH_MODE
    SYSCFG_DL_init();
    chassis_init();
    chassis_brake();
    oled_stopwatch_init();

    NVIC_ClearPendingIRQ(STOPWATCH_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(STOPWATCH_TIMER_INST_INT_IRQN);
    DL_TimerG_startCounter(STOPWATCH_TIMER_INST);

    while (1) {
        oled_stopwatch_process();
    }
#elif IR_SERIAL_TEST_MODE
    uint8_t ir_values[GRAY_SENSOR_CHANNELS];

    SYSCFG_DL_init();
    chassis_init();
    chassis_brake();
    gray_sensor_init();

    while (1) {
        gray_sensor_read_all(ir_values);
        if (gray_sensor_read_ok()) {
            ir_serial_test_send_frame(ir_values);
        } else {
            ir_serial_test_send_char('I');
            ir_serial_test_send_char('2');
            ir_serial_test_send_char('C');
            ir_serial_test_send_char('_');
            ir_serial_test_send_char('E');
            ir_serial_test_send_char('R');
            ir_serial_test_send_char('R');
            ir_serial_test_send_char('\r');
            ir_serial_test_send_char('\n');
        }
        delay_cycles((CPUCLK_FREQ / 1000U) * 100U);
    }
#else
    uint16_t tick_10ms_cnt = 0;
#if MOTOR_TEST_ALWAYS_RUN
    uint16_t motor_test_ticks = 0;
    uint8_t motor_test_done = 0;
#else
    uint8_t telemetry_tick_divider = 0U;
    uint32_t control_tick = 0U;
#endif

    SYSCFG_DL_init();
    /* Must precede imu_sensing_init(): the sensor driver timestamps and
       schedules against system_time_ms(). */
    system_time_init();
    pid_telemetry_uart_init();

    start_key_init();
    mode_key_init();
    vision_uart_init();
    gimbal_init();
    chassis_init();
    gray_sensor_init();

    /* On-board run timer (说明 5). TIMG12 ticks every 1 ms and is the time
       base for the stopwatch; must be up before app_state_init(), which can
       already reset the timer while entering its initial state. */
    oled_stopwatch_init();
    NVIC_ClearPendingIRQ(STOPWATCH_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(STOPWATCH_TIMER_INST_INT_IRQN);
    DL_TimerG_startCounter(STOPWATCH_TIMER_INST);

    /* Spends ~54 ms in the sensor's mandated reset/settling delays. A missing
       or mis-wired IMU is not fatal: process() retries on a 500 ms cadence. */
    imu_sensing_init(system_time_ms());

    /* Pendulum rod. Feedback first so the encoder baseline and the PA26 angle
       capture are live before any STEP pulse can be emitted; stepper_init()
       leaves EN de-asserted, so the rod stays free until something enables it. */
    stepper_feedback_init();
    stepper_init();

    app_state_init();

#if PID_TUNING_MODE
    /* Raised-wheel open-loop encoder check.  PID gains are temporarily zero,
     * so the final motor command remains PID_TUNING_WHEEL_COMMAND. */
    chassis_unlock();
#endif

#if MOTOR_TEST_ALWAYS_RUN
    motor_unlock();
#endif

    while (1) {
        motor_update_pwm();                     /* [every 1 ms] motor enable refresh */
        chassis_encoder_poll();                 /* [every 1 ms] encoder feedback count */

#if KEY_INPUTS_ENABLED
        start_key_scan();                       /* [every 1 ms] keep existing semantics */
        mode_key_scan();
#endif

#if MOTOR_TEST_ALWAYS_RUN
        if (!motor_test_done) {
            tick_10ms_cnt++;
            if (tick_10ms_cnt >= TICK_10MS_DIVIDER) {
                tick_10ms_cnt = 0;
                if (motor_test_ticks < 200) {
                    chassis_drive(
                        (MOTOR_TEST_LEFT_SPEED + MOTOR_TEST_RIGHT_SPEED) / 2,
                        (MOTOR_TEST_LEFT_SPEED - MOTOR_TEST_RIGHT_SPEED) / 2);
                    motor_test_ticks++;
                } else {
                    chassis_brake();
                    motor_test_done = 1;
                }
            }
        }
#else
        tick_10ms_cnt++;
        if (tick_10ms_cnt >= TICK_10MS_DIVIDER) {
            tick_10ms_cnt = 0;
#if PID_TUNING_MODE
            if (control_tick <= PID_TUNING_RUN_TICKS) {
                chassis_drive(PID_TUNING_WHEEL_COMMAND, 0);
            } else {
                chassis_brake();
            }
#else
            app_state_tick();                   /* [every 10 ms] normal flow */
#endif
            control_tick++;

            telemetry_tick_divider++;
            if (telemetry_tick_divider >= PID_TELEMETRY_INTERVAL_TICKS) {
                telemetry_tick_divider = 0U;
                if (app_state_get() == APP_STATE_RUNNING) {
                    pid_telemetry_emit(control_tick);
                }
            }
        }
#endif

        /* [every 1 ms] drains the vision RX ring and assembles telemetry
           lines. The ISR only buffers bytes; parsing happens here so a
           malformed line cannot stretch interrupt latency. */
        vision_uart_process(system_time_ms());

        /* [every 1 ms] accumulates the TIMG8 quadrature count into 32 bits and
           re-arms the PA26 angle capture. Must run well inside the time the
           rod needs to travel 32767 counts, or update_position() misreads a
           wrap as reverse motion. */
        stepper_feedback_process(system_time_ms());

        /* [every 20 ms internally] IMU burst read; self-scheduling, returns
           immediately on the ticks between samples. Placed after the control
           tick so its ~1.2 ms blocking read never delays line tracking within
           the same iteration. */
        imu_sensing_process(system_time_ms());

        /* [every 1 ms] bounded software-I2C slice; deliberately after the
           control tick so it never delays line tracking in the same
           iteration. */
        oled_stopwatch_process();

        delay_cycles(CPUCLK_FREQ / 1000 * 1);  /* 1 ms tick */
    }
#endif
}
