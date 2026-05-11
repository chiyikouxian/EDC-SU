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
#include "buzzer.h"
#include "mode_key.h"
#include "start_key.h"
#include "vision_uart.h"
#include "gimbal.h"
#include "chassis_iface.h"
#include "gray_sensor.h"
#include "line_track.h"
#include "track_bridge.h"
#include "motor.h"

#define DEBUG_INTERVAL_MS     500
#define TICK_10MS_DIVIDER      10
#define DEBUG_PRINT_WHILE_RUNNING 0

static void uart_send_char(char c)
{
    while (DL_UART_isBusy(DEBUG_UART_INST)) ;
    DL_UART_transmitData(DEBUG_UART_INST, (uint8_t)c);
}

static void uart_send_str(const char *s)
{
    while (*s) {
        uart_send_char(*s++);
    }
}

static const char *mode_names[]    = {"DRV", "NAV", "ADV"};
static const char *state_names[]   = {"IDLE", "FDET", "GSCAN", "SBEEP",
                                      "RUN", "TSTOP", "FBEEP", "ERR"};
static const char *chassis_names[] = {"IDL", "FLIN", "FOLL", "RCHD",
                                      "LOST", "ERR"};
static const char *target_names[]  = {"-", "A", "B", "C", "D"};

int main(void)
{
    uint16_t tick_10ms_cnt = 0;
    uint16_t debug_cnt     = 0;

    SYSCFG_DL_init();

    buzzer_init();
    mode_key_init();
    start_key_init();
    vision_uart_init();
    gimbal_init();
    chassis_init();
    gray_sensor_init();
    app_state_init();

    while (1) {
        motor_update_pwm();                     /* [every 1 ms] motor enable refresh */

        start_key_scan();                       /* [every 1 ms] keep existing semantics */

        tick_10ms_cnt++;
        if (tick_10ms_cnt >= TICK_10MS_DIVIDER) {
            tick_10ms_cnt = 0;
            app_state_tick();                   /* [every 10 ms] unchanged timing */
        }

        debug_cnt++;
        if (debug_cnt >= DEBUG_INTERVAL_MS) {
            debug_cnt = 0;

#if !DEBUG_PRINT_WHILE_RUNNING
            if (app_state_get() == APP_STATE_RUNNING) {
                continue;
            }
#endif

            {
                uint8_t gv[GRAY_SENSOR_CHANNELS];
                gray_sensor_read_all(gv);

                uart_send_str("S=");
                uart_send_str(state_names[app_state_get()]);
                uart_send_str(" M=");
                uart_send_str(mode_names[app_state_get_mode()]);
                uart_send_str(" T=");
                uart_send_str(target_names[chassis_get_target()]);
                uart_send_str(" C=");
                uart_send_str(chassis_names[chassis_get_status()]);
                uart_send_str(" G=");
                uart_send_str(gray_sensor_debug_string(gv));

                {
                    LineTrackState_t lt;
                    line_track_compute(gv, &lt);
                    uart_send_str(" L=");
                    uart_send_str(line_track_debug_string(&lt));
                }

                uart_send_str(" B=");
                uart_send_str(track_bridge_debug_string());

                uart_send_str("\r\n");
            }
        }

        delay_cycles(CPUCLK_FREQ / 1000 * 1);  /* 1 ms tick */
    }
}
