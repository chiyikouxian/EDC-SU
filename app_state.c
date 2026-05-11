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

#define STARTUP_BEEP_COUNT   3
#define FINISH_BEEP_COUNT    1
#define RUN_TIMEOUT_TICKS    (120u * TICKS_PER_SEC)
#define SCAN_STEP_INTERVAL   (500 / TICK_MS)

static AppState_t    state;
static RunMode_t     run_mode;
static TargetPoint_t target;
static uint32_t      state_ticks;
static uint16_t      scan_step_cnt;

static void enter_state(AppState_t new_state)
{
    state       = new_state;
    state_ticks = 0;
    track_bridge_mark_stale();
}

void app_state_init(void)
{
    state          = APP_STATE_IDLE;
    run_mode       = RUN_MODE_BASIC_DRIVE;
    target         = TARGET_NONE;
    state_ticks    = 0;
    scan_step_cnt  = 0;

    line_track_reset();
    track_bridge_init();
}

/* ---- per-state handlers ---- */

static void handle_idle(void)
{
    mode_key_scan();
    run_mode = mode_key_get_mode();

    if (start_key_triggered()) {
        vision_uart_clear();
        target = TARGET_NONE;

        switch (run_mode) {
            case RUN_MODE_BASIC_DRIVE:
                target = TARGET_C;
                enter_state(APP_STATE_START_BEEP);
                buzzer_start_sequence(STARTUP_BEEP_COUNT);
                break;
            case RUN_MODE_BASIC_NAV:
                enter_state(APP_STATE_FIXED_DETECT);
                break;
            case RUN_MODE_ADVANCED_NAV:
                chassis_lock();
                gimbal_center();
                gimbal_scan_start();
                scan_step_cnt = 0;
                enter_state(APP_STATE_GIMBAL_SCAN);
                break;
            default:
                enter_state(APP_STATE_ERROR);
                break;
        }
    }
}

static void handle_fixed_detect(void)
{
    chassis_lock();

    VisionResult_t res;
    if (vision_uart_take_result(&res) &&
        res.valid &&
        res.target != TARGET_NONE) {
        target = res.target;
        enter_state(APP_STATE_START_BEEP);
        buzzer_start_sequence(STARTUP_BEEP_COUNT);
    }
}

static void handle_gimbal_scan(void)
{
    chassis_lock();

    VisionResult_t res;
    if (vision_uart_take_result(&res) &&
        res.valid &&
        res.target != TARGET_NONE) {
        target = res.target;
        gimbal_stop();
        enter_state(APP_STATE_START_BEEP);
        buzzer_start_sequence(STARTUP_BEEP_COUNT);
        return;
    }

    scan_step_cnt++;
    if (scan_step_cnt >= SCAN_STEP_INTERVAL) {
        scan_step_cnt = 0;
        gimbal_scan_step();
    }
}

static void handle_start_beep(void)
{
    chassis_lock();
    buzzer_tick();

    if (buzzer_on_last_beep_start()) {
        /* timing begins at the start of the last beep */
    }

    if (buzzer_is_done()) {
        chassis_unlock();
        line_track_reset();
        chassis_follow_target(target);
        enter_state(APP_STATE_RUNNING);
    }
}

static void handle_running(void)
{
    /* Update tracking bridge before chassis tick so Hardware A can consume
       fresh line_track data in the same tick if needed. */
    {
        uint8_t gv[GRAY_SENSOR_CHANNELS];
        LineTrackState_t lt;
        gray_sensor_read_all(gv);

        /* Apply turn sensor mask to suppress opposite-side interference.
           Mask is set by chassis on turn start; cleared by chassis on exit. */
        {
            int8_t mask = track_bridge_get_turn_mask();
            if (mask > 0) {
                gv[0] = 0;  /* mask X1 */
                gv[1] = 0;  /* mask X2 */
            } else if (mask < 0) {
                gv[6] = 0;  /* mask X7 */
                gv[7] = 0;  /* mask X8 */
            }
        }

        line_track_compute(gv, &lt);
        track_bridge_update(&lt, state_ticks);
    }

    chassis_tick();
    state_ticks++;

    ChassisStatus_t cs = chassis_get_status();

    if (cs == CHASSIS_STATUS_TARGET_REACHED) {
        chassis_stop();
        enter_state(APP_STATE_TARGET_STOP);
        return;
    }

    if (cs == CHASSIS_STATUS_LINE_LOST || cs == CHASSIS_STATUS_ERROR) {
        chassis_stop();
        chassis_lock();
        enter_state(APP_STATE_ERROR);
        return;
    }

    if (state_ticks >= RUN_TIMEOUT_TICKS) {
        chassis_stop();
        chassis_lock();
        enter_state(APP_STATE_ERROR);
    }
}

static void handle_target_stop(void)
{
    chassis_lock();
    buzzer_start_sequence(FINISH_BEEP_COUNT);
    enter_state(APP_STATE_FINISH_BEEP);
}

static void handle_finish_beep(void)
{
    buzzer_tick();
    if (buzzer_is_done()) {
        enter_state(APP_STATE_IDLE);
    }
}

static void handle_error(void)
{
    chassis_lock();
    gimbal_stop();
    /* stay in error until power cycle or manual reset */
}

/* ---- public API ---- */

void app_state_tick(void)
{
    switch (state) {
        case APP_STATE_IDLE:         handle_idle();          break;
        case APP_STATE_FIXED_DETECT: handle_fixed_detect();  break;
        case APP_STATE_GIMBAL_SCAN:  handle_gimbal_scan();   break;
        case APP_STATE_START_BEEP:   handle_start_beep();    break;
        case APP_STATE_RUNNING:      handle_running();       break;
        case APP_STATE_TARGET_STOP:  handle_target_stop();   break;
        case APP_STATE_FINISH_BEEP:  handle_finish_beep();   break;
        case APP_STATE_ERROR:        handle_error();         break;
        default:                     enter_state(APP_STATE_ERROR); break;
    }
}

AppState_t app_state_get(void)
{
    return state;
}

RunMode_t app_state_get_mode(void)
{
    return run_mode;
}
