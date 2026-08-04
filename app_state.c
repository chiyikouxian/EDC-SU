#include "app_state.h"
#include "start_key.h"
#include "vision_uart.h"
#include "gimbal.h"
#include "chassis_iface.h"
#include "gray_sensor.h"
#include "line_track.h"
#include "track_bridge.h"
#include "oled_stopwatch.h"
#include "mode_key.h"
#include "stepper.h"
#include "stepper_feedback.h"
#include "system_time.h"
#include "imu_sensing.h"
#include "problem2_config.h"
#include "problem4_config.h"
#include "problem56_config.h"
#include "ti_msp_dl_config.h"

#define RUN_TIMEOUT_TICKS    (120u * TICKS_PER_SEC)
/* SCAN_STEP_INTERVAL / scan_step_cnt went with handle_gimbal_scan(); the
   gimbal module itself is left in place but is no longer driven from here. */
#define MODE_SELECT_CONFIRM_TICKS  100U
#define IR_DEBUG_INTERVAL_TICKS    50U

/* ---- pendulum ball control ---- */

#define BALL_TARGET_POSITION_MM_X10       1270L
#define BALL_POSITION_MIN_MM_X10           100L
#define BALL_POSITION_MAX_MM_X10          2600L
#define BALL_VELOCITY_LIMIT_MM_S_X10     15000L
#define BALL_MAX_SAMPLE_JUMP_MM_X10        600L
#define BALL_VALID_STREAK_REQUIRED           3U
#define BALL_ACCEPTED_SAMPLE_TIMEOUT_MS      250U
#define BALL_INVALID_GRACE_MS                250U
#define BALL_REACQUIRE_STREAK_REQUIRED        5U

#define ROD_NEUTRAL_ANGLE_CDEG            23780L
#define ROD_TARGET_OFFSET_LIMIT_CDEG        500L
/* Extra target-angle headroom reserved for friction/stuck feedforward only.
   Without this, a P/D term already saturated at ROD_TARGET_OFFSET_LIMIT_CDEG
   (e.g. a large ball_error_mm_x10 far from target) leaves the outer clamp no
   room left for the feedforward term, so it gets silently discarded exactly
   when the ball needs the extra kick the most. Bounded well inside the hard
   travel guards (APP_ROD_HOME_MIN/MAX_ANGLE_CDEG). */
#define ROD_FRICTION_FF_HEADROOM_CDEG        450L
#define ROD_HARD_MIN_ANGLE_CDEG \
    ((int32_t)APP_ROD_HOME_MIN_ANGLE_CDEG)
#define ROD_HARD_MAX_ANGLE_CDEG \
    ((int32_t)APP_ROD_HOME_MAX_ANGLE_CDEG)
#define ROD_ABS_FEEDBACK_TIMEOUT_MS          100U
#define ROD_ANGLE_DEADBAND_CDEG                15L

#define BALL_POSITION_NEAR_GAIN_CDEG_PER_MM     3L
#define BALL_POSITION_MID_GAIN_CDEG_PER_MM      5L
#define BALL_POSITION_FAR_GAIN_CDEG_PER_MM      7L
/* Item-3 travel uses a modestly stronger position term so the ball enters
   the final stable window before the 5 s deadline. READY centering and DONE
   holding keep the quieter base gains above. */
#define BALL_Q3_POSITION_NEAR_GAIN_CDEG_PER_MM  4L
#define BALL_Q3_POSITION_MID_GAIN_CDEG_PER_MM   6L
/* Once -5 cm has been reached, use a quieter controller: lower position
   authority, stronger velocity damping, and bounded friction compensation. */
#define BALL_Q3_HOLD_NEAR_GAIN_CDEG_PER_MM      4L
#define BALL_Q3_HOLD_MID_GAIN_CDEG_PER_MM       4L
#define BALL_Q3_HOLD_FAR_GAIN_CDEG_PER_MM       5L
#define BALL_Q3_HOLD_VELOCITY_DIVISOR            8L
#define ROD_Q3_HOLD_OFFSET_LIMIT_CDEG           400L
/* Must not be tighter than APP_BALL_STUCK_FF_BOOST_MAX_CDEG: the -5 cm hold
   is the one point in item 3 that must actually settle, so this phase can
   least afford to cap the stuck-escalation boost below what it is allowed
   to reach everywhere else. A tighter cap here silently discarded exactly
   the extra kick a stubborn stick at this position needed. */
#define ROD_Q3_HOLD_FF_LIMIT_CDEG               400L
#define ROD_Q3_HOLD_SLEW_CDEG_PER_SAMPLE        75L
#define BALL_POSITION_MID_THRESHOLD_MM_X10    200L
#define BALL_POSITION_FAR_THRESHOLD_MM_X10    600L
#define BALL_VELOCITY_DIVISOR                   6L
#define BALL_VELOCITY_TARGET_LIMIT_CDEG       400L
#define ROD_TARGET_SLEW_CDEG_PER_SAMPLE       100L
#define ROD_SPEED_GAIN_HZ_PER_CDEG              4L
#define ROD_MIN_COMMAND_FREQUENCY_HZ            20L
#define ROD_MAX_COMMAND_FREQUENCY_HZ           700L
/* Open-loop stepper acceleration limit: caps how much the applied STEP
   frequency may change per 10 ms app tick, so encoder noise or gain-driven
   sign flips near the angle deadband cannot command an instantaneous
   direction/speed jump the motor cannot physically follow (audible chatter). */
#define ROD_COMMAND_RAMP_HZ_PER_TICK            35L
#define ROD_STEPPER_COMMAND_SIGN                 1L

#define ROD_DIRECTION_CHECK_DELAY_MS           100U
#define ROD_DIRECTION_CHECK_DELTA_CDEG           40L

#define ROD_HOME_FEEDBACK_TIMEOUT_MS           2000U
#define ROD_HOME_MOTION_TIMEOUT_MS            10000U
#define ROD_HOME_MAX_FREQUENCY_HZ               200L
#define ROD_HOME_ANGLE_DEADBAND_CDEG              50L
#define ROD_HOME_COMMAND_STEPS_PER_REV          6400L

/*
 * Debug bypass: skip vision UART in BASIC_NAV mode.
 *
 * When BASIC_NAV_BYPASS_VISION is 1, the BASIC_NAV start sequence
 * skips APP_STATE_FIXED_DETECT and directly sets target to
 * BASIC_NAV_DEBUG_TARGET, then starts running.
 * This allows field testing of a single navigation route without
 * waiting for vision recognition.
 *
 * Set BASIC_NAV_BYPASS_VISION back to 0 before production/vision
 * integration testing.
 *
 * Current debug target: TARGET_D (center -> D route test).
 * NAV/ADV route tables for A/B/C/D are selected in chassis_iface.c.
 */
#define BASIC_NAV_BYPASS_VISION   1
#define BASIC_NAV_DEBUG_TARGET    TARGET_D

static AppState_t    state;
static RunMode_t     run_mode;
static TargetPoint_t target;
static uint32_t      state_ticks;
static uint8_t       a_line_count;
static uint8_t       a_line_confirm_ticks;
static bool          a_line_latched;
static uint8_t       p4_b_line_confirm_ticks;
static bool          p4_b_line_latched;
static bool          mode_select_active;
static uint8_t       mode_press_count;
static uint16_t      mode_select_ticks;
static uint8_t       ir_debug_ticks;
static int32_t       rod_home_target_cdeg;
#if APP_Q3_RUNTIME_ENABLED
static bool          q3_start_requested;
#endif

typedef struct {
    BallControlSnapshot_t snapshot;
    uint32_t last_vision_sample_count;
    int32_t last_raw_position_mm_x10;
    int32_t filtered_velocity_mm_s_x10;
    int32_t direction_start_angle_cdeg;
    int32_t pending_position_mm_x10;
    int32_t stuck_reference_position_mm_x10;
    uint32_t direction_start_ms;
    uint32_t last_accepted_vision_ms;
    uint32_t homing_started_ms;
    uint32_t last_position_update_ms;
    uint32_t stuck_since_ms;
    int32_t applied_command_hz;
    uint8_t pending_position_count;
    int8_t direction_command_sign;
    bool has_last_position;
    bool stuck_tracking;
    bool direction_check_active;
    bool homing_stepper_enabled;
    bool homing_move_started;
    BallFrictionState_t friction_state;
} BallControlRuntime_t;

static BallControlRuntime_t ball_control;

typedef struct {
    uint32_t started_ms;
    uint32_t stable_since_ms;
} AppQ3Runtime_t;

#if APP_Q3_RUNTIME_ENABLED
static AppQ3Runtime_t q3_test;
#endif

static void ball_control_stop_motion(BallControlState_t next_state);

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int32_t abs_i32(int32_t value)
{
    return value < 0 ? (int32_t)(-(int64_t)value) : value;
}

static void app_debug_send_char(char value)
{
    while (DL_UART_Main_isTXFIFOFull(DEBUG_UART_INST)) {}
    DL_UART_Main_transmitData(DEBUG_UART_INST, (uint8_t)value);
}

static void app_debug_send_str(const char *value)
{
    while (*value != '\0') {
        app_debug_send_char(*value++);
    }
}

static void app_debug_send_int(int value)
{
    char digits[12];
    uint8_t count = 0U;
    unsigned int magnitude;

    if (value < 0) {
        app_debug_send_char('-');
        magnitude = (unsigned int)(-value);
    } else {
        magnitude = (unsigned int)value;
    }

    do {
        digits[count++] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while (magnitude > 0U && count < sizeof(digits));

    while (count > 0U) {
        app_debug_send_char(digits[--count]);
    }
}

static void p4_report_encoder_at_b(void)
{
    int left_count;
    int right_count;
    int avg_count;

    chassis_encoder_get_counts(&left_count, &right_count);
    avg_count = ((left_count < 0 ? -left_count : left_count) +
                 (right_count < 0 ? -right_count : right_count)) / 2;

    app_debug_send_str("P4_B,");
    app_debug_send_int(left_count);
    app_debug_send_char(',');
    app_debug_send_int(right_count);
    app_debug_send_char(',');
    app_debug_send_int(avg_count);
    app_debug_send_str("\r\n");
}

static void ir_debug_report(const uint8_t values[GRAY_SENSOR_CHANNELS])
{
    uint8_t index;

    app_debug_send_str("IR:");
    for (index = 0U; index < GRAY_SENSOR_CHANNELS; index++) {
        app_debug_send_char(values[index] ? '1' : '0');
    }
    app_debug_send_str("\r\n");
}

#if APP_Q3_RUNTIME_ENABLED
static void q3_set_terminal(AppQ3Phase_t phase, AppQ3Result_t result,
    uint32_t now_ms)
{
    ball_control.snapshot.q3_phase = phase;
    ball_control.snapshot.q3_result = result;
    ball_control.snapshot.q3_elapsed_ms =
        (uint32_t)(now_ms - q3_test.started_ms);
    /* A passed or timed-out item-3 run keeps the closed loop alive. Reaching
       -5 cm must be followed by continued regulation, not a motor stop. */
    if ((result != APP_Q3_RESULT_PASS) &&
        (result != APP_Q3_RESULT_TIMEOUT) &&
        (ball_control.snapshot.state != BALL_CONTROL_FAULT)) {
        ball_control_stop_motion(BALL_CONTROL_HOLD);
    }
    q3_test.stable_since_ms = 0U;
}

static bool q3_condition_stable(bool condition, uint32_t duration_ms,
    uint32_t now_ms)
{
    if (!condition) {
        q3_test.stable_since_ms = 0U;
        return false;
    }
    if (q3_test.stable_since_ms == 0U) {
        q3_test.stable_since_ms = now_ms;
        return false;
    }
    return (uint32_t)(now_ms - q3_test.stable_since_ms) >= duration_ms;
}
#endif /* APP_ROD_Q3_TEST_MODE */

/* Rate-limits the STEP command actually sent to the driver so encoder noise
   or a gain-driven sign flip near the angle deadband cannot demand an
   instantaneous speed/direction jump the open-loop stepper cannot follow
   (that jump is what shows up as audible chatter/vibration at the motor).
   A pass through the (0, ROD_MIN_COMMAND_FREQUENCY_HZ) gap -- an invalid
   region for stepper_set_speed(), see STEPPER_MIN_FREQUENCY_HZ -- is snapped
   to 0 so a direction reversal always transits a brief stop instead of an
   invalid frequency. */
static int32_t ball_control_ramp_command_hz(int32_t desired_hz)
{
    int32_t applied = ball_control.applied_command_hz;
    int32_t delta;

    /* Never keep driving in the old direction after the angle loop requests
       a reversal. Gradually ramping through zero can spend hundreds of
       milliseconds commanding motion away from the new target. Stop for one
       control tick, then ramp up in the new direction on the following tick. */
    if (((desired_hz > 0) && (applied < 0)) ||
        ((desired_hz < 0) && (applied > 0))) {
        ball_control.applied_command_hz = 0;
        return 0;
    }

    delta = clamp_i32(desired_hz - applied,
        -ROD_COMMAND_RAMP_HZ_PER_TICK, ROD_COMMAND_RAMP_HZ_PER_TICK);

    applied += delta;
    if ((applied > -ROD_MIN_COMMAND_FREQUENCY_HZ) &&
        (applied < ROD_MIN_COMMAND_FREQUENCY_HZ)) {
        applied = 0;
    }
    ball_control.applied_command_hz = applied;
    return applied;
}

static void ball_control_reset_stuck_tracking(void)
{
    ball_control.stuck_reference_position_mm_x10 = 0;
    ball_control.stuck_since_ms = 0U;
    ball_control.stuck_tracking = false;
    if (ball_control.friction_state == BALL_FRICTION_STUCK) {
        ball_control.friction_state = BALL_FRICTION_STATIONARY;
    }
}

static void ball_control_reset_friction(void)
{
    ball_control.friction_state = BALL_FRICTION_STATIONARY;
    ball_control_reset_stuck_tracking();
    ball_control.snapshot.friction_state =
        (uint8_t)BALL_FRICTION_STATIONARY;
    ball_control.snapshot.friction_feedforward_cdeg = 0;
}

static int32_t ball_control_friction_feedforward(
    int32_t error_mm_x10, int32_t velocity_mm_s_x10)
{
    int32_t absolute_velocity = abs_i32(velocity_mm_s_x10);
    bool moving_toward_target;

    if (ball_control.friction_state == BALL_FRICTION_STUCK) {
        ball_control.friction_state = BALL_FRICTION_STATIONARY;
    }
    if (ball_control.friction_state == BALL_FRICTION_MOVING) {
        if (absolute_velocity <=
            APP_BALL_STATIC_SPEED_THRESHOLD_MM_S_X10) {
            ball_control.friction_state = BALL_FRICTION_STATIONARY;
        }
    } else if (absolute_velocity >=
               APP_BALL_MOTION_SPEED_THRESHOLD_MM_S_X10) {
        ball_control.friction_state = BALL_FRICTION_MOVING;
    }

    if (ball_control.friction_state == BALL_FRICTION_STATIONARY) {
        if (error_mm_x10 > APP_BALL_POSITION_DEADBAND_MM_X10) {
            return -APP_BALL_STATIC_FRICTION_FF_CDEG;
        }
        if (error_mm_x10 < -APP_BALL_POSITION_DEADBAND_MM_X10) {
            return APP_BALL_STATIC_FRICTION_FF_CDEG;
        }
        return 0;
    }

    /* On this mechanism, rod-angle delta and ball-coordinate acceleration
       have the same sign. Feed forward only while velocity reduces error;
       braking remains entirely under the velocity-damping term. */
    moving_toward_target =
        ((velocity_mm_s_x10 > 0) && (error_mm_x10 < 0)) ||
        ((velocity_mm_s_x10 < 0) && (error_mm_x10 > 0));
    if (!moving_toward_target) {
        return 0;
    }
    return velocity_mm_s_x10 > 0 ?
        APP_BALL_KINETIC_FRICTION_FF_CDEG :
        -APP_BALL_KINETIC_FRICTION_FF_CDEG;
}

static int32_t ball_control_stuck_feedforward(int32_t error_mm_x10,
    int32_t position_mm_x10, uint32_t now_ms)
{
    if (abs_i32(error_mm_x10) <= APP_BALL_POSITION_DEADBAND_MM_X10) {
        ball_control_reset_stuck_tracking();
        return 0;
    }
    if (abs_i32(error_mm_x10) > APP_BALL_STUCK_NEAR_TARGET_BAND_MM_X10) {
        /* Far from target: a momentary low-speed/low-motion reading here is
           the natural turnaround of a swing, not a friction stick. Do not
           accumulate stuck time or apply any boost, and do not let a brief
           in-band dip while passing through this outer region seed tracking
           that would otherwise carry over once the ball later gets close. */
        ball_control_reset_stuck_tracking();
        return 0;
    }

    if (!ball_control.stuck_tracking ||
        (abs_i32(position_mm_x10 -
            ball_control.stuck_reference_position_mm_x10) >
         APP_BALL_STUCK_POSITION_BAND_MM_X10)) {
        ball_control.stuck_reference_position_mm_x10 = position_mm_x10;
        ball_control.stuck_since_ms = now_ms;
        ball_control.stuck_tracking = true;
        return 0;
    }

    if ((uint32_t)(now_ms - ball_control.stuck_since_ms) <
        APP_BALL_STUCK_CONFIRM_MS) {
        return 0;
    }

    {
        /* stuck_since_ms marks when the ball first entered the current
           reference band and is not refreshed while it stays inside it, so
           this duration grows for as long as the ball remains stuck. A
           single fixed boost sometimes was not enough to break the ball
           free -- the rod would settle at the boosted angle and just sit
           there. Escalate with time so a stubborn stick eventually gets a
           stronger kick, capped well short of the hard travel guards. */
        uint32_t stuck_duration_ms =
            (uint32_t)(now_ms - ball_control.stuck_since_ms);
        uint32_t escalation_steps =
            (stuck_duration_ms - APP_BALL_STUCK_CONFIRM_MS) /
            APP_BALL_STUCK_FF_BOOST_STEP_MS;
        int32_t boost_cdeg = clamp_i32(
            APP_BALL_STUCK_FF_BOOST_CDEG +
                (int32_t)(escalation_steps * APP_BALL_STUCK_FF_BOOST_STEP_CDEG),
            APP_BALL_STUCK_FF_BOOST_CDEG, APP_BALL_STUCK_FF_BOOST_MAX_CDEG);

        ball_control.friction_state = BALL_FRICTION_STUCK;
        return error_mm_x10 > 0 ? -boost_cdeg : boost_cdeg;
    }
}

static void ball_control_stop_motion(BallControlState_t next_state)
{
    stepper_stop();
    ball_control.snapshot.state = next_state;
    ball_control.snapshot.step_frequency_hz = 0;
    ball_control.applied_command_hz = 0;
    ball_control.direction_check_active = false;
    ball_control.direction_command_sign = 0;
}

static void ball_control_fault(BallControlFault_t reason)
{
    /* Keep EN asserted after control-quality faults so the gravity-loaded
       rod retains holding torque. Explicit electrical/operator emergencies
       may still call stepper_emergency_stop() elsewhere. */
    stepper_stop();
    ball_control.snapshot.state = BALL_CONTROL_FAULT;
    ball_control.snapshot.step_frequency_hz = 0;
    ball_control.applied_command_hz = 0;
    ball_control.snapshot.fault_reason = (uint8_t)reason;
    ball_control.direction_check_active = false;
    ball_control.direction_command_sign = 0;
}

static void ball_control_init(void)
{
    ball_control.snapshot.state = BALL_CONTROL_OFF;
    ball_control.snapshot.target_position_mm_x10 =
        BALL_TARGET_POSITION_MM_X10;
    ball_control.snapshot.filtered_position_mm_x10 = 0;
    ball_control.snapshot.ball_error_mm_x10 = 0;
    ball_control.snapshot.target_angle_cdeg = ROD_NEUTRAL_ANGLE_CDEG;
    ball_control.snapshot.rod_error_cdeg = 0;
    ball_control.snapshot.step_frequency_hz = 0;
    ball_control.snapshot.filtered_velocity_mm_s_x10 = 0;
    ball_control.snapshot.friction_feedforward_cdeg = 0;
    ball_control.snapshot.rejected_samples = 0U;
    ball_control.snapshot.valid_streak = 0U;
    ball_control.snapshot.fault_reason = BALL_CONTROL_FAULT_NONE;
    ball_control.snapshot.friction_state =
        (uint8_t)BALL_FRICTION_STATIONARY;
    ball_control.snapshot.homed = false;
    ball_control.snapshot.q3_phase = APP_Q3_IDLE;
    ball_control.snapshot.q3_elapsed_ms = 0U;
    ball_control.snapshot.q3_plus_reached_ms = 0U;
    ball_control.snapshot.q3_result = APP_Q3_RESULT_NONE;
    ball_control.last_vision_sample_count = 0U;
    ball_control.last_raw_position_mm_x10 = 0;
    ball_control.filtered_velocity_mm_s_x10 = 0;
    ball_control.direction_start_angle_cdeg = 0;
    ball_control.pending_position_mm_x10 = 0;
    ball_control.stuck_reference_position_mm_x10 = 0;
    ball_control.applied_command_hz = 0;
    ball_control.direction_start_ms = 0U;
    ball_control.last_accepted_vision_ms = 0U;
    ball_control.homing_started_ms = 0U;
    ball_control.last_position_update_ms = 0U;
    ball_control.stuck_since_ms = 0U;
    ball_control.pending_position_count = 0U;
    ball_control.direction_command_sign = 0;
    ball_control.has_last_position = false;
    ball_control.stuck_tracking = false;
    ball_control.direction_check_active = false;
    ball_control.homing_stepper_enabled = false;
    ball_control.homing_move_started = false;
    ball_control.friction_state = BALL_FRICTION_STATIONARY;
#if APP_Q3_RUNTIME_ENABLED
    q3_test.started_ms = 0U;
    q3_test.stable_since_ms = 0U;
    q3_start_requested = false;
#endif
}

static void ball_control_begin_homing(void)
{
    stepper_stop();
    ball_control.snapshot.state = BALL_CONTROL_HOMING;
    ball_control.snapshot.target_angle_cdeg = rod_home_target_cdeg;
    ball_control.snapshot.rod_error_cdeg = 0;
    ball_control.snapshot.step_frequency_hz = 0;
    ball_control.applied_command_hz = 0;
    ball_control.snapshot.filtered_velocity_mm_s_x10 = 0;
    ball_control_reset_friction();
    ball_control.snapshot.fault_reason = BALL_CONTROL_FAULT_NONE;
    ball_control.snapshot.homed = false;
    ball_control.homing_started_ms = system_time_ms();
    ball_control.last_position_update_ms = 0U;
    ball_control.homing_stepper_enabled = false;
    ball_control.homing_move_started = false;
    ball_control.direction_check_active = false;
    ball_control.direction_command_sign = 0;
}

static void ball_control_start(void)
{
    VisionTelemetry_t vision;

    (void)vision_uart_get_telemetry(&vision);
    ball_control.last_vision_sample_count = vision.sample_count;
    ball_control.has_last_position = false;
    ball_control.pending_position_count = 0U;
    ball_control.last_accepted_vision_ms = system_time_ms();
    ball_control.last_position_update_ms = 0U;
    ball_control.snapshot.valid_streak = 0U;
    ball_control.filtered_velocity_mm_s_x10 = 0;
    ball_control.snapshot.filtered_velocity_mm_s_x10 = 0;
    ball_control_reset_friction();
    ball_control.snapshot.target_angle_cdeg = ROD_NEUTRAL_ANGLE_CDEG;
    ball_control.snapshot.ball_error_mm_x10 = 0;
    ball_control.snapshot.rod_error_cdeg = 0;
    ball_control.snapshot.step_frequency_hz = 0;
    ball_control.applied_command_hz = 0;
    ball_control.direction_check_active = false;
    ball_control.snapshot.fault_reason = BALL_CONTROL_FAULT_NONE;
    if (!ball_control.snapshot.homed) {
        ball_control_fault(BALL_CONTROL_FAULT_NOT_HOMED);
        return;
    }
    if (!stepper_enable()) {
        ball_control_fault(BALL_CONTROL_FAULT_COMMAND);
        return;
    }
    ball_control.snapshot.state = BALL_CONTROL_WAITING;
}

static void ball_control_stop(void)
{
    ball_control_stop_motion(BALL_CONTROL_OFF);
    ball_control.snapshot.valid_streak = 0U;
    ball_control.has_last_position = false;
    ball_control.pending_position_count = 0U;
    ball_control.last_position_update_ms = 0U;
    ball_control_reset_friction();
}

static bool ball_control_accept_new_vision(const VisionTelemetry_t *vision)
{
    int32_t jump;
    int32_t pending_jump;
    int32_t position_term;
    int32_t position_gain;
    int32_t velocity_term;
    int32_t target_offset;
    int32_t desired_target_angle;
    int32_t target_delta;
    int32_t absolute_ball_error;
    int32_t friction_feedforward;
    int32_t velocity_divisor = BALL_VELOCITY_DIVISOR;
    int32_t target_offset_limit = ROD_TARGET_OFFSET_LIMIT_CDEG;
    int32_t friction_ff_limit = ROD_FRICTION_FF_HEADROOM_CDEG;
    int32_t target_slew = ROD_TARGET_SLEW_CDEG_PER_SAMPLE;
    int32_t previous_filtered_position;
    int32_t observed_velocity;
    uint32_t sample_period_ms;
    bool had_position_history;
    bool reacquired_position = false;

    if ((vision->position_mm_x10 < BALL_POSITION_MIN_MM_X10) ||
        (vision->position_mm_x10 > BALL_POSITION_MAX_MM_X10)) {
        return false;
    }
    previous_filtered_position =
        ball_control.snapshot.filtered_position_mm_x10;
    had_position_history = ball_control.has_last_position;
    if (ball_control.has_last_position) {
        jump = vision->position_mm_x10 -
            ball_control.last_raw_position_mm_x10;
        if ((jump < -BALL_MAX_SAMPLE_JUMP_MM_X10) ||
            (jump > BALL_MAX_SAMPLE_JUMP_MM_X10)) {
            /* A jump this large is usually a misdetection. Require a run of
               mutually consistent samples at the new location before trusting
               it, so one bad frame cannot yank the target angle. */
            pending_jump = vision->position_mm_x10 -
                ball_control.pending_position_mm_x10;
            if ((ball_control.pending_position_count == 0U) ||
                (pending_jump < -BALL_MAX_SAMPLE_JUMP_MM_X10) ||
                (pending_jump > BALL_MAX_SAMPLE_JUMP_MM_X10)) {
                ball_control.pending_position_mm_x10 =
                    vision->position_mm_x10;
                ball_control.pending_position_count = 1U;
            } else {
                ball_control.pending_position_count++;
            }
            if (ball_control.pending_position_count <
                BALL_REACQUIRE_STREAK_REQUIRED) {
                return false;
            }
            reacquired_position = true;
            ball_control.pending_position_count = 0U;
        }
        if (ball_control.has_last_position) {
            ball_control.snapshot.filtered_position_mm_x10 =
                (ball_control.snapshot.filtered_position_mm_x10 +
                 vision->position_mm_x10) / 2;
        } else {
            ball_control.snapshot.filtered_position_mm_x10 =
                vision->position_mm_x10;
            ball_control.has_last_position = true;
        }
    } else {
        pending_jump = vision->position_mm_x10 -
            ball_control.pending_position_mm_x10;
        if ((ball_control.pending_position_count == 0U) ||
            (pending_jump < -BALL_MAX_SAMPLE_JUMP_MM_X10) ||
            (pending_jump > BALL_MAX_SAMPLE_JUMP_MM_X10)) {
            ball_control.pending_position_mm_x10 =
                vision->position_mm_x10;
            ball_control.pending_position_count = 1U;
        } else {
            ball_control.pending_position_count++;
        }
        if (ball_control.pending_position_count <
            BALL_REACQUIRE_STREAK_REQUIRED) {
            return false;
        }
        reacquired_position = true;
        ball_control.snapshot.filtered_position_mm_x10 =
            vision->position_mm_x10;
        ball_control.has_last_position = true;
    }
    /* Velocity is differentiated locally rather than taken from the sender's
       velocity_mm_s field, so it always matches the filtered position this
       loop is actually acting on. */
    if (had_position_history && !reacquired_position &&
        (ball_control.last_position_update_ms != 0U)) {
        sample_period_ms = (uint32_t)(vision->local_receive_ms -
            ball_control.last_position_update_ms);
        if ((sample_period_ms >= 10U) && (sample_period_ms <= 200U)) {
            observed_velocity = (int32_t)(
                ((int64_t)(ball_control.snapshot.filtered_position_mm_x10 -
                    previous_filtered_position) * 1000L) /
                (int64_t)sample_period_ms);
            observed_velocity = clamp_i32(observed_velocity,
                -BALL_VELOCITY_LIMIT_MM_S_X10,
                BALL_VELOCITY_LIMIT_MM_S_X10);
            ball_control.filtered_velocity_mm_s_x10 =
                (ball_control.filtered_velocity_mm_s_x10 +
                 observed_velocity) / 2L;
        } else {
            ball_control.filtered_velocity_mm_s_x10 = 0;
        }
    } else {
        ball_control.filtered_velocity_mm_s_x10 = 0;
    }
    ball_control.last_position_update_ms = vision->local_receive_ms;
    ball_control.pending_position_count = 0U;
    ball_control.last_raw_position_mm_x10 = vision->position_mm_x10;
    ball_control.snapshot.ball_error_mm_x10 =
        ball_control.snapshot.filtered_position_mm_x10 -
        ball_control.snapshot.target_position_mm_x10;
    ball_control.snapshot.filtered_velocity_mm_s_x10 =
        ball_control.filtered_velocity_mm_s_x10;
    absolute_ball_error = abs_i32(ball_control.snapshot.ball_error_mm_x10);
    if (absolute_ball_error >
        BALL_POSITION_FAR_THRESHOLD_MM_X10) {
        position_gain = BALL_POSITION_FAR_GAIN_CDEG_PER_MM;
    } else if (absolute_ball_error >
               BALL_POSITION_MID_THRESHOLD_MM_X10) {
        position_gain = BALL_POSITION_MID_GAIN_CDEG_PER_MM;
    } else {
        position_gain = BALL_POSITION_NEAR_GAIN_CDEG_PER_MM;
    }
#if APP_Q3_RUNTIME_ENABLED
    if ((ball_control.snapshot.q3_phase == APP_Q3_MOVE_PLUS) ||
        (ball_control.snapshot.q3_phase == APP_Q3_MOVE_MINUS)) {
        if (absolute_ball_error >
            BALL_POSITION_FAR_THRESHOLD_MM_X10) {
            position_gain = BALL_POSITION_FAR_GAIN_CDEG_PER_MM;
        } else if (absolute_ball_error >
                   BALL_POSITION_MID_THRESHOLD_MM_X10) {
            position_gain = BALL_Q3_POSITION_MID_GAIN_CDEG_PER_MM;
        } else {
            position_gain = BALL_Q3_POSITION_NEAR_GAIN_CDEG_PER_MM;
        }
    } else if (((ball_control.snapshot.q3_phase == APP_Q3_SETTLE_MINUS) ||
                (ball_control.snapshot.q3_phase == APP_Q3_DONE) ||
                (ball_control.snapshot.q3_phase == APP_Q3_TIMEOUT)) &&
               (absolute_ball_error <=
                APP_Q3_SETTLE_ENTRY_BAND_MM_X10)) {
        /* The quiet hold gains assume the ball is already close; they trade
           away correction authority for smoothness. Gate them on actual
           proximity rather than just the phase name so that a DONE/TIMEOUT
           run left holding a large residual error (e.g. after drifting off
           target once q3_tick() stops running these terminal phases) still
           gets full-strength correction instead of being stuck applying a
           tilt too weak to close the gap. */
        if (absolute_ball_error >
            BALL_POSITION_FAR_THRESHOLD_MM_X10) {
            position_gain = BALL_Q3_HOLD_FAR_GAIN_CDEG_PER_MM;
        } else if (absolute_ball_error >
                   BALL_POSITION_MID_THRESHOLD_MM_X10) {
            position_gain = BALL_Q3_HOLD_MID_GAIN_CDEG_PER_MM;
        } else {
            position_gain = BALL_Q3_HOLD_NEAR_GAIN_CDEG_PER_MM;
        }
        velocity_divisor = BALL_Q3_HOLD_VELOCITY_DIVISOR;
        target_offset_limit = ROD_Q3_HOLD_OFFSET_LIMIT_CDEG;
        friction_ff_limit = ROD_Q3_HOLD_FF_LIMIT_CDEG;
        target_slew = ROD_Q3_HOLD_SLEW_CDEG_PER_SAMPLE;
    }
#endif
    position_term =
        (ball_control.snapshot.ball_error_mm_x10 *
         position_gain) / 10L;
    velocity_term = clamp_i32(
        ball_control.filtered_velocity_mm_s_x10 /
            velocity_divisor,
        -BALL_VELOCITY_TARGET_LIMIT_CDEG,
        BALL_VELOCITY_TARGET_LIMIT_CDEG);
    target_offset = clamp_i32(
        position_term + velocity_term,
        -target_offset_limit,
        target_offset_limit);
    friction_feedforward = ball_control_friction_feedforward(
        ball_control.snapshot.ball_error_mm_x10,
        ball_control.filtered_velocity_mm_s_x10);
    friction_feedforward += ball_control_stuck_feedforward(
        ball_control.snapshot.ball_error_mm_x10,
        ball_control.snapshot.filtered_position_mm_x10,
        vision->local_receive_ms);
    if ((absolute_ball_error <= APP_BALL_POSITION_DEADBAND_MM_X10) &&
        (abs_i32(ball_control.filtered_velocity_mm_s_x10) <=
         APP_BALL_STATIC_SPEED_THRESHOLD_MM_S_X10)) {
        target_offset = 0;
        friction_feedforward = 0;
    }
    /* Clamp the feedforward on its own reserved headroom before adding it,
       so a P/D term already pinned at ROD_TARGET_OFFSET_LIMIT_CDEG cannot
       crowd it out in the combined clamp below. */
    friction_feedforward = clamp_i32(friction_feedforward,
        -friction_ff_limit, friction_ff_limit);
    ball_control.snapshot.friction_state =
        (uint8_t)ball_control.friction_state;
    ball_control.snapshot.friction_feedforward_cdeg =
        friction_feedforward;
    desired_target_angle = clamp_i32(
        ROD_NEUTRAL_ANGLE_CDEG - target_offset + friction_feedforward,
        ROD_NEUTRAL_ANGLE_CDEG -
            (target_offset_limit + friction_ff_limit),
        ROD_NEUTRAL_ANGLE_CDEG +
            (target_offset_limit + friction_ff_limit));
    target_delta = clamp_i32(
        desired_target_angle - ball_control.snapshot.target_angle_cdeg,
        -target_slew,
        target_slew);
    ball_control.snapshot.target_angle_cdeg += target_delta;
    if (ball_control.snapshot.valid_streak <
        BALL_VALID_STREAK_REQUIRED) {
        ball_control.snapshot.valid_streak++;
    }
    return true;
}

/* Catches a reversed DIR wiring or a mechanically stalled rod: if the angle
   has moved opposite to the commanded direction by more than the tolerance
   after the settle delay, the command is not producing the intended motion. */
static bool ball_control_direction_is_valid(
    int32_t command_hz, int32_t angle_cdeg, uint32_t now_ms)
{
    int8_t command_sign = command_hz > 0 ? 1 : -1;
    int32_t delta;

    if (!ball_control.direction_check_active ||
        (command_sign != ball_control.direction_command_sign)) {
        ball_control.direction_check_active = true;
        ball_control.direction_command_sign = command_sign;
        ball_control.direction_start_angle_cdeg = angle_cdeg;
        ball_control.direction_start_ms = now_ms;
        return true;
    }
    if ((uint32_t)(now_ms - ball_control.direction_start_ms) <
        ROD_DIRECTION_CHECK_DELAY_MS) {
        return true;
    }
    delta = angle_cdeg - ball_control.direction_start_angle_cdeg;
    if (((command_sign > 0) &&
         (delta <= -ROD_DIRECTION_CHECK_DELTA_CDEG)) ||
        ((command_sign < 0) &&
         (delta >= ROD_DIRECTION_CHECK_DELTA_CDEG))) {
        return false;
    }
    ball_control.direction_start_angle_cdeg = angle_cdeg;
    ball_control.direction_start_ms = now_ms;
    return true;
}

static void ball_control_home_tick(void)
{
    uint32_t now_ms;
    StepperFeedbackSnapshot_t encoder;
    int32_t angle_cdeg;
    int32_t angle_error;
    int32_t command_pulses;

    if ((ball_control.snapshot.state != BALL_CONTROL_HOMING) ||
        ball_control.snapshot.homed) {
        return;
    }
    now_ms = system_time_ms();
    if ((uint32_t)(now_ms - ball_control.homing_started_ms) >
        ROD_HOME_MOTION_TIMEOUT_MS) {
        ball_control_fault(BALL_CONTROL_FAULT_HOME_MOTION_TIMEOUT);
        return;
    }
    encoder = stepper_feedback_get_snapshot();
    if (!encoder.abs_pwm_valid ||
        ((uint32_t)(now_ms - encoder.abs_pwm_timestamp_ms) >
         ROD_ABS_FEEDBACK_TIMEOUT_MS)) {
        if (ball_control.homing_move_started) {
            ball_control_fault(BALL_CONTROL_FAULT_HOME_FEEDBACK_TIMEOUT);
            return;
        }
        /* Before the first jog, allow a longer grace period: PA26 needs a
           couple of capture windows after power-on to produce a reading. */
        if ((uint32_t)(now_ms - ball_control.homing_started_ms) >
            ROD_HOME_FEEDBACK_TIMEOUT_MS) {
            ball_control_fault(
                BALL_CONTROL_FAULT_HOME_FEEDBACK_TIMEOUT);
        }
        return;
    }
    if (!ball_control.homing_stepper_enabled) {
        if (!stepper_enable()) {
            ball_control_fault(BALL_CONTROL_FAULT_COMMAND);
            return;
        }
        ball_control.homing_stepper_enabled = true;
    }

    angle_cdeg = (int32_t)encoder.abs_pwm_angle_cdeg;
    angle_error = rod_home_target_cdeg - angle_cdeg;
    ball_control.snapshot.rod_error_cdeg = angle_error;

    /* A homing jog is issued only once, but PA26 remains an active stop and
       safety input while those finite pulses are being emitted. */
    if (ball_control.homing_move_started) {
        if ((angle_error >= -ROD_HOME_ANGLE_DEADBAND_CDEG) &&
            (angle_error <= ROD_HOME_ANGLE_DEADBAND_CDEG)) {
            stepper_stop();
            ball_control.snapshot.step_frequency_hz = 0;
            ball_control.snapshot.homed = true;
            ball_control.snapshot.state = BALL_CONTROL_OFF;
            ball_control.snapshot.fault_reason = BALL_CONTROL_FAULT_NONE;
            return;
        }
        if (((ball_control.direction_command_sign > 0) &&
             (angle_cdeg >= ROD_HARD_MAX_ANGLE_CDEG)) ||
            ((ball_control.direction_command_sign < 0) &&
             (angle_cdeg <= ROD_HARD_MIN_ANGLE_CDEG))) {
            ball_control_fault(BALL_CONTROL_FAULT_HARD_LIMIT);
            return;
        }
        if (!ball_control_direction_is_valid(
                ball_control.direction_command_sign, angle_cdeg, now_ms)) {
            ball_control_fault(BALL_CONTROL_FAULT_DIRECTION);
            return;
        }
        if (!stepper_is_busy()) {
            /* The one permitted jog ended without reaching the target. Do
               not issue a corrective jog in homing-only mode. */
            ball_control_fault(BALL_CONTROL_FAULT_COMMAND);
        }
        return;
    }

    if ((angle_error >= -ROD_HOME_ANGLE_DEADBAND_CDEG) &&
        (angle_error <= ROD_HOME_ANGLE_DEADBAND_CDEG)) {
        stepper_stop();
        ball_control.snapshot.step_frequency_hz = 0;
        ball_control.snapshot.homed = true;
        ball_control.snapshot.state = BALL_CONTROL_OFF;
        ball_control.snapshot.fault_reason = BALL_CONTROL_FAULT_NONE;
        return;
    }
    command_pulses = (int32_t)(
        ((int64_t)angle_error * ROD_HOME_COMMAND_STEPS_PER_REV) /
        36000L);
    command_pulses *= ROD_STEPPER_COMMAND_SIGN;
    if (command_pulses == 0) {
        stepper_stop();
        ball_control.snapshot.homed = true;
        ball_control.snapshot.state = BALL_CONTROL_OFF;
        return;
    }
    if (((angle_cdeg <= ROD_HARD_MIN_ANGLE_CDEG) &&
         (angle_error < 0)) ||
        ((angle_cdeg >= ROD_HARD_MAX_ANGLE_CDEG) &&
         (angle_error > 0))) {
        ball_control_fault(BALL_CONTROL_FAULT_HARD_LIMIT);
        return;
    }
    if (!stepper_jog(command_pulses,
            (uint32_t)ROD_HOME_MAX_FREQUENCY_HZ)) {
        ball_control_fault(BALL_CONTROL_FAULT_COMMAND);
        return;
    }
    ball_control.homing_move_started = true;
    ball_control.direction_check_active = false;
    ball_control.direction_command_sign = angle_error > 0 ? 1 : -1;
    ball_control.snapshot.state = BALL_CONTROL_HOMING;
    ball_control.snapshot.step_frequency_hz =
        command_pulses > 0 ? ROD_HOME_MAX_FREQUENCY_HZ :
        -ROD_HOME_MAX_FREQUENCY_HZ;
}

/* PA26 remains the absolute rod-position feedback.  In item 4 the IMU X gyro
   supplies a velocity damping term: with the stated mounting, the rod axis is
   Y and its tilt motion is a rotation around X.  An absent/stale IMU merely
   removes this extra damping; it never disables the existing PA26 safety loop. */
static int32_t ball_control_imu_x_damping_cdeg(uint32_t now_ms)
{
#if P4_IMU_BALANCE_ENABLE || P56_IMU_BALANCE_ENABLE
    ImuSnapshot_t imu;
    int32_t rate_dps_x10;
    int32_t damping_cdeg;
    int32_t sign;
    int32_t gain;
    int32_t limit;
    uint32_t max_age_ms;

    if (run_mode == RUN_MODE_PROBLEM4) {
#if P4_IMU_BALANCE_ENABLE
        sign = P4_IMU_GYRO_X_SIGN;
        gain = P4_IMU_GYRO_X_DAMP_CDEG_PER_DPS_X10;
        limit = P4_IMU_GYRO_X_DAMP_LIMIT_CDEG;
        max_age_ms = P4_IMU_MAX_AGE_MS;
#else
        return 0;
#endif
    } else if (run_mode == RUN_MODE_PROBLEM56) {
#if P56_IMU_BALANCE_ENABLE
        sign = P56_IMU_GYRO_X_SIGN;
        gain = P56_IMU_GYRO_X_DAMP_CDEG_PER_DPS_X10;
        limit = P56_IMU_GYRO_X_DAMP_LIMIT_CDEG;
        max_age_ms = P56_IMU_MAX_AGE_MS;
#else
        return 0;
#endif
    } else {
        return 0;
    }
    imu = imu_sensing_get_snapshot();
    if (!imu.valid ||
        ((uint32_t)(now_ms - imu.timestamp_ms) > max_age_ms)) {
        return 0;
    }

    rate_dps_x10 = (int32_t)imu.scaled.gyro_dps_x10[ICM42688P_AXIS_X] *
        sign;
    damping_cdeg = rate_dps_x10 * gain;
    return clamp_i32(damping_cdeg, -limit, limit);
#else
    (void)now_ms;
    return 0;
#endif
}

static void ball_control_tick(void)
{
    uint32_t now_ms = system_time_ms();
    VisionTelemetry_t vision;
    StepperFeedbackSnapshot_t encoder;
    bool vision_available;
    int32_t angle_cdeg;
    int32_t angle_error;
    int32_t command_hz;

    if (ball_control.snapshot.state == BALL_CONTROL_FAULT) {
        return;
    }
    (void)vision_uart_get_telemetry(&vision);
    encoder = stepper_feedback_get_snapshot();
    vision_available = vision.link_fresh && vision.ball_valid;
    /* Vision or angle feedback lost: stop emitting STEP but keep EN asserted,
       so the gravity-loaded rod holds instead of dropping. */
    if (!vision_available &&
        ((uint32_t)(now_ms - ball_control.last_accepted_vision_ms) >
         BALL_INVALID_GRACE_MS)) {
        ball_control.snapshot.valid_streak = 0U;
        ball_control.has_last_position = false;
        ball_control.pending_position_count = 0U;
        ball_control.last_position_update_ms = 0U;
        ball_control_reset_friction();
        ball_control_stop_motion(BALL_CONTROL_HOLD);
        return;
    }
    if (!encoder.abs_pwm_valid ||
        ((uint32_t)(now_ms - encoder.abs_pwm_timestamp_ms) >
         ROD_ABS_FEEDBACK_TIMEOUT_MS)) {
        ball_control_reset_friction();
        ball_control_stop_motion(BALL_CONTROL_HOLD);
        return;
    }
    if (vision_available &&
        (vision.sample_count != ball_control.last_vision_sample_count)) {
        ball_control.last_vision_sample_count = vision.sample_count;
        if (!ball_control_accept_new_vision(&vision)) {
            ball_control.snapshot.rejected_samples++;
            if ((uint32_t)(now_ms -
                    ball_control.last_accepted_vision_ms) >
                BALL_ACCEPTED_SAMPLE_TIMEOUT_MS) {
                ball_control.snapshot.valid_streak = 0U;
                ball_control_reset_friction();
                ball_control_stop_motion(BALL_CONTROL_HOLD);
                return;
            }
        } else {
            ball_control.last_accepted_vision_ms = now_ms;
        }
    }
    if (ball_control.snapshot.valid_streak <
        BALL_VALID_STREAK_REQUIRED) {
        ball_control_stop_motion(BALL_CONTROL_WAITING);
        return;
    }

    angle_cdeg = (int32_t)encoder.abs_pwm_angle_cdeg;
    angle_error = ball_control.snapshot.target_angle_cdeg - angle_cdeg;
    /* Rate damping is applied after the visual outer loop has chosen its
       target angle, so it damps only rod motion and does not bias the desired
       ball position. */
    angle_error -= ball_control_imu_x_damping_cdeg(now_ms);
    ball_control.snapshot.rod_error_cdeg = angle_error;
    if ((angle_error >= -ROD_ANGLE_DEADBAND_CDEG) &&
        (angle_error <= ROD_ANGLE_DEADBAND_CDEG)) {
        ball_control_stop_motion(BALL_CONTROL_ACTIVE);
        return;
    }
    command_hz = angle_error * ROD_SPEED_GAIN_HZ_PER_CDEG;
    command_hz = clamp_i32(command_hz,
        -ROD_MAX_COMMAND_FREQUENCY_HZ,
        ROD_MAX_COMMAND_FREQUENCY_HZ);
    if ((command_hz > 0) &&
        (command_hz < ROD_MIN_COMMAND_FREQUENCY_HZ)) {
        command_hz = ROD_MIN_COMMAND_FREQUENCY_HZ;
    } else if ((command_hz < 0) &&
               (command_hz > -ROD_MIN_COMMAND_FREQUENCY_HZ)) {
        command_hz = -ROD_MIN_COMMAND_FREQUENCY_HZ;
    }
    command_hz *= ROD_STEPPER_COMMAND_SIGN;
    command_hz = ball_control_ramp_command_hz(command_hz);
    if (command_hz == 0) {
        /* Ramp is transiting a direction reversal through zero; the motor
           sits idle for this tick rather than receiving an invalid
           sub-minimum frequency. */
        stepper_stop();
        ball_control.snapshot.state = BALL_CONTROL_ACTIVE;
        ball_control.snapshot.step_frequency_hz = 0;
        ball_control.direction_check_active = false;
        ball_control.direction_command_sign = 0;
        return;
    }

    if (((angle_cdeg <= ROD_HARD_MIN_ANGLE_CDEG) &&
         (command_hz < 0)) ||
        ((angle_cdeg >= ROD_HARD_MAX_ANGLE_CDEG) &&
         (command_hz > 0))) {
        ball_control_fault(BALL_CONTROL_FAULT_HARD_LIMIT);
        return;
    }
    if (!ball_control_direction_is_valid(
            command_hz, angle_cdeg, now_ms)) {
        ball_control_fault(BALL_CONTROL_FAULT_DIRECTION);
        return;
    }
    if (!stepper_set_speed(command_hz)) {
        ball_control_fault(BALL_CONTROL_FAULT_COMMAND);
        return;
    }
    ball_control.snapshot.state = BALL_CONTROL_ACTIVE;
    ball_control.snapshot.step_frequency_hz = command_hz;
}

#if APP_Q3_RUNTIME_ENABLED
static void q3_prepare_ready(void)
{
    chassis_lock();
    /* READY is intentionally passive: after the one-time PA26 homing move,
       do not use vision or issue any further STEP correction until PA25 is
       pressed to start the item-3 sequence. */
    ball_control_stop();
    if (!app_state_set_ball_target_position_mm_x10(
            APP_Q3_CENTER_POSITION_MM_X10)) {
        ball_control.snapshot.q3_phase = APP_Q3_FAULT;
        ball_control.snapshot.q3_result = APP_Q3_RESULT_FAULT;
        return;
    }
    ball_control.snapshot.q3_phase = APP_Q3_READY;
    ball_control.snapshot.q3_elapsed_ms = 0U;
    ball_control.snapshot.q3_plus_reached_ms = 0U;
    ball_control.snapshot.q3_result = APP_Q3_RESULT_NONE;
    q3_test.started_ms = 0U;
    q3_test.stable_since_ms = 0U;
}

static void q3_start(void)
{
    uint32_t now_ms = system_time_ms();

    ball_control_start();
    if (ball_control.snapshot.state == BALL_CONTROL_FAULT) {
        q3_set_terminal(APP_Q3_FAULT, APP_Q3_RESULT_FAULT, now_ms);
        return;
    }
    if (!app_state_set_ball_target_position_mm_x10(
            APP_Q3_PLUS_POSITION_MM_X10)) {
        q3_set_terminal(APP_Q3_FAULT, APP_Q3_RESULT_FAULT, now_ms);
        return;
    }
    ball_control.snapshot.q3_phase = APP_Q3_MOVE_PLUS;
    ball_control.snapshot.q3_elapsed_ms = 0U;
    ball_control.snapshot.q3_plus_reached_ms = 0U;
    ball_control.snapshot.q3_result = APP_Q3_RESULT_NONE;
    q3_test.started_ms = now_ms;
    q3_test.stable_since_ms = 0U;
}

static void q3_tick(void)
{
    uint32_t now_ms = system_time_ms();
    bool within_position_band;
    bool within_settle_entry_band;
    bool within_speed_band;

    ball_control.snapshot.q3_elapsed_ms =
        (uint32_t)(now_ms - q3_test.started_ms);
    if (ball_control.snapshot.state == BALL_CONTROL_FAULT) {
        q3_set_terminal(APP_Q3_FAULT, APP_Q3_RESULT_FAULT, now_ms);
        return;
    }
    if (ball_control.snapshot.q3_elapsed_ms >= APP_Q3_TIMEOUT_MS) {
        q3_set_terminal(APP_Q3_TIMEOUT, APP_Q3_RESULT_TIMEOUT, now_ms);
        return;
    }

    within_position_band =
        abs_i32(ball_control.snapshot.ball_error_mm_x10) <=
        APP_Q3_ARRIVAL_BAND_MM_X10;
    within_settle_entry_band =
        abs_i32(ball_control.snapshot.ball_error_mm_x10) <=
        APP_Q3_SETTLE_ENTRY_BAND_MM_X10;
    within_speed_band =
        abs_i32(ball_control.snapshot.filtered_velocity_mm_s_x10) <=
        APP_Q3_STABLE_SPEED_MM_S_X10;

    if (ball_control.snapshot.q3_phase == APP_Q3_MOVE_PLUS) {
        /* +5 cm is the turnaround point, not a hold point. Reverse as soon
           as the filtered position first enters its arrival band. */
        if (within_position_band) {
            if (!app_state_set_ball_target_position_mm_x10(
                    APP_Q3_MINUS_POSITION_MM_X10)) {
                q3_set_terminal(APP_Q3_FAULT, APP_Q3_RESULT_FAULT,
                    now_ms);
                return;
            }
            ball_control.snapshot.q3_plus_reached_ms =
                ball_control.snapshot.q3_elapsed_ms;
            ball_control.snapshot.q3_phase = APP_Q3_MOVE_MINUS;
            q3_test.stable_since_ms = 0U;
        }
        return;
    }

    if (ball_control.snapshot.q3_phase == APP_Q3_MOVE_MINUS) {
        if (within_settle_entry_band) {
            /* Enter the low-authority damping controller as soon as -5 cm
               is approached; the narrower pass band is checked below. */
            ball_control.snapshot.q3_phase = APP_Q3_SETTLE_MINUS;
            q3_test.stable_since_ms = 0U;
        }
        return;
    }

    if (ball_control.snapshot.q3_phase == APP_Q3_SETTLE_MINUS) {
        if (abs_i32(ball_control.snapshot.ball_error_mm_x10) >
            APP_Q3_SETTLE_EXIT_BAND_MM_X10) {
            /* A large overshoot left the capture region. Restore the faster
               approach controller until the ball is close enough to settle
               again, rather than holding a weak fixed tilt far from target. */
            ball_control.snapshot.q3_phase = APP_Q3_MOVE_MINUS;
            q3_test.stable_since_ms = 0U;
            return;
        }
        if (q3_condition_stable(within_position_band && within_speed_band,
                APP_Q3_MINUS_STABLE_MS, now_ms)) {
            q3_set_terminal(APP_Q3_DONE, APP_Q3_RESULT_PASS, now_ms);
        }
    }
}
#endif /* APP_Q3_RUNTIME_ENABLED */

static bool a_line_stop_distance_ready(void)
{
    int left_count;
    int right_count;
    int distance_count;

    chassis_encoder_get_counts(&left_count, &right_count);
    if (left_count < 0) left_count = -left_count;
    if (right_count < 0) right_count = -right_count;

    distance_count = (left_count + right_count) / 2;
    if (run_mode == RUN_MODE_PROBLEM56) {
        return distance_count >= P56_A_LINE_MIN_STOP_COUNTS;
    }

    return distance_count >= P2_A_LINE_MIN_STOP_COUNTS;
}

static int encoder_distance_count(void)
{
    int left_count;
    int right_count;

    chassis_encoder_get_counts(&left_count, &right_count);
    if (left_count < 0) left_count = -left_count;
    if (right_count < 0) right_count = -right_count;

    return (left_count + right_count) / 2;
}

static void enter_state(AppState_t new_state)
{
    AppState_t previous_state = state;

    state       = new_state;
    state_ticks = 0;
    track_bridge_mark_stale();

    /* The run timer follows the wheels: APP_STATE_RUNNING is the only state
       in which the chassis drives, so every entry starts the clock and every
       exit freezes it. Routing this through the single state-transition
       funnel covers all start paths and all stop paths (A-line stop, B-line
       stop, target reached, line lost, error, timeout, debug key) without
       needing a hook at each call site. The pendulum loop is released on the
       same edge, so a stopped vehicle never leaves STEP pulses running. */
    if (new_state != previous_state) {
        if (new_state == APP_STATE_RUNNING) {
            oled_stopwatch_start();
        } else if (previous_state == APP_STATE_RUNNING) {
            oled_stopwatch_stop();
            ball_control_stop();
        }
    }
}

#if APP_Q3_RUNTIME_ENABLED
static void start_problem3(void)
{
    mode_select_active = false;
    mode_press_count = 0U;
    mode_select_ticks = 0U;
    run_mode = RUN_MODE_PROBLEM3;
    q3_start_requested = true;

    /* Item 3 operates the pendulum only.  Lock the chassis immediately; if
       PA26 homing was skipped or has not finished, restart it and begin the
       O -> +5 cm -> -5 cm sequence automatically as soon as it completes. */
    chassis_lock();
    ball_control_stop();
    ball_control.snapshot.q3_phase = APP_Q3_IDLE;
    ball_control.snapshot.q3_result = APP_Q3_RESULT_NONE;
    if (!ball_control.snapshot.homed) {
        ball_control_begin_homing();
    }
}
#endif

static void start_line_problem(RunMode_t selected_mode)
{
    mode_select_active = false;
    mode_press_count = 0U;
    mode_select_ticks = 0U;

    /* The old vision_uart_clear() discarded a pending shape/target result.
       The ball protocol needs no equivalent: ball_control_start() below
       snapshots sample_count, so samples that arrived before this run are
       never mistaken for fresh ones. */
    a_line_count = 1U;
    a_line_confirm_ticks = P2_A_LINE_CONFIRM_TICKS;
    a_line_latched = true;
    p4_b_line_confirm_ticks = P4_B_LINE_CONFIRM_TICKS;
    p4_b_line_latched = true;
    run_mode = selected_mode;
    if (run_mode == RUN_MODE_PROBLEM4) {
        app_debug_send_str("P4_START\r\n");
    } else if (run_mode == RUN_MODE_PROBLEM56) {
        app_debug_send_str("P56_START\r\n");
    }
    target = TARGET_C;
    /* The chassis must remain usable before the pendulum has been wired or
       calibrated.  Only start the ball loop after a completed PA26 homing
       pass; otherwise leave STEP stopped and run the original line follower.
       Item 6 may override the centre target before a later, homed run. */
    if (ball_control.snapshot.homed) {
        (void)app_state_set_ball_target_position_mm_x10(
            BALL_TARGET_POSITION_MM_X10);
        ball_control_start();
    } else {
        ball_control_stop();
    }
    chassis_unlock();
    line_track_reset();
    chassis_follow_target(target);
    enter_state(APP_STATE_RUNNING);
}

void app_state_init(void)
{
    state          = APP_STATE_IDLE;
    run_mode       = RUN_MODE_BASIC_DRIVE;
    target         = TARGET_NONE;
    state_ticks    = 0;
    /* For problem 2 the car starts on the A wide line. Count that as the
       first A-line pass, then stop on the next confirmed A-line detection. */
    a_line_count = 1U;
    a_line_confirm_ticks = P2_A_LINE_CONFIRM_TICKS;
    a_line_latched = true;
    p4_b_line_confirm_ticks = P4_B_LINE_CONFIRM_TICKS;
    p4_b_line_latched = true;
    mode_select_active = false;
    mode_press_count = 0U;
    mode_select_ticks = 0U;
    ir_debug_ticks = 0U;
    rod_home_target_cdeg =
        (int32_t)APP_ROD_HOME_DEFAULT_ANGLE_CDEG;
    line_track_reset();
    track_bridge_init();
    /* Homing begins in IDLE, but the start key may explicitly bypass it for
       line-following-only tests. */
    ball_control_init();
    ball_control_begin_homing();
}

/* ---- per-state handlers ---- */

static void handle_idle(void)
{
    /* Rod homing runs here. Keys are swallowed only while homing is actually
       in progress, so a press during the power-on move cannot launch the car
       the instant homing completes.

       A homing FAILURE must not block the vehicle: items 2/4/5/6 are scored on
       driving and timing, and item 2 does not involve the pendulum at all. If
       the rod cannot home (PA26 missing, DIR reversed, mechanically stuck) the
       chassis still runs and ball_control_start() simply faults with
       NOT_HOMED, which stops STEP pulses and leaves the rod passive. */
    ball_control_home_tick();
    if (ball_control.snapshot.state == BALL_CONTROL_HOMING) {
        /* PA25 is always allowed to start the chassis.  Cancel the pending
           rod move, keep STEP stopped, and arm the normal PA13 mode menu.
           This avoids losing a valid operator start while PA26 feedback is
           absent or the 10 s homing timeout is still running. */
        if (start_key_triggered()) {
            ball_control_stop();
            mode_select_active = true;
            mode_press_count = 0U;
            mode_select_ticks = 0U;
            (void)mode_key_triggered();
        } else {
            (void)mode_key_triggered();
            return;
        }
    }

#if APP_Q3_RUNTIME_ENABLED
    if (run_mode == RUN_MODE_PROBLEM3) {
    /* Item 3 is selected by PA25 + four PA13 presses.  The chassis stays
       locked while the pendulum runs O -> +5 cm -> -5 cm. */
    chassis_lock();
    if (ball_control.snapshot.q3_phase == APP_Q3_IDLE) {
        q3_prepare_ready();
    }
    if (ball_control.snapshot.q3_phase == APP_Q3_READY) {
        if (q3_start_requested || start_key_triggered()) {
            q3_start_requested = false;
            q3_start();
            if (ball_control.snapshot.q3_phase == APP_Q3_MOVE_PLUS) {
                enter_state(APP_STATE_RUNNING);
            }
        }
        return;
    }
    if ((ball_control.snapshot.q3_phase == APP_Q3_DONE) ||
        (ball_control.snapshot.q3_phase == APP_Q3_TIMEOUT) ||
        (ball_control.snapshot.q3_phase == APP_Q3_ABORTED) ||
        (ball_control.snapshot.q3_phase == APP_Q3_FAULT)) {
        /* APP_Q3_FAULT must be resettable here too: without it, a control-
           quality fault (e.g. BALL_CONTROL_FAULT_DIRECTION) leaves the rod
           frozen at whatever angle it held when the fault fired, and no
           start-key press could recover it short of a power cycle. The next
           q3_prepare_ready() clears the underlying BALL_CONTROL_FAULT via
           ball_control_stop(); it does not clear snapshot.fault_reason, which
           is diagnostic-only and kept until the next fault or homing. */
        if (start_key_triggered()) {
            ball_control.snapshot.q3_phase = APP_Q3_IDLE;
            ball_control.snapshot.q3_result = APP_Q3_RESULT_NONE;
            ball_control.snapshot.q3_elapsed_ms = 0U;
            ball_control.snapshot.q3_plus_reached_ms = 0U;
            q3_start_requested = false;
            run_mode = RUN_MODE_BASIC_DRIVE;
        }
        return;
    }
    (void)start_key_triggered();
    return;
    }
#endif
#if APP_ROD_HOMING_ONLY
    /* Bench mode: stay idle after the rod reaches its configured PA26 angle. */
    (void)start_key_triggered();
    (void)mode_key_triggered();
    return;
#endif

    if (!mode_select_active) {
        if (start_key_triggered()) {
            mode_select_active = true;
            mode_press_count = 0U;
            mode_select_ticks = 0U;
        }
        return;
    }

    if (mode_key_triggered()) {
        mode_press_count++;
        mode_select_ticks = 0U;
        if (mode_press_count >= 4U) {
            start_problem3();
        }
        return;
    }

    if (mode_press_count >= 1U) {
        mode_select_ticks++;
        if (mode_select_ticks >= MODE_SELECT_CONFIRM_TICKS) {
            if (mode_press_count >= 3U) {
                start_line_problem(RUN_MODE_PROBLEM56);
            } else if (mode_press_count >= 2U) {
                start_line_problem(RUN_MODE_PROBLEM4);
            } else {
                start_line_problem(RUN_MODE_BASIC_DRIVE);
            }
        }
    }
}

/* handle_fixed_detect() and handle_gimbal_scan() lived here. Both consumed the
   old shape/target vision protocol (VisionResult_t), which UART1 no longer
   carries -- it now streams ball-position telemetry for the pendulum loop.
   Neither was reachable: nothing ever called enter_state() with
   APP_STATE_FIXED_DETECT or APP_STATE_GIMBAL_SCAN once BASIC_NAV_BYPASS_VISION
   made start_line_problem() the sole entry into APP_STATE_RUNNING. */

static void handle_start_beep(void)
{
    chassis_unlock();
    line_track_reset();
    chassis_follow_target(target);
    enter_state(APP_STATE_RUNNING);
}

static void handle_running(void)
{
#if APP_Q3_RUNTIME_ENABLED
    if (run_mode == RUN_MODE_PROBLEM3) {
    /* Item 3 never drives the chassis, so bypass every line-tracking path. */
    chassis_lock();
    if (start_key_triggered()) {
        if ((ball_control.snapshot.q3_phase == APP_Q3_DONE) ||
            (ball_control.snapshot.q3_phase == APP_Q3_TIMEOUT)) {
            enter_state(APP_STATE_IDLE);
            return;
        }
        q3_set_terminal(APP_Q3_ABORTED, APP_Q3_RESULT_ABORTED,
            system_time_ms());
        enter_state(APP_STATE_IDLE);
        return;
    }
    if ((ball_control.snapshot.q3_phase == APP_Q3_DONE) ||
        (ball_control.snapshot.q3_phase == APP_Q3_TIMEOUT)) {
        /* Keep regulating around -5 cm after success or timeout: the result
           and elapsed time stay frozen, but the motor remains closed-loop. */
        ball_control_tick();
        if (ball_control.snapshot.state == BALL_CONTROL_FAULT) {
            q3_set_terminal(APP_Q3_FAULT, APP_Q3_RESULT_FAULT,
                system_time_ms());
        }
        return;
    }
    ball_control_tick();
    q3_tick();
    if (ball_control.snapshot.q3_phase == APP_Q3_FAULT) {
        enter_state(APP_STATE_IDLE);
    }
    return;
    }
#endif
#if P2_DEBUG_STOP_ON_START_KEY
    if (start_key_triggered()) {
        chassis_lock();
        enter_state(APP_STATE_IDLE);
        return;
    }
#endif

    /* The line follower remains independent of unfinished/absent pendulum
       hardware.  A successfully homed rod still runs its closed loop first. */
    if (ball_control.snapshot.homed) {
        ball_control_tick();
    }

    /* Update tracking bridge before chassis tick so Hardware A can consume
       fresh line_track data in the same tick if needed. */
    {
        uint8_t gv[GRAY_SENSOR_CHANNELS];
        LineTrackState_t lt;
        uint8_t a_line_center_count;
        uint8_t a_line_center_required;
        uint8_t a_line_release_channels;
        uint8_t a_line_confirm_required;
        gray_sensor_read_all(gv);

        /* Apply turn sensor mask to suppress opposite-side interference.
           Mask is set by chassis on turn start; cleared by chassis on exit. */
        {
            int8_t mask = track_bridge_get_turn_mask();
            if (mask > 0) {
                gv[0] = 0;  /* mask X1 */
                gv[1] = 0;  /* mask X2 */
            } else if (mask < 0) {
                gv[4] = 0;  /* mask X5 */
                gv[5] = 0;  /* mask X6 */
            }
        }

        line_track_compute(gv, &lt);
        track_bridge_update(&lt, state_ticks);

        ir_debug_ticks++;
        if (ir_debug_ticks >= IR_DEBUG_INTERVAL_TICKS) {
            ir_debug_ticks = 0U;
            ir_debug_report(gv);
        }

        a_line_center_count = 0U;
        if (gv[1] == GRAY_ACTIVE_LEVEL) a_line_center_count++;
        if (gv[2] == GRAY_ACTIVE_LEVEL) a_line_center_count++;
        if (gv[3] == GRAY_ACTIVE_LEVEL) a_line_center_count++;
        if (gv[4] == GRAY_ACTIVE_LEVEL) a_line_center_count++;

#if !P2_PURE_LINE_TRACK_TEST
        if (run_mode == RUN_MODE_BASIC_DRIVE ||
            run_mode == RUN_MODE_PROBLEM56) {
        a_line_center_required =
            (run_mode == RUN_MODE_PROBLEM56) ?
            P56_A_LINE_CENTER_CHANNELS : P2_A_LINE_CENTER_CHANNELS;
        a_line_release_channels =
            (run_mode == RUN_MODE_PROBLEM56) ?
            P56_A_LINE_RELEASE_CHANNELS : P2_A_LINE_RELEASE_CHANNELS;
        a_line_confirm_required =
            (run_mode == RUN_MODE_PROBLEM56) ?
            P56_A_LINE_CONFIRM_TICKS : P2_A_LINE_CONFIRM_TICKS;

        if (a_line_center_count >= a_line_center_required) {
            if (a_line_confirm_ticks < a_line_confirm_required) {
                a_line_confirm_ticks++;
            }
            if (a_line_confirm_ticks >= a_line_confirm_required &&
                !a_line_latched) {
                a_line_latched = true;
                a_line_count++;
                if (a_line_count >= 2U) {
                    if (a_line_stop_distance_ready()) {
                        /* 要求 2: second confirmed A-line pass ends the lap.
                           enter_state() freezes the run timer. */
                        chassis_brake();
                        enter_state(APP_STATE_TARGET_STOP);
                        return;
                    }
                    a_line_count = 1U;
                }
            }
        } else if (lt.active_count <= a_line_release_channels) {
            a_line_confirm_ticks = 0U;
            a_line_latched = false;
        }
        }
#endif
        if (run_mode == RUN_MODE_PROBLEM4) {
            if (lt.active_count >= P4_B_LINE_ACTIVE_CHANNELS) {
                if (p4_b_line_confirm_ticks < P4_B_LINE_CONFIRM_TICKS) {
                    p4_b_line_confirm_ticks++;
                }
                if (p4_b_line_confirm_ticks >= P4_B_LINE_CONFIRM_TICKS &&
                    !p4_b_line_latched) {
                    p4_report_encoder_at_b();
                    chassis_brake();
                    enter_state(APP_STATE_TARGET_STOP);
                    return;
                }
            } else if (lt.active_count <= P4_B_LINE_RELEASE_CHANNELS) {
                p4_b_line_confirm_ticks = 0U;
                p4_b_line_latched = false;
            }
        }
    }

    if (!(run_mode == RUN_MODE_PROBLEM4 && P4_MANUAL_PUSH_MEASURE)) {
        chassis_tick();
    }
    state_ticks++;

    if (run_mode == RUN_MODE_PROBLEM4 &&
        !P4_MANUAL_PUSH_MEASURE &&
        encoder_distance_count() >= P4_AB_TARGET_COUNTS) {
        p4_report_encoder_at_b();
        chassis_brake();
        enter_state(APP_STATE_TARGET_STOP);
        return;
    }

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

#if !P2_DISABLE_RUN_TIMEOUT
    if (state_ticks >= RUN_TIMEOUT_TICKS) {
        chassis_stop();
        chassis_lock();
        enter_state(APP_STATE_ERROR);
    }
#endif
}

static void handle_target_stop(void)
{
    chassis_lock();
    enter_state(APP_STATE_IDLE);
}

static void handle_finish_beep(void)
{
    enter_state(APP_STATE_IDLE);
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
        /* FIXED_DETECT and GIMBAL_SCAN are gone with the old vision protocol;
           an unexpected value now falls through to the ERROR default. */
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

BallControlSnapshot_t app_state_get_ball_control_snapshot(void)
{
    return ball_control.snapshot;
}

/* Item 6: the ball may be told to hold any specified rod position. Resetting
   the target angle to neutral avoids carrying a tilt computed for the old
   target into the first sample against the new one. */
bool app_state_set_ball_target_position_mm_x10(int32_t position_mm_x10)
{
    if ((position_mm_x10 < BALL_POSITION_MIN_MM_X10) ||
        (position_mm_x10 > BALL_POSITION_MAX_MM_X10)) {
        return false;
    }
    ball_control.snapshot.target_position_mm_x10 = position_mm_x10;
    ball_control.snapshot.target_angle_cdeg = ROD_NEUTRAL_ANGLE_CDEG;
    ball_control.snapshot.ball_error_mm_x10 =
        ball_control.snapshot.filtered_position_mm_x10 - position_mm_x10;
    ball_control_reset_friction();
    return true;
}

int32_t app_state_get_ball_target_position_mm_x10(void)
{
    return ball_control.snapshot.target_position_mm_x10;
}

bool app_state_set_rod_home_angle_cdeg(uint16_t angle_cdeg)
{
    if ((state != APP_STATE_IDLE) ||
        (angle_cdeg < APP_ROD_HOME_MIN_ANGLE_CDEG) ||
        (angle_cdeg > APP_ROD_HOME_MAX_ANGLE_CDEG)) {
        return false;
    }
    rod_home_target_cdeg = (int32_t)angle_cdeg;
    ball_control_begin_homing();
    return true;
}

uint16_t app_state_get_rod_home_angle_cdeg(void)
{
    return (uint16_t)rod_home_target_cdeg;
}

bool app_state_restart_rod_homing(void)
{
    if (state != APP_STATE_IDLE) {
        return false;
    }
    ball_control_begin_homing();
    return true;
}
