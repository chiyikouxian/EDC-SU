#ifndef APP_STATE_H
#define APP_STATE_H

#include "app_common.h"

/* Rod absolute-angle calibration, in centidegrees. The measured horizontal
   position is 237.80 deg. Keep the power-on target equal to the ball-control
   neutral angle (ROD_NEUTRAL_ANGLE_CDEG in app_state.c) so entering visual
   control does not introduce a false zero-angle offset. */
#define APP_ROD_HOME_DEFAULT_ANGLE_CDEG  23780U
#define APP_ROD_HOME_MIN_ANGLE_CDEG      20820U
#define APP_ROD_HOME_MAX_ANGLE_CDEG      26290U

/* Bench tuning mode: 1 = only perform PA26 power-on homing, 0 = enable the
   normal vehicle and vision-control start path. */
#define APP_ROD_HOMING_ONLY              0U

/* Competition item-3 control is selected at runtime with PA25 + four PA13
   presses. The legacy compile-time switch is retained for compatibility. */
#define APP_ROD_Q3_TEST_MODE             0U
#define APP_Q3_RUNTIME_ENABLED            1U
#define APP_Q3_CENTER_POSITION_MM_X10    1270L
#define APP_Q3_PLUS_POSITION_MM_X10      1770L
#define APP_Q3_MINUS_POSITION_MM_X10      770L
#define APP_Q3_ARRIVAL_BAND_MM_X10         80L
#define APP_Q3_SETTLE_ENTRY_BAND_MM_X10   300L
#define APP_Q3_SETTLE_EXIT_BAND_MM_X10    450L
#define APP_Q3_STABLE_SPEED_MM_S_X10      300L
#define APP_Q3_MINUS_STABLE_MS             500U
#define APP_Q3_TIMEOUT_MS                 5000U

/* Friction compensation tuning. A stationary ball first receives the static
   feedforward. If it remains within the stuck-position band for the confirm
   time while outside the arrival deadband, the bounded boost is added. */
#define APP_BALL_POSITION_DEADBAND_MM_X10            50L
#define APP_BALL_STATIC_SPEED_THRESHOLD_MM_S_X10     50L
#define APP_BALL_MOTION_SPEED_THRESHOLD_MM_S_X10     80L
#define APP_BALL_STATIC_FRICTION_FF_CDEG              60L
#define APP_BALL_KINETIC_FRICTION_FF_CDEG              0L
/* The 1 mm band originally here was tighter than realistic vision jitter, so
   the reference kept resetting and the 1 s confirm timer rarely completed;
   widened so a genuinely stuck ball is still detected without every noisy
   sample restarting the clock. Boost raised so the break-free kick is
   distinguishable once it does fire. */
#define APP_BALL_STUCK_POSITION_BAND_MM_X10           25L
#define APP_BALL_STUCK_CONFIRM_MS                     600U
/* Stuck escalation must only fire near the target: a real friction stick is
   "close to target but the last bit of travel will not close," which is
   exactly what the escalating boost is for. Without this gate, a brief
   velocity/position lull at the turnaround point of a large-amplitude
   oscillation (still tens of mm from target) looks identical to a genuine
   stick, and boosting there just re-launches the ball outward -- producing
   a self-sustaining oscillation instead of ever settling. */
#define APP_BALL_STUCK_NEAR_TARGET_BAND_MM_X10       150L
/* A single fixed boost sometimes was not enough to break the ball free (the
   rod would settle at the boosted angle and just sit there indefinitely).
   The boost now escalates the longer the ball stays stuck, capped at
   APP_BALL_STUCK_FF_BOOST_MAX_CDEG so it cannot run away toward the hard
   travel guards. */
#define APP_BALL_STUCK_FF_BOOST_CDEG                  80L
#define APP_BALL_STUCK_FF_BOOST_STEP_CDEG              40L
#define APP_BALL_STUCK_FF_BOOST_STEP_MS               500U
#define APP_BALL_STUCK_FF_BOOST_MAX_CDEG             390L

typedef enum {
    BALL_FRICTION_STATIONARY = 0,
    BALL_FRICTION_MOVING,
    BALL_FRICTION_STUCK
} BallFrictionState_t;

typedef enum {
    BALL_CONTROL_OFF = 0,
    BALL_CONTROL_WAITING,
    BALL_CONTROL_ACTIVE,
    BALL_CONTROL_HOLD,
    BALL_CONTROL_FAULT,
    BALL_CONTROL_HOMING
} BallControlState_t;

typedef enum {
    BALL_CONTROL_FAULT_NONE = 0,
    BALL_CONTROL_FAULT_HOME_FEEDBACK_TIMEOUT,
    BALL_CONTROL_FAULT_HOME_MOTION_TIMEOUT,
    BALL_CONTROL_FAULT_HARD_LIMIT,
    BALL_CONTROL_FAULT_DIRECTION,
    BALL_CONTROL_FAULT_COMMAND,
    BALL_CONTROL_FAULT_NOT_HOMED
} BallControlFault_t;

typedef enum {
    APP_Q3_IDLE = 0,
    APP_Q3_READY,
    APP_Q3_MOVE_PLUS,
    APP_Q3_MOVE_MINUS,
    APP_Q3_SETTLE_MINUS,
    APP_Q3_DONE,
    APP_Q3_TIMEOUT,
    APP_Q3_ABORTED,
    APP_Q3_FAULT
} AppQ3Phase_t;

typedef enum {
    APP_Q3_RESULT_NONE = 0,
    APP_Q3_RESULT_PASS,
    APP_Q3_RESULT_TIMEOUT,
    APP_Q3_RESULT_ABORTED,
    APP_Q3_RESULT_FAULT
} AppQ3Result_t;

typedef struct {
    BallControlState_t state;
    int32_t target_position_mm_x10;
    int32_t filtered_position_mm_x10;
    int32_t ball_error_mm_x10;
    int32_t target_angle_cdeg;
    int32_t rod_error_cdeg;
    int32_t step_frequency_hz;
    int32_t filtered_velocity_mm_s_x10;
    int32_t friction_feedforward_cdeg;
    uint32_t rejected_samples;
    uint8_t valid_streak;
    uint8_t fault_reason;
    uint8_t friction_state;
    bool homed;
    AppQ3Phase_t q3_phase;
    uint32_t q3_elapsed_ms;
    uint32_t q3_plus_reached_ms;
    AppQ3Result_t q3_result;
} BallControlSnapshot_t;

void app_state_init(void);
void app_state_tick(void);
AppState_t app_state_get(void);
RunMode_t  app_state_get_mode(void);
BallControlSnapshot_t app_state_get_ball_control_snapshot(void);
bool app_state_set_ball_target_position_mm_x10(int32_t position_mm_x10);
int32_t app_state_get_ball_target_position_mm_x10(void);
bool app_state_set_rod_home_angle_cdeg(uint16_t angle_cdeg);
uint16_t app_state_get_rod_home_angle_cdeg(void);
bool app_state_restart_rod_homing(void);

#endif /* APP_STATE_H */
