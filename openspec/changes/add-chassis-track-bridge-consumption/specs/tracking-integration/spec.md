## MODIFIED Requirements

### Requirement: Tracking Data Output Contract

The system SHALL provide a stable tracking-data contract that exposes the line_track computation results in a chassis-consumable form, without coupling to any specific motor or PID implementation. The chassis SHALL consume bridge data via `track_bridge_get()` as its primary input path in production builds.

#### Scenario: Bridge data available during running

- **WHEN** the main state machine is in `APP_STATE_RUNNING`
- **THEN** `track_bridge_update()` SHALL refresh the bridge data from the latest `LineTrackState_t`
- **AND** the bridge data SHALL include at minimum: signed error, line-detected flag, all-white flag, center-hit flag, active probe count, and a freshness indicator

#### Scenario: Bridge data stale outside running

- **WHEN** the main state machine is NOT in `APP_STATE_RUNNING`
- **THEN** the bridge data SHALL be marked as stale
- **AND** `track_bridge_get()` SHALL still return the last-known data with the stale flag set
- **AND** the returned data outside `APP_STATE_RUNNING` SHALL be treated as non-real-time reference data only

#### Scenario: Zero-copy non-blocking read

- **WHEN** the chassis layer or debug code calls `track_bridge_get()`
- **THEN** the function SHALL return a const pointer to an internally maintained struct
- **AND** it SHALL NOT allocate memory, block, or copy the struct

### Requirement: Bridge Update Integration Without State-Machine Changes

The system SHALL integrate the bridge update into the existing main loop without modifying the state-machine transition logic or the `app_state.h` / `chassis_iface.h` external interfaces.

#### Scenario: Update called from handle_running

- **WHEN** `handle_running()` executes each app/control tick (`TICK_MS`, currently 10ms)
- **THEN** `track_bridge_update()` SHALL be called before `chassis_tick()`
- **AND** no other state handlers (IDLE, FDET, GSCAN, SBEEP, TSTOP, FBEEP, ERROR) SHALL call `track_bridge_update()`

#### Scenario: Tick field is logical freshness metadata

- **WHEN** `TrackBridgeData_t.tick` is exposed to downstream code
- **THEN** it SHALL represent the main-control logical tick count associated with the latest bridge update
- **AND** downstream code SHALL NOT assume that the tick value permanently maps to a fixed physical duration unless separately documented
- **AND** downstream freshness checks SHALL use the `stale` flag as the primary validity indicator

#### Scenario: Existing interfaces unchanged

- **WHEN** the bridge module is added
- **THEN** `app_state.h` SHALL NOT require new includes, types, or function declarations
- **AND** `chassis_iface.h` SHALL remain unchanged
- **AND** the vision UART protocol SHALL remain unchanged

#### Scenario: DRV/NAV/ADV regression

- **WHEN** the bridge module is compiled and linked
- **THEN** DRV find-line dry-run SHALL complete as before
- **AND** NAV fixed-detect + follow-target dry-run SHALL complete as before
- **AND** ADV gimbal-scan + follow-target dry-run SHALL complete as before

### Requirement: Bridge Debug Observability

The system SHALL expose the bridge data via DEBUG_UART so that the tracking-to-chassis data path can be verified on the bench independently of real motor hardware.

#### Scenario: Bridge field in debug output

- **WHEN** the main loop outputs the periodic DEBUG_UART status line
- **THEN** a `B=` field SHALL be appended after the existing `L=` field
- **AND** the `B=` value SHALL include at minimum the error value sign and magnitude, and a state indicator consistent with `line_detected`/`all_white`

#### Scenario: Consistency with L= field

- **WHEN** both `L=` and `B=` fields are present in the same debug line
- **THEN** the error value in `B=` SHALL equal the error value in `L=`
- **AND** the line-state indicator in `B=` SHALL be consistent with the `L=` state indicator

#### Scenario: Stale indicator in debug

- **WHEN** the system is IDLE and the periodic debug line outputs `B=`
- **THEN** the bridge debug string SHALL include a recognizable stale marker distinguishable from valid running data

### Requirement: Hardware Collaboration Boundary Documentation

The system SHALL document the tracking-data boundary between Hardware B (main control) and Hardware A (chassis) so that both sides can develop and test independently.

#### Scenario: README documents the bridge contract

- **WHEN** a developer reads the README
- **THEN** there SHALL be a "循迹桥接" section that lists every field in `TrackBridgeData_t` with its semantics, range, and validity conditions

#### Scenario: README states what main control does NOT provide

- **WHEN** Hardware A reads the bridge documentation
- **THEN** it SHALL be clear that the main control provides tracking error and line state only
- **AND** it SHALL be clear that motor PWM, differential targets, PID parameters, intersection strategy, and path action tables are Hardware A responsibilities

## ADDED Requirements

### Requirement: Chassis Primary Input via Track Bridge

The chassis SHALL consume `track_bridge_get()` as its primary tracking input path in production builds, with a compile-time switch to fall back to a local sensor read for standalone debugging.

#### Scenario: Production build uses bridge

- **WHEN** `CHASSIS_USE_TRACK_BRIDGE` is defined as `1`
- **THEN** the chassis SHALL obtain tracking data exclusively via `track_bridge_get()`
- **AND** it SHALL NOT call any local sensor read function such as `track_read()` for real-time control

#### Scenario: Standalone debug build uses fallback

- **WHEN** `CHASSIS_USE_TRACK_BRIDGE` is defined as `0`
- **THEN** the chassis SHALL fall back to its local `track_read()` for tracking input
- **AND** the chassis SHALL still compile and run independently of `track_bridge.h/.c`

### Requirement: Error Scaling Parameterization

The chassis SHALL apply a configurable integer scale factor to the bridge error value before feeding it into PID control, so that the same bridge contract `[-100, +100]` can drive PID loops tuned to different error ranges.

#### Scenario: Current default scale maps bridge range directly

- **WHEN** `CHASSIS_TRACK_ERROR_SCALE` is defined (current default 1)
- **THEN** `chassis_line_input.error` SHALL equal `bridge->error * CHASSIS_TRACK_ERROR_SCALE`
- **AND** the bridge contract SHALL remain `[-100, +100]` regardless of the scale value

#### Scenario: Scale is a chassis-side concern

- **WHEN** the scale factor is adjusted during vehicle tuning
- **THEN** `track_bridge.h` and `TrackBridgeData_t` SHALL NOT require modification
- **AND** the main control SHALL continue to output `[-100, +100]` error unchanged

### Requirement: Stale Protection with Unified Brake

The chassis SHALL immediately brake when bridge data is marked stale, without mode-specific behavior or stale-data PID continuation.

#### Scenario: Brake on stale

- **WHEN** `TrackBridgeData_t.stale` is `true`
- **THEN** the chassis SHALL execute an immediate brake (motor outputs set to safe state)
- **AND** it SHALL NOT feed the stale error value into PID
- **AND** this behavior SHALL be identical across DRV, NAV, and ADV modes

#### Scenario: Recovery from stale

- **WHEN** `stale` transitions from `true` to `false` after entering `APP_STATE_RUNNING`
- **THEN** the chassis SHALL resume normal PID tracking on the next tick

### Requirement: Lost-Line Handling with Directional Find-Line

The chassis SHALL distinguish short-duration and long-duration line-loss events, and SHALL use the last valid error sign to determine the find-line rotation direction.

#### Scenario: Short loss triggers find-line

- **WHEN** `line_detected == false` or `all_white == true` persists for fewer than `CHASSIS_LOST_LINE_SHORT_TICKS` ticks
- **THEN** the chassis SHALL enter a find-line rotation
- **AND** the rotation direction SHALL be right (clockwise) if `last_line_error >= 0`, left (counter-clockwise) if `last_line_error < 0`

#### Scenario: Long loss triggers brake

- **WHEN** `line_detected == false` or `all_white == true` persists for `CHASSIS_LOST_LINE_LONG_TICKS` or more ticks
- **THEN** the chassis SHALL execute a brake protection stop

#### Scenario: Recovery from line loss

- **WHEN** the line is re-detected during find-line rotation
- **THEN** the chassis SHALL exit the find-line state and resume normal PID tracking

### Requirement: Node Detection via Active Probe Count

The chassis SHALL use the active probe count to identify intersection or node regions, with a configurable threshold.

#### Scenario: Node region detected

- **WHEN** `TrackBridgeData_t.active_count >= CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD` (current default 6)
- **THEN** the chassis SHALL recognize the current region as a node or intersection
- **AND** the threshold SHALL be configurable via the `CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD` macro

#### Scenario: Below threshold is normal line

- **WHEN** `active_count < CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD`
- **THEN** the chassis SHALL treat the region as a normal line segment (not a node)

### Requirement: First-Round Acceptance Without Encoder Closed Loop

The first integration round SHALL accept the vehicle tracking baseline without requiring encoder-based speed closed-loop control.

#### Scenario: Encoder loop disabled by default

- **WHEN** the chassis code is first integrated into the main-control project
- **THEN** `CHASSIS_USE_ENCODER` SHALL default to `0`
- **AND** the chassis SHALL operate in open-loop speed mode

#### Scenario: Acceptance criteria for first round

- **WHEN** the first integration round is evaluated
- **THEN** the acceptance SHALL require: `track_bridge_get()` → error PID → differential steering → line following functional
- **AND** encoder closed-loop performance SHALL NOT be part of the acceptance criteria
