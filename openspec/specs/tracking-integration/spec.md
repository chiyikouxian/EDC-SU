# tracking-integration Specification

## Purpose
TBD - created by archiving change add-line-tracking-integration-prep. Update Purpose after archive.
## Requirements
### Requirement: Tracking Data Output Contract

The system SHALL provide a stable tracking-data contract that exposes the line_track computation results in a chassis-consumable form, without coupling to any specific motor or PID implementation.

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

- **WHEN** `handle_running()` executes each 10ms tick
- **THEN** `track_bridge_update()` SHALL be called before `chassis_tick()`
- **AND** no other state handlers (IDLE, FDET, GSCAN, SBEEP, TSTOP, FBEEP, ERROR) SHALL call `track_bridge_update()`

#### Scenario: Tick field is logical freshness metadata

- **WHEN** `TrackBridgeData_t.tick` is exposed to downstream code
- **THEN** it SHALL represent the main-control logical tick count associated with the latest bridge update
- **AND** downstream code SHALL NOT assume that the tick value permanently maps to a fixed physical duration such as 10ms unless separately documented
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

