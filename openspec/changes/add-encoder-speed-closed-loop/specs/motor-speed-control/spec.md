## ADDED Requirements

### Requirement: Encoder Speed Closed Loop

The chassis SHALL use independent left and right encoder feedback controllers to regulate wheel speed when encoder speed closed-loop control is enabled.

#### Scenario: Closed-loop command is applied

- **WHEN** the chassis receives non-zero left and right wheel-speed targets
- **THEN** the firmware SHALL measure encoder-count increments over a fixed control period
- **AND** it SHALL calculate independent left and right PID corrections
- **AND** it SHALL limit the final motor commands to the supported PWM range

#### Scenario: Controller is stopped safely

- **WHEN** the chassis is stopped, braked, locked, or given a zero-speed target
- **THEN** the corresponding speed-controller integral and previous-error state SHALL be reset
- **AND** no stale PID correction SHALL drive the motor

### Requirement: Speed Loop Tuning and Fallback

The firmware SHALL expose compile-time tuning parameters and a compile-time fallback to direct open-loop motor commands.

#### Scenario: Parameters are calibrated

- **WHEN** the vehicle is prepared for hardware tuning
- **THEN** target-count scaling, PID gains, integral limits, and correction limits SHALL be configurable without changing control-flow code

#### Scenario: Closed loop is disabled

- **WHEN** encoder speed closed-loop control is disabled at compile time
- **THEN** chassis wheel commands SHALL use the existing direct open-loop motor path
