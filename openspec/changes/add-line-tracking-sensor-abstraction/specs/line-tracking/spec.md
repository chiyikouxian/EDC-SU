## ADDED Requirements

### Requirement: Normalized Tracking Input From Eight Grayscale Channels

The system SHALL convert the 8-channel grayscale sensor states into a normalized tracking abstraction that upper layers can consume without knowing the raw GPIO channel-selection details.

#### Scenario: Consume one grayscale frame
- **WHEN** one ordered `X1` through `X8` grayscale state frame is available
- **THEN** the tracking abstraction SHALL accept that frame as input
- **AND** it SHALL preserve the convention that `X1` is leftmost and `X8` is rightmost

#### Scenario: Keep raw sensor driver separate
- **WHEN** upper layers use the tracking abstraction
- **THEN** they SHALL NOT need to manipulate `AD0/AD1/AD2 + OUT` addressing details directly

### Requirement: Signed Tracking Error

The system SHALL compute one bounded signed tracking error from the grayscale input frame for later steering or PID layers.

#### Scenario: Line moves from left to right
- **WHEN** the detected black line moves physically from probes near `X1` toward probes near `X8`
- **THEN** the reported tracking error SHALL change monotonically in the documented direction
- **AND** the sign convention SHALL be documented in the firmware interface or README

#### Scenario: Center alignment
- **WHEN** the line is primarily under the middle probes
- **THEN** the reported tracking error SHALL be near zero

### Requirement: High-Level Line State

The system SHALL expose high-level line-state semantics in addition to raw error output.

#### Scenario: Any probe sees the line
- **WHEN** one or more probes match the configured active line level
- **THEN** the abstraction SHALL report that a line is detected

#### Scenario: All probes miss the line
- **WHEN** all probes report the non-active level
- **THEN** the abstraction SHALL report an all-white or possible line-lost condition at sensor-abstraction level

#### Scenario: Middle probes see the line
- **WHEN** the line is detected primarily by the center probes
- **THEN** the abstraction SHALL expose a center-hit or center-aligned state

### Requirement: Non-Intrusive Integration With Existing Main Control

The tracking abstraction SHALL be added without changing the current chassis contract or vision-driven state-machine semantics in this change.

#### Scenario: Existing interfaces remain stable
- **WHEN** the tracking abstraction module is added
- **THEN** the system SHALL keep `chassis_iface.h` unchanged
- **AND** it SHALL keep the current vision UART protocol unchanged
- **AND** it SHALL not require semantic changes to the current DRV, NAV, or ADV state-machine flows in order to compile
