## ADDED Requirements

### Requirement: Eight-Channel Multiplexed Grayscale Sensor Support

The system SHALL support one 8-channel grayscale line sensor module that uses three address-select pins and one shared digital output pin.

#### Scenario: Select and read all channels
- **WHEN** the main controller performs one grayscale scan cycle
- **THEN** it SHALL drive `AD0/AD1/AD2` through the binary combinations `000` to `111`
- **AND** it SHALL read one digital `OUT` value for each selected channel
- **AND** it SHALL return one ordered 8-element result representing `X1` through `X8`

#### Scenario: Bounded scan timing
- **WHEN** the system scans all 8 grayscale channels
- **THEN** the implementation SHALL use a bounded per-channel settle delay
- **AND** the total scan SHALL remain suitable for the existing MSPM0 main-loop cadence

### Requirement: Configurable Digital Sensor Semantics

The system SHALL support configurable active-level interpretation for the grayscale module so the same driver can be adapted to the actual module behavior and track materials.

#### Scenario: Red-light module active level
- **WHEN** the red-light grayscale module is installed and tested on the target track
- **THEN** the firmware SHALL allow the active level to be configured without rewriting the whole driver
- **AND** upper-layer code SHALL receive consistent line-detection semantics after configuration

#### Scenario: Channel state normalization
- **WHEN** one grayscale scan completes
- **THEN** upper layers SHALL be able to consume normalized per-channel digital states
- **AND** the module SHALL hide raw GPIO addressing details from the caller

### Requirement: Grayscale Sensor Observability

The system SHALL provide debug visibility for grayscale sensor bring-up, installation-height validation, and probe troubleshooting.

#### Scenario: DEBUG_UART inspection
- **WHEN** the operator enables or observes grayscale bring-up output over DEBUG_UART
- **THEN** the firmware SHALL expose the state of all 8 channels in a form that can be correlated with `X1` through `X8`

#### Scenario: Installation-height validation
- **WHEN** the red-light module is mounted near the recommended 18mm probe-to-ground height
- **THEN** the debug output SHALL make it possible to verify whether each channel responds to black and white targets

### Requirement: Non-Intrusive Integration With Existing Main Control

The system SHALL integrate the grayscale module without changing existing vision protocol semantics or chassis contract boundaries in this change.

#### Scenario: Existing main-control behavior remains stable
- **WHEN** the grayscale sensor module is added
- **THEN** the system SHALL keep the current vision UART protocol unchanged
- **AND** it SHALL keep the existing `chassis_iface.h` external contract unchanged
- **AND** it SHALL not require changes to the current NAV or ADV mode semantics in order to compile or run
