## ADDED Requirements

### Requirement: Run Mode Selection

The main control system SHALL support selecting among basic drive mode, basic navigation mode, and advanced navigation mode before a run starts, using one mode button that cycles modes.

#### Scenario: Default mode after power-on
- **WHEN** the system powers on
- **THEN** the selected run mode SHALL default to basic drive mode

#### Scenario: Select basic drive mode
- **WHEN** the selected mode cycles to basic drive mode while the system is idle
- **THEN** the system SHALL prepare the A-to-C drive flow without requiring a vision result

#### Scenario: Select basic navigation mode
- **WHEN** the selected mode cycles to basic navigation mode while the system is idle
- **THEN** the system SHALL prepare fixed-view vision recognition before allowing the chassis to move

#### Scenario: Select advanced navigation mode
- **WHEN** the selected mode cycles to advanced navigation mode while the system is idle
- **THEN** the system SHALL lock the chassis and prepare gimbal-based scanning before allowing the chassis to move

#### Scenario: Mode button cycles modes
- **WHEN** the operator presses the mode button while the system is idle
- **THEN** the system SHALL change the selected mode in the sequence basic drive, basic navigation, advanced navigation, then back to basic drive

#### Scenario: Mode key hardware binding
- **WHEN** the MSPM0 reads the mode key hardware input
- **THEN** it SHALL use PA27 configured as a pull-up GPIO input
- **AND** a low level on PA27 SHALL be interpreted as pressed

### Requirement: Long-Press Start Input

The main control system SHALL use a dedicated start key that only requests a run start after a 3 second long press.

#### Scenario: Start key hardware binding
- **WHEN** the MSPM0 reads the start key hardware input
- **THEN** it SHALL use PA25 configured as a pull-up GPIO input
- **AND** a low level on PA25 SHALL be interpreted as pressed

#### Scenario: Long press starts run
- **WHEN** the start key remains pressed for at least 3 seconds while the system is idle
- **THEN** the system SHALL request one run start event

#### Scenario: Short press ignored
- **WHEN** the start key is released before 3 seconds have elapsed
- **THEN** the system SHALL NOT request a run start event

### Requirement: Startup And Finish Beep Sequencing

The main control system SHALL coordinate a low-level-triggered active buzzer with run state transitions so that startup uses three beeps and finish uses one beep.

#### Scenario: Buzzer active level
- **WHEN** the system commands the buzzer to turn on
- **THEN** the buzzer control output SHALL be driven low
- **AND** when the system commands the buzzer to turn off, the buzzer control output SHALL be driven high

#### Scenario: Buzzer hardware binding
- **WHEN** the MSPM0 controls the buzzer
- **THEN** it SHALL use PB15 configured as a GPIO output
- **AND** PB15 low SHALL turn the active buzzer on

#### Scenario: Start timing on final startup beep
- **WHEN** the selected mode is ready to run
- **THEN** the system SHALL emit three startup beeps
- **AND** the system SHALL transition to chassis motion only at the final startup beep timing point

#### Scenario: Stop timing at finish beep
- **WHEN** the chassis reports that the target point has been reached
- **THEN** the system SHALL stop the chassis
- **AND** the system SHALL emit one finish beep

### Requirement: Vision Result Intake

The main control system SHALL accept structured UART results from the Raspberry Pi vision module and convert valid shape detections into target points.

#### Scenario: Receive valid shape result
- **WHEN** the vision UART receives a valid result for CIRCLE, TRIANGLE, RECT, or PENTAGON
- **THEN** the system SHALL map the result to target point A, B, C, or D respectively

#### Scenario: Receive no target result
- **WHEN** the vision UART receives a NONE or invalid result
- **THEN** the system SHALL keep the chassis locked and continue waiting, scanning, or enter an error state according to the current mode

### Requirement: Chassis Safety Lock

The main control system SHALL keep the chassis locked whenever recognition or gimbal scanning is not complete.

#### Scenario: Basic navigation recognition pending
- **WHEN** basic navigation mode is waiting for a valid vision result
- **THEN** the system SHALL keep motor motion disabled

#### Scenario: Advanced navigation scanning pending
- **WHEN** advanced navigation mode is scanning with the gimbal
- **THEN** the system SHALL keep motor motion disabled

### Requirement: Gimbal Scan Control

The main control system SHALL provide gimbal control states for centering, step scanning, target lock, and scan stop.

#### Scenario: Advanced mode begins scan
- **WHEN** advanced navigation mode starts recognition
- **THEN** the system SHALL command the gimbal to begin scanning while the chassis remains locked

#### Scenario: Vision target found during scan
- **WHEN** a valid vision result is received during gimbal scanning
- **THEN** the system SHALL stop gimbal scanning
- **AND** the system SHALL proceed to startup beep sequencing for the mapped target point

### Requirement: Chassis Command Interface

The main control system SHALL expose abstract chassis commands for lock, unlock, find line, follow target, and stop.

#### Scenario: Basic drive starts
- **WHEN** basic drive mode completes startup beep sequencing
- **THEN** the system SHALL unlock the chassis and command the chassis to find or follow the line toward C

#### Scenario: Navigation mode starts
- **WHEN** basic or advanced navigation mode completes startup beep sequencing with a valid target point
- **THEN** the system SHALL unlock the chassis and command the chassis to follow the route for that target point
