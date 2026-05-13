## ADDED Requirements

### Requirement: Basic Navigation Route Tables

The system SHALL define table-driven route action sequences for basic navigation from the center start position to targets A, B, C, and D.

#### Scenario: Navigation route exists for every target
- **WHEN** basic navigation mode starts after receiving target A, B, C, or D
- **THEN** the chassis SHALL load a corresponding center-start route table
- **AND** the route table SHALL end with a stop action for the requested target point

#### Scenario: Target C route is context-specific
- **WHEN** basic drive mode starts toward C
- **THEN** the chassis SHALL use the existing A-to-C basic drive route
- **WHEN** basic navigation mode starts toward C
- **THEN** the chassis SHALL use the center-to-C navigation route instead

### Requirement: Navigation Route Execution

The chassis SHALL execute navigation route tables using confirmed node events and the existing left, right, straight, and stop route actions.

#### Scenario: Dispatch one action per node
- **WHEN** a route-relevant node is confirmed during navigation route execution
- **THEN** the chassis SHALL advance the route index exactly once for that physical node
- **AND** it SHALL dispatch the route action at the current route index

#### Scenario: Stop reports target reached
- **WHEN** the navigation route dispatches its stop action
- **THEN** the chassis SHALL brake or stop the motors
- **AND** `chassis_get_status()` SHALL return `CHASSIS_STATUS_TARGET_REACHED`

### Requirement: Navigation Debug Observability

The system SHALL expose enough route execution state over debug output to calibrate the center-start route tables on the real field.

#### Scenario: Active navigation route visible
- **WHEN** navigation route execution is active
- **THEN** debug output SHALL identify the requested target and route context
- **AND** debug output SHALL include the current confirmed node count or route action index

#### Scenario: Route calibration support
- **WHEN** a field test misses a turn or stops at the wrong target
- **THEN** developers SHALL be able to determine from debug output which route action was last dispatched
