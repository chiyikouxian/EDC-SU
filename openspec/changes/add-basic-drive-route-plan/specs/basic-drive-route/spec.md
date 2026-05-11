## ADDED Requirements

### Requirement: Basic Drive Route Table

The system SHALL define the basic drive route as a table-driven node action sequence from A to C, without using coordinate-based navigation.

#### Scenario: Basic drive route action sequence

- **WHEN** `RUN_MODE_BASIC_DRIVE` starts route execution toward `TARGET_C`
- **THEN** the chassis SHALL execute route actions in this order: right, left, straight, right, left, right, stop
- **AND** the stop action SHALL correspond to the C-point terminal region

#### Scenario: Route nodes have physical meanings

- **WHEN** a developer inspects the basic drive route table
- **THEN** each route step SHALL identify the node index and action
- **AND** comments or documentation SHALL describe the node's physical location on the A-to-C course

### Requirement: Basic Drive Node Recognition

The chassis SHALL convert route-relevant line features into confirmed node events, and each physical node SHALL increment the route node count only once.

#### Scenario: Confirm node event

- **WHEN** the grayscale/line state indicates a route-relevant node for the required confirmation duration
- **THEN** the chassis SHALL increment the route node index exactly once for that node
- **AND** it SHALL dispatch the matching route-table action

#### Scenario: Latch while inside node region

- **WHEN** the vehicle remains over the same physical node for multiple control ticks
- **THEN** the chassis SHALL keep the node latched
- **AND** it SHALL NOT increment the route node index again until the vehicle leaves the node region

### Requirement: Straight-Through Branch Handling

The basic drive route SHALL treat the middle-circle branch connection between the second and fourth turn nodes as a straight-through route node.

#### Scenario: Node 3 goes straight

- **WHEN** the chassis reaches node 3, where the small-square left-side path connects to the middle-circle horizontal line
- **THEN** the chassis SHALL execute a straight action
- **AND** it SHALL continue along the intended A-to-C route rather than turning into the circle branch

#### Scenario: Straight action suppresses branch turn cue

- **WHEN** the straight action is active at node 3
- **THEN** transient `turn_hint` or branch-like line cues SHALL NOT start a forced left or right turn
- **AND** normal turn handling SHALL resume after the vehicle has left the node

### Requirement: C-Point Stop In Basic Drive

The chassis SHALL stop at the seventh route node in basic drive mode and report target reached to the main state machine.

#### Scenario: Stop at C

- **WHEN** the basic drive route reaches node 7
- **THEN** the chassis SHALL stop or brake the motors
- **AND** `chassis_get_status()` SHALL return `CHASSIS_STATUS_TARGET_REACHED`

#### Scenario: Finish beep flow

- **WHEN** the main state machine observes `CHASSIS_STATUS_TARGET_REACHED`
- **THEN** it SHALL stop the chassis
- **AND** it SHALL enter the finish-beep flow so the buzzer emits one beep
