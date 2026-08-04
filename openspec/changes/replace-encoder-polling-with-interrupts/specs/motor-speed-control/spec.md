## ADDED Requirements

### Requirement: Interrupt-Driven Encoder Capture

The chassis SHALL capture wheel encoder feedback using GPIO edge interrupts when high-speed closed-loop control is enabled.

#### Scenario: Encoder edge is captured

- **WHEN** a rising or falling edge occurs on a left or right encoder A phase
- **THEN** the GPIO interrupt handler SHALL read the corresponding B phase
- **AND** it SHALL increment or decrement the wheel count according to direction
- **AND** it SHALL clear the handled GPIO interrupt status

#### Scenario: Speed controller reads encoder counts

- **WHEN** the 10 ms speed controller calculates measured wheel speed
- **THEN** it SHALL obtain a consistent snapshot of the interrupt-updated left and right counts
- **AND** it SHALL NOT depend on the former 1 ms quadrature polling decoder

#### Scenario: Interrupt handler remains bounded

- **WHEN** an encoder GPIO interrupt is handled
- **THEN** the handler SHALL NOT run PID calculations or blocking UART output
- **AND** it SHALL perform only the operations required to update and acknowledge encoder feedback
