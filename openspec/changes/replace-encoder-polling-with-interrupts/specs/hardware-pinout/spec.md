## ADDED Requirements

### Requirement: Encoder Interrupt Assignment

The encoder A-phase pins SHALL be configured as GPIO inputs with both-edge interrupts while the corresponding B-phase pins remain GPIO direction inputs.

#### Scenario: Left encoder interrupt is configured

- **WHEN** SysConfig generates the encoder GPIO configuration
- **THEN** `GPIO_ENCODER_LEFT_A` on `PB23` SHALL enable rising and falling edge interrupts
- **AND** `GPIO_ENCODER_LEFT_B` on `PA17` SHALL remain an input used for direction

#### Scenario: Right encoder interrupt is configured

- **WHEN** SysConfig generates the encoder GPIO configuration
- **THEN** `GPIO_ENCODER_RIGHT_A` on `PB16` SHALL enable rising and falling edge interrupts
- **AND** `GPIO_ENCODER_RIGHT_B` on `PA31` SHALL remain an input used for direction
