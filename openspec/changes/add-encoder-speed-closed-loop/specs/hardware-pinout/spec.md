## MODIFIED Requirements

### Requirement: Encoder Pin Reservation

The system SHALL reserve encoder GPIO pins and SHALL permit encoder pulse feedback to drive wheel-speed closed-loop control.

#### Scenario: Encoder pins are reserved

- **WHEN** the GPIO encoder configuration is checked
- **THEN** `GPIO_ENCODER_LF_A` SHALL be assigned to `PB23`
- **AND** `GPIO_ENCODER_LF_B` SHALL be assigned to `PA17`
- **AND** `GPIO_ENCODER_LR_A` SHALL be assigned to `PA18`
- **AND** `GPIO_ENCODER_LR_B` SHALL be assigned to `PA12`
- **AND** `GPIO_ENCODER_RR_A` SHALL be assigned to `PB8`
- **AND** `GPIO_ENCODER_RR_B` SHALL be assigned to `PB12`
- **AND** `GPIO_ENCODER_RF_A` SHALL be assigned to `PB16`
- **AND** `GPIO_ENCODER_RF_B` SHALL be assigned to `PA31`

#### Scenario: Encoder replacement pins avoid reserved debug pins

- **WHEN** the replacement encoder pins are checked
- **THEN** `GPIO_ENCODER_LF_A` SHALL use `PB23`
- **AND** `GPIO_ENCODER_RF_B` SHALL remain on `PA31`
- **AND** encoder pins SHALL NOT use `PA0` or `PA1`
- **AND** encoder pulse counting MAY be used by the wheel-speed closed-loop controller
