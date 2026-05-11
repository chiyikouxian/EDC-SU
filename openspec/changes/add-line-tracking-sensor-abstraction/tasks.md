## 1. Proposal Confirmation

- [x] 1.1 Confirm grayscale sensor integration is already stable on hardware.
- [x] 1.2 Confirm this stage only covers sensor-to-tracking abstraction, not motor closed-loop control.
- [x] 1.3 Confirm existing NAV and ADV semantics must remain unchanged.

## 2. Abstraction Design

- [x] 2.1 Define the channel ordering convention (`X1` leftmost through `X8` rightmost) as the only valid input order.
- [x] 2.2 Define one signed tracking error convention and document which direction is positive vs negative.
- [x] 2.3 Define basic tracking states such as line detected, all white / possible line lost, and center-hit.
- [x] 2.4 Keep the abstraction independent from any direct motor-control API.

## 3. Module Implementation

- [x] 3.1 Add a dedicated line-tracking abstraction module that consumes 8-channel grayscale states.
- [x] 3.2 Implement an API that computes normalized tracking state from raw `X1~X8` values.
- [x] 3.3 Implement a bounded error output suitable for later PID or rule-based steering layers.
- [x] 3.4 Ensure the module can be tested using existing `gray_sensor` data without changing the current state machine.

## 4. Debug Visibility

- [x] 4.1 Add debug visibility for the computed tracking error and high-level line state.
- [x] 4.2 Ensure debug output remains readable alongside the existing `S/M/T/C/G` fields.
- [x] 4.3 Document how to interpret left bias, right bias, and all-white states during bench testing.

## 5. Verification

- [x] 5.1 Verify the reported tracking error changes in the expected direction as a black line moves from X1 toward X8.
- [x] 5.2 Verify the abstraction reports center-hit when the line is under the middle probes.
- [x] 5.3 Verify all-white input is reported consistently as a possible line-lost condition at sensor-abstraction level.
- [x] 5.4 Verify existing DRV/NAV/ADV flows still compile and remain behaviorally unchanged.
- [x] 5.5 `openspec.cmd validate add-line-tracking-sensor-abstraction --strict --no-interactive` passes.
