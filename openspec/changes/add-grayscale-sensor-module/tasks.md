## 1. Proposal Confirmation

- [x] 1.1 Confirm sensor variant is the red-light 8-channel module.
- [x] 1.2 Confirm target installation height is approximately 18mm from probe to ground.
- [x] 1.3 Confirm this stage only covers sensor integration and observability, not chassis closed-loop control.

## 2. Driver And Configuration

- [x] 2.1 Inspect merchant sample code and map the module shape as `AD0/AD1/AD2 + OUT`.
- [x] 2.2 Add grayscale sensor GPIO instances in `gpio_software_poll.syscfg` without editing generated files directly.
- [x] 2.3 Implement a dedicated grayscale sensor module to scan all 8 channels.
- [x] 2.4 Keep the per-channel settle delay bounded and suitable for the existing main loop.
- [x] 2.5 Support configurable active level for red-light module bring-up and track adaptation.

## 3. Data Abstraction

- [x] 3.1 Expose an API for reading all 8 raw digital values.
- [x] 3.2 Expose basic helper status: `gray_sensor_has_line()` (any channel active) and `gray_sensor_is_all_white()` (no channel active). The "possible line lost" condition is semantically identical to "all white" at sensor level; state-machine context determines whether to treat it as a lost-line event.
- [x] 3.3 Keep the module decoupled from chassis control and state-machine route semantics.

## 4. Debug Visibility

- [x] 4.1 Add DEBUG_UART output for grayscale sensor bring-up and calibration.
- [x] 4.2 Ensure the output is easy to correlate with X1~X8 channel ordering from the merchant documentation.
- [x] 4.3 Document the red-light module assumptions and the recommended 18mm installation height.

## 5. Verification

- [ ] 5.1 Verify all 8 channels change as expected when moving black/white targets under the probes. *(needs hardware)*
- [ ] 5.2 Verify active-level configuration matches the actual red-light module behavior on the user’s track material. *(needs hardware)*
- [ ] 5.3 Verify existing NAV and ADV flows still compile and remain behaviorally unchanged. *(needs CCS Theia build)*
- [x] 5.4 `openspec.cmd validate add-grayscale-sensor-module --strict --no-interactive` passes.
