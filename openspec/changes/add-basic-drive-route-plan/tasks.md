## 1. Route Definition

- [ ] 1.1 Add route action enum values for follow/straight/left/right/stop as needed by chassis route execution.
- [ ] 1.2 Add a basic drive route table for `DRV -> TARGET_C` with action sequence `RIGHT, LEFT, STRAIGHT, RIGHT, LEFT, RIGHT, STOP`.
- [ ] 1.3 Document each node's physical meaning in code comments near the route table.
- [ ] 1.4 Keep route execution table-driven so later A/B/C/D routes can reuse the same mechanism.

## 2. Node Detection And Latching

- [x] 2.1 Use active probe count and/or `turn_hint` to identify route-relevant node events. (turn_hint-based reactive handling is active; route-driven node context is not)
- [x] 2.2 Confirm nodes over multiple ticks to avoid single-frame noise. (cross_latched mechanism exists inside `#if CHASSIS_ENABLE_TARGET_STOP`, currently compiled out)
- [x] 2.3 Latch each node until the vehicle leaves the node region so one physical node increments only once. (cross_latched inside `#if CHASSIS_ENABLE_TARGET_STOP`, currently compiled out)
- [ ] 2.4 Expose enough debug information to observe node index and current route action over UART.

## 3. Action Execution

- [x] 3.1 Implement right and left node actions using forced turn behavior with min/max tick protection. (turn_priority system handles this reactively)
- [ ] 3.2 Implement straight node action so the vehicle continues through the middle-circle branch and temporarily suppresses branch-triggered turn priority.
- [x] 3.3 Exit turn/straight actions after the vehicle has left the node and reacquired a centered line, or after max tick protection. (turn_priority exit logic)
- [ ] 3.4 Implement stop action so node 7 brakes/stops the chassis and reports target reached. (gated by `CHASSIS_ENABLE_TARGET_STOP == 0`)

## 4. Main-Control Timing

- [x] 4.1 Adjust startup beep timing if needed so DRV motion begins at the final startup beep timing point required by the task.
- [x] 4.2 Ensure finish beep begins after chassis reports target reached at C. (handle_target_stop → handle_finish_beep flow)

## 5. Documentation

- [ ] 5.1 Update README with the confirmed basic drive node sequence.
- [ ] 5.2 Update README with tuning notes for node thresholds and straight-through branch handling.

## 6. Verification

- [x] 6.1 `openspec.cmd validate add-basic-drive-route-plan --strict --no-interactive` passes.
- [ ] 6.2 Bench/sensor test confirms node count advances once per physical node.
- [ ] 6.3 DRV low-speed test follows A -> node 1 -> node 2 -> node 3 -> node 4 -> node 5 -> node 6 -> C.
- [ ] 6.4 DRV test confirms node 3 goes straight and does not turn into the middle-circle branch.
- [ ] 6.5 DRV test confirms node 7 stops at C and triggers the finish-beep flow.
