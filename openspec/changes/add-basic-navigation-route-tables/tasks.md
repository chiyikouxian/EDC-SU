## 1. Route Scope

- [ ] 1.1 Confirm the physical node order for center-start navigation to A, B, C, and D on the real field.
        *(A and D routes confirmed; B/C pending field measurement.)*
- [ ] 1.2 Define separate route tables for `NAV/ADV center -> A/B/C/D`.
        *(A defined: RIGHT,LEFT,LEFT,RIGHT,LEFT,STOP. D defined: RIGHT,LEFT,LEFT,RIGHT,RIGHT,STOP. B/C intentionally have no route; attempts enter ERROR. This item stays unchecked until B/C routes are confirmed and added.)*
- [x] 1.3 Keep the existing `DRV A -> C` route table behavior unchanged.
        *(route_drv_a_to_c preserved inline in chassis_iface.c with identical sequence: RIGHT,LEFT,STRAIGHT,RIGHT,LEFT,RIGHT,STOP.)*

## 2. Route Selection

- [x] 2.1 Add an internal route context or equivalent selection mechanism so `TARGET_C` can mean DRV A-to-C or NAV center-to-C depending on mode/start.
        *(chassis_route_lookup(mode, target, &len) dispatches by RunMode_t + TargetPoint_t, inlined in chassis_iface.c.)*
- [ ] 2.2 Ensure `chassis_follow_target(TARGET_A/B/C/D)` loads a valid navigation route in NAV/ADV.
        *(TARGET_A and TARGET_D: yes. TARGET_B/C: intentionally enter CHASSIS_ERROR; routes not yet confirmed, must not free-drive.)*
- [x] 2.3 Ensure `TARGET_NONE` still enters chassis error state.
        *(Unchanged: first check in chassis_follow_target returns CHASSIS_ERROR.)*

## 3. Action Execution

- [x] 3.1 Reuse existing route actions for left, right, straight, and stop.
        *(ROUTE_ACTION_RIGHT/LEFT/STRAIGHT/STOP dispatch unchanged in chassis_run_line.)*
- [x] 3.2 Add any required start-from-center action handling if the first detected center/circle feature differs from normal node detection.
        *(Not needed for current confirmed routes; first node is a standard intersection.)*
- [x] 3.3 Ensure each target route ends with `ROUTE_ACTION_STOP` and reports `CHASSIS_STATUS_TARGET_REACHED`.
        *(route_drv_a_to_c, route_nav_center_to_a, and route_nav_center_to_d end with ROUTE_ACTION_STOP.)*

## 4. Debug And Tuning

- [x] 4.1 Add or update UART debug output to identify the active route target/context and node/action index.
        *(Debug log includes mode (DRV/NAV/ADV), route_index, and route_length.)*
- [x] 4.2 Document the route table sequences and calibration method.
        *(Code comments in chassis_iface.c document each route; DRV vs NAV separation is explicit; B/C absence is documented.)*
- [x] 4.3 Preserve current tuning macros unless field testing proves a route-specific threshold is required.
        *(No tuning macro changes in this change.)*

## 5. Verification

- [x] 5.1 `openspec.cmd validate add-basic-navigation-route-tables --strict --no-interactive` passes.
        *(Verified from PowerShell in this workspace.)*
- [ ] 5.2 Bench test confirms NAV route selection for all four vision targets.
        *(Pending hardware availability; A and D are implemented, B/C are intentionally blocked.)*
- [ ] 5.3 Low-speed field test confirms center -> A, center -> B, center -> C, and center -> D each stop at the correct target.
        *(Pending: A and D routes ready for test; B/C routes not yet defined.)*
- [ ] 5.4 Regression test confirms DRV A -> C still follows the existing completed baseline route.
        *(Pending hardware availability.)*
