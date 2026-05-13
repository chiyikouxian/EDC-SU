## 1. Route Scope

- [x] 1.1 Confirm the physical node order for center-start navigation to A, B, C, and D on the real field.
        *(A/C: RIGHT,LEFT,LEFT,RIGHT,LEFT,STOP. B/D: RIGHT,LEFT,LEFT,RIGHT,RIGHT,STOP.)*
- [x] 1.2 Define separate route tables for `NAV/ADV center -> A/B/C/D`.
        *(A/B/C/D route tables are defined inline in chassis_iface.c.)*
- [x] 1.3 Keep the existing `DRV A -> C` route table behavior unchanged.
        *(route_drv_a_to_c preserved inline in chassis_iface.c with identical sequence: RIGHT,LEFT,STRAIGHT,RIGHT,LEFT,RIGHT,STOP.)*

## 2. Route Selection

- [x] 2.1 Add an internal route context or equivalent selection mechanism so `TARGET_C` can mean DRV A-to-C or NAV center-to-C depending on mode/start.
        *(chassis_route_lookup(mode, target, &len) dispatches by RunMode_t + TargetPoint_t, inlined in chassis_iface.c.)*
- [x] 2.2 Ensure `chassis_follow_target(TARGET_A/B/C/D)` loads a valid navigation route in NAV/ADV.
        *(TARGET_A/B/C/D all load valid NAV/ADV route tables.)*
- [x] 2.3 Ensure `TARGET_NONE` still enters chassis error state.
        *(Unchanged: first check in chassis_follow_target returns CHASSIS_ERROR.)*

## 3. Action Execution

- [x] 3.1 Reuse existing route actions for left, right, straight, and stop.
        *(ROUTE_ACTION_RIGHT/LEFT/STRAIGHT/STOP dispatch unchanged in chassis_run_line.)*
- [x] 3.2 Add any required start-from-center action handling if the first detected center/circle feature differs from normal node detection.
        *(Not needed for current confirmed routes; first node is a standard intersection.)*
- [x] 3.3 Ensure each target route ends with `ROUTE_ACTION_STOP` and reports `CHASSIS_STATUS_TARGET_REACHED`.
        *(route_drv_a_to_c and all four NAV/ADV routes end with ROUTE_ACTION_STOP.)*

## 4. Debug And Tuning

- [x] 4.1 Add or update UART debug output to identify the active route target/context and node/action index.
        *(Debug log includes mode (DRV/NAV/ADV), route_index, and route_length.)*
- [x] 4.2 Document the route table sequences and calibration method.
        *(Code comments in chassis_iface.c document each route; DRV vs NAV separation is explicit.)*
- [x] 4.3 Preserve current tuning macros unless field testing proves a route-specific threshold is required.
        *(No tuning macro changes in this change.)*

## 5. Verification

- [x] 5.1 `openspec.cmd validate add-basic-navigation-route-tables --strict --no-interactive` passes.
        *(Verified from PowerShell in this workspace.)*
- [ ] 5.2 Bench test confirms NAV route selection for all four vision targets.
        *(Pending hardware availability; A/B/C/D are implemented.)*
- [ ] 5.3 Low-speed field test confirms center -> A, center -> B, center -> C, and center -> D each stop at the correct target.
        *(Pending: A/B/C/D routes ready for test.)*
- [ ] 5.4 Regression test confirms DRV A -> C still follows the existing completed baseline route.
        *(Pending hardware availability.)*
