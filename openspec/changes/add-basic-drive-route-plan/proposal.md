# Change: Add Basic Drive Route Plan

## Why

The competition basic drive mode requires the vehicle to start at A, follow the black line through the middle square, and stop at C. Current code can follow line data through `track_bridge_get()` and PID differential steering, but it does not yet encode the A-to-C route nodes, straight-through branch handling, or C-point stop behavior.

## What Changes

- Add a basic drive route plan for `DRV` mode targeting C.
- Encode the confirmed A-to-C node action sequence:
  - Node 1: right turn
  - Node 2: left turn
  - Node 3: straight through the middle-circle branch connection
  - Node 4: right turn
  - Node 5: left turn
  - Node 6: right turn
  - Node 7: stop at C
- Define route actions as table-driven behavior rather than coordinate-based navigation.
- Require the straight action to ignore transient `turn_hint` / branch cues while passing the node.
- Require C-point stop to report `CHASSIS_STATUS_TARGET_REACHED`, allowing the existing main state machine to stop and start the finish beep.
- Keep this change scoped to basic drive mode; A/B/C/D navigation route tables remain future work.

## Impact

- Affected specs: `basic-drive-route` (new)
- Affected code:
  - `chassis_iface.c`: route table, node action dispatch, target reached transition
  - `chassis_iface.h`: no required public API change expected
  - `app_state.c`: possible small timing adjustment for final-startup-beep launch semantics
  - `README.md`: document the basic drive route sequence after implementation
- Out of scope:
  - Basic navigation A/B/C/D route tables
  - Advanced navigation route tables
  - Encoder speed closed loop
  - Vision and gimbal behavior changes
