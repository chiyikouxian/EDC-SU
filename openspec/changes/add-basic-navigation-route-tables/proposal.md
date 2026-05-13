# Change: Add Basic Navigation Route Tables

## Why

Basic navigation mode already recognizes a vision target and calls `chassis_follow_target(target)`, but the chassis currently only loads a route table for `TARGET_C`. Targets A, B, and D fall back to generic line following and cannot reliably stop at the requested point.

## What Changes

- Add table-driven route execution for basic navigation from the center start position to targets A, B, C, and D.
- Keep the existing vision UART shape-to-target mapping unchanged:
  - circle -> A
  - triangle -> B
  - rectangle -> C
  - pentagon -> D
- Separate route selection by start context so basic drive A-to-C and basic navigation center-to-target routes do not share the same `TARGET_C` table.
- Reuse the existing node detection, route action dispatch, forced turn, straight-through suppression, and target-reached reporting mechanisms.
- Add debug visibility for selected route, target, node index, and action dispatch so route tables can be calibrated on the real field.
- Treat the initial center/circle departure as a calibrated route sequence, not coordinate navigation.

## Impact

- Affected specs: `basic-navigation-route` (new)
- Affected code:
  - `chassis_iface.c`: route table selection, basic navigation A/B/C/D route tables, route debug labels
  - `chassis_iface.h`: no public API signature change expected
  - `app_state.c`: may need a minimal call-site distinction between DRV and NAV/ADV route context
  - `README.md` or tuning docs: route table calibration notes
- Out of scope:
  - Vision protocol changes
  - Advanced-mode gimbal search changes
  - Encoder closed-loop speed control
  - Coordinate-based path planning
