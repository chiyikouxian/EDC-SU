# Chassis Interface Contract

## Overview

`chassis_iface.h` is the **stable boundary** between Hardware B (main control)
and Hardware A (chassis/motor control). All chassis interaction from `app_state.c`
goes through this header. The implementation in `chassis_iface.c` can be swapped
without changing the state machine.

## Contract API

| Function | Semantics |
|---|---|
| `chassis_init()` | Reset to IDLE, locked, no target. Called once at startup. |
| `chassis_tick()` | Advance internal timing. Called every 10 ms from `handle_running()`. |
| `chassis_lock()` | Prevent motion commands. Motor outputs must be safe. |
| `chassis_unlock()` | Allow subsequent motion commands. |
| `chassis_find_line()` | If unlocked, begin finding the black line (basic drive mode). Ignored if locked. |
| `chassis_follow_target(target)` | If unlocked, begin route to target A/B/C/D. TARGET_NONE -> ERROR. Ignored if locked. |
| `chassis_stop()` | Immediately halt motion. Status returns to IDLE. |
| `chassis_get_status()` | Return current status (non-blocking). |
| `chassis_get_target()` | Return current/last commanded target point. |

## Status Values

| Status | Meaning |
|---|---|
| `IDLE` | No active motion command. |
| `FINDING_LINE` | Executing find-line (rotation to locate black line). |
| `FOLLOWING` | Following route toward commanded target. |
| `TARGET_REACHED` | Route complete, arrived at target. |
| `LINE_LOST` | Lost line tracking, cannot recover autonomously. |
| `ERROR` | Unrecoverable chassis fault. |

## Non-Blocking Requirement

All functions must return immediately. Route execution progress is reported
through `chassis_get_status()` on subsequent ticks. **No function may block
the 10 ms main loop.**

## Integration Path 1: Same-Project Merge

When Hardware A code is merged into this firmware:

1. Replace `chassis_iface.c` internals with calls to Hardware A functions.
2. `chassis_follow_target(TARGET_A)` -> call Hardware A's `route_execute(ROUTE_A)`.
3. `chassis_tick()` -> call Hardware A's `route_update()` and translate its status.
4. `chassis_get_status()` -> translate Hardware A status to `ChassisStatus_t`.
5. **Do not change `chassis_iface.h` or `app_state.c`.**

## Integration Path 2: Dual-Controller Communication

When chassis runs on a separate MCU:

1. Replace `chassis_iface.c` internals with UART/GPIO command encoding.
2. `chassis_follow_target(TARGET_A)` -> send command frame to chassis controller.
3. `chassis_tick()` -> read status frame from chassis controller, update local status.
4. Add communication timeout -> set `CHASSIS_STATUS_ERROR` on timeout.
5. **Do not change `chassis_iface.h` or `app_state.c`.**

## Current Dry-Run Implementation

The current `chassis_iface.c` is a **dry-run stub** for bench testing:

- `chassis_find_line()`: enters FINDING_LINE, transitions to TARGET_REACHED after 3 seconds.
- `chassis_follow_target()`: enters FOLLOWING, transitions to TARGET_REACHED after 5 seconds.
- No motor outputs are driven.
- Debug helpers available:
  - `chassis_debug_simulate_line_lost()`: force LINE_LOST during active run.
  - `chassis_debug_simulate_error()`: force ERROR at any time.

## Dry-Run Test Procedure

### Basic Drive (mode=DRV)

1. Power on -> serial shows `S=IDLE M=DRV T=- C=IDL`
2. Long-press PA25 for 3s -> three beeps -> `S=SBEEP`
3. After beeps -> `S=RUN M=DRV T=C C=FLIN`
4. After ~3s dry-run -> `S=TSTOP` -> one beep -> `S=IDLE`

### Basic Navigation (mode=NAV)

1. Short-press PA27 to switch to NAV -> `S=IDLE M=NAV`
2. Long-press PA25 -> `S=FDET` (waiting for vision)
3. Send `$RECT,C,1\r\n` via VISION_UART -> three beeps
4. After beeps -> `S=RUN M=NAV T=C C=FOLL`
5. After ~5s dry-run -> finish beep -> `S=IDLE`

### Advanced Navigation (mode=ADV)

1. Short-press PA27 twice to ADV -> `S=IDLE M=ADV`
2. Long-press PA25 -> `S=GSCAN` (gimbal scan stub)
3. Send `$CIRCLE,A,1\r\n` via VISION_UART -> three beeps
4. After beeps -> `S=RUN M=ADV T=A C=FOLL`
5. After ~5s -> finish beep -> `S=IDLE`

### Error Simulation

1. During `S=RUN`, call `chassis_debug_simulate_line_lost()` from debugger
2. Next tick -> `S=ERR` (chassis stopped and locked)
3. Recovery requires power cycle or manual reset
