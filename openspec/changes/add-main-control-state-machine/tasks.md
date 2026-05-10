## 1. Proposal Confirmation

- [x] 1.1 Confirm the main-control scope for the MSPM0 empty project.
- [x] 1.2 Confirm that this stage may use abstract chassis and gimbal interfaces.

## 2. MSPM0 Project Structure

- [x] 2.1 Inspect `empty/` project entry points and generated-file boundaries.
- [x] 2.2 Add main-control modules: `app_state`, `buzzer`, `vision_uart`, `gimbal`, `chassis_iface`, `mode_key`, and `start_key`.
- [x] 2.3 Keep business logic out of generated `ti_msp_dl_config.*` files.

## 3. Main Control State Machine

- [x] 3.1 Define run modes: basic drive, basic navigation, and advanced navigation.
- [x] 3.2 Define states: idle, fixed detect, gimbal scan, start beep, running, target stop, finish beep, and error.
- [x] 3.3 Implement state transitions, timeouts, and error placeholders.

## 4. Interfaces

- [x] 4.1 Implement text vision UART frames: `$SHAPE,TARGET,VALID\r\n`.
- [x] 4.2 Implement non-blocking active-low buzzer sequencing.
- [x] 4.3 Define gimbal center, scan step, and stop interfaces.
- [x] 4.4 Define chassis lock, unlock, find-line, follow-target, stop, and status interfaces.
- [x] 4.5 Implement idle-only mode selection and active-mode latching.
- [x] 4.6 Implement start-key interface.
- [x] 4.7 Add `RUN_TIMEOUT_TICKS` watchdog for RUNNING state.

## 5. GPIO Binding

- [x] 5.1 Bind mode key to PA27 as pull-up GPIO input, active-low. *(changed from PA26 after hardware routing)*
- [x] 5.2 Bind start key to PA25 as pull-up GPIO input, active-low.
- [x] 5.3 Implement start key as a 3 second long-press detector.
- [x] 5.4 Bind buzzer to PB15 as active-low GPIO output.
- [x] 5.5 Add `MODE_KEY`, `START_KEY`, and `BUZZER` GPIO instances to `gpio_software_poll.syscfg`.

## 6. Build Files

- [x] 6.1 CCS managed-make project auto-discovers `.c` files in project root; no manual makefile edits needed.

## 7. Verification

- [x] 7.1 `openspec.cmd validate add-main-control-state-machine --strict --no-interactive` passed.
- [x] 7.2 SysConfig regenerated `ti_msp_dl_config.*` with MODE_KEY(PA27), START_KEY(PA25), BUZZER(PB15).
- [x] 7.3 Build in CCS Theia / TI Clang toolchain passed.
- [x] 7.4 Hardware-tested: PA27 mode switching OK, PA25 3-second start press OK, PB15 active-low buzzer OK.

## 8. Vision UART Parser

- [x] 8.1 Add `vision_uart_feed_byte()` peripheral-independent byte input API.
- [x] 8.2 Implement RX state machine: wait '$', collect until '\n', discard '\r'.
- [x] 8.3 Parse 3 comma-separated fields: SHAPE, TARGET, VALID.
- [x] 8.4 Strict cross-check: shape-target-valid must match mapping table; reject mismatches.
- [x] 8.5 Buffer overflow (>40 bytes) resets to wait-'$' state.
- [x] 8.6 `$NONE,X,0` produces has_result=true with valid=false.
- [x] 8.7 Bind VISION_UART to UART1 (PB6=TX, PB7=RX, 9600 baud), `vision_uart_poll()` drains RX FIFO into parser.
