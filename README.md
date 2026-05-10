# EDC-SU

MSPM0G3507 / CCS Theia / SysConfig main-control project. This version is intended to be archived as a reusable line-following chassis base for future tasks.

## Stable Baseline

Recommended archive names:

```text
tag:    chassis-line-following-v1
branch: chassis-line-following-base
```

This baseline includes:

- 8-channel grayscale sensor scanning
- signed line error calculation through `line_track`
- `track_bridge` data contract between main control and chassis
- TB6612 four-wheel open-loop motor drive
- PID differential line following
- short lost-line recovery and long lost-line protection
- corner handling with brake + pivot turn
- turn-hint and short lost-line assisted corner entry
- DRV / NAV / ADV startup flow
- UART debug observability
- SysConfig pin mapping

Recommended reuse boundary:

- Keep `gray_sensor.c/h`, `line_track.c/h`, `track_bridge.c/h`, `motor.c/h`, and basic `chassis_iface` contracts stable.
- For future tasks, mainly change route logic, task state machine, vision protocol, gimbal/servo logic, and action tables.

## Current Verified State

- `DRV / NAV / ADV` can enter `RUN`.
- Normal line width can produce `C=FOLL`.
- Grayscale direction contract is verified: `X1` is left, `X8` is right.
- Left offset is negative, right offset is positive.
- `X4/X5` centered line produces `L=E:+000 HIT`.
- `GRAY_ACTIVE_LEVEL = 1`.
- A wheel `AIN2` is `PA13`, not `PA26`.
- Corner handling is now able to turn reliably enough for the current tuning baseline.
- Encoder closed-loop remains disabled: `CHASSIS_USE_ENCODER = 0`.

## RUN Data Path

```text
gray_sensor_read_all()
-> line_track_compute()
-> track_bridge_update()
-> chassis_tick()
-> track_bridge_get()
-> stale / lost-line / turn-hint / corner / PID logic
-> motor_set_speed()
```

Frozen contracts:

- Do not change `gray_sensor.c/h`.
- Do not change `line_track.c/h`.
- Do not change `track_bridge.c/h`.
- Do not change `chassis_get_status()` contract.
- Do not change `ChassisStatus_t`.
- Keep `track_bridge_update() -> chassis_tick()` order in RUN.
- Keep `motor_update_pwm()` at 1 ms and `app_state_tick()` at 10 ms.

## Current Chassis Parameters

Source: [chassis_iface.c](chassis_iface.c)

```c
#define CHASSIS_BASE_SPEED                 12
#define CHASSIS_MAX_SPEED                  28
#define CHASSIS_CURVE_SPEED                4
#define CHASSIS_FIND_SPEED                 16
#define CHASSIS_LOST_COAST_SPEED           7
#define CHASSIS_LOST_COAST_TURN_LIMIT      6
#define CHASSIS_TURN_HINT_CONFIRM_TICKS    1
#define CHASSIS_CROSS_CONFIRM              4
#define CHASSIS_CURVE_ERROR               300

#define CHASSIS_CORNER_ERROR              150
#define CHASSIS_CORNER_LOST_CONFIRM_TICKS  2
#define CHASSIS_CORNER_BRAKE_TICKS         4
#define CHASSIS_CORNER_TURN_TICKS         60
#define CHASSIS_CORNER_TURN_SPEED         20
```

PID:

```c
pid_init(&line_pid, 12, 0, 6, 18, 1200);
```

Corner behavior:

```text
large line error OR turn_hint OR short lost-line with last large error
-> CORNER_BRAKE
-> CORNER_TURN
-> return to normal PID after line is detected again
```

## Pinout

### User IO / UART

| Function | Pin | Notes |
|---|---|---|
| Mode key | PA27 | pull-up input, active low |
| Start key | PA25 | pull-up input, active low, about 30 ms debounce |
| Buzzer | PB15 | active buzzer, active low |
| Debug UART | PA10 TX / PA11 RX | UART0, 9600 8N1 |
| Vision UART | PB6 TX / PB7 RX | UART1, 9600 8N1 |

### Grayscale Sensor

| Function | Pin |
|---|---|
| AD0 | PB0 |
| AD1 | PB1 |
| AD2 | PB2 |
| OUT | PB3 |

### Motor Driver

| Function | Pin |
|---|---|
| AIN1 | PA8 |
| AIN2 | PA13 |
| PWMA | PA28 |
| BIN1 | PB9 |
| BIN2 | PB19 |
| PWMB | PB13 |
| CIN1 | PB17 |
| CIN2 | PB18 |
| PWMC | PB4 |
| DIN1 | PB24 |
| DIN2 | PA24 |
| PWMD | PB20 |
| STBY | PA15 |

`AIN2=PA13` is intentional. `PB13` remains `PWMB`; it is a different port and not a conflict.

### Encoder Reserved Pins

Encoder pins are reserved but not enabled in this baseline.

| Signal | Pin |
|---|---|
| LF_A | PA22 |
| LF_B | PA17 |
| LR_A | PA18 |
| LR_B | PA12 |
| RR_A | PB8 |
| RR_B | PB12 |
| RF_A | PB16 |
| RF_B | PA31 |

## Debug Output

Format:

```text
S=... M=... T=... C=... G=... L=... B=...
```

Fields:

| Field | Meaning |
|---|---|
| `S` | app state: `IDLE/FDET/GSCAN/SBEEP/RUN/TSTOP/FBEEP/ERR` |
| `M` | mode: `DRV/NAV/ADV` |
| `T` | target: `-/A/B/C/D` |
| `C` | chassis status: `IDL/FLIN/FOLL/RCHD/LOST/ERR` |
| `G` | raw grayscale sensor values |
| `L` | live line-track result |
| `B` | bridge data in RUN; `STALE` outside RUN |

Typical centered line:

```text
G=X1:0 X2:0 X3:0 X4:1 X5:1 X6:0 X7:0 X8:0 L=E:+000 HIT
```

## Vision UART Protocol

```text
$SHAPE,TARGET,VALID\r\n
```

Examples:

| Shape | Target | Frame |
|---|---|---|
| CIRCLE | A | `$CIRCLE,A,1\r\n` |
| TRIANGLE | B | `$TRIANGLE,B,1\r\n` |
| RECT | C | `$RECT,C,1\r\n` |
| PENTAGON | D | `$PENTAGON,D,1\r\n` |
| NONE | X | `$NONE,X,0\r\n` |

NAV / ADV startup order:

```text
press start key
-> NAV enters FDET, ADV enters GSCAN
-> send valid vision frame
-> SBEEP
-> RUN
```

Frames sent before pressing the start key are cleared by `vision_uart_clear()`.

## Git Archive Commands

Current remote:

```text
origin https://github.com/chiyikouxian/EDC-SU.git
```

After confirming this exact version is the desired base:

```bash
git tag -a chassis-line-following-v1 -m "stable chassis line following base"
git branch chassis-line-following-base
git push origin main
git push origin chassis-line-following-base
git push origin chassis-line-following-v1
```

For a future task:

```bash
git checkout -b new-task chassis-line-following-base
```

`Debug/` build outputs are ignored by `.gitignore`.
