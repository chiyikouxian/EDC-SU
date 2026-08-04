# Change: 启用编码器轮速闭环 PID

## Why

当前固件已经以 1 ms 周期采集左右编码器计数，但循迹输出仍直接作为电机 PWM 命令，无法补偿左右电机差异、电池电压变化和负载扰动。需要在循迹控制与电机 PWM 之间加入左右轮独立的速度闭环。

## What Changes

- 保持 `CHASSIS_USE_ENCODER=1`，新增并默认开启速度闭环开关。
- 以固定控制周期计算左右编码器增量，得到实际轮速反馈。
- 将循迹层给出的左右速度命令转换为目标编码器速度。
- 为左右轮分别运行可调 PI/PID 控制，并输出受限的电机 PWM。
- 在停车、制动、锁车、目标切换时清零速度控制器状态，防止积分残留。
- 增加串口调试数据，便于实车标定目标计数和 PID 参数。
- 保留一个编译期开关，可在联调异常时回退到原有开环控制。

## Impact

- Affected specs: `hardware-pinout`, new `motor-speed-control`
- Affected code: `chassis_iface.c`, `chassis_iface.h`, potentially `main.c` debug output
- Hardware dependency: 左右编码器方向必须正确，且 1 ms GPIO 轮询不能持续漏计
- Tuning dependency: 首次上车必须低速架空标定目标计数、`Kp`、`Ki`，再落地测试
