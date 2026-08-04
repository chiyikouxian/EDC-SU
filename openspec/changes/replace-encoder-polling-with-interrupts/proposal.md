# Change: 将编码器轮询替换为 GPIO 边沿中断

## Why

当前编码器采用 1 ms GPIO 软件轮询。实测速度命令 `8` 时已达到约 610 次跳变/秒，提高到 `12` 后接近轮询可可靠识别的上限，继续提速会漏计并导致速度 PID 错误增加 PWM。需要使用 GPIO 边沿中断捕获编码器脉冲，以支持稳定的高速速度闭环。

## What Changes

- 为左右编码器 A 相启用上升沿和下降沿 GPIO 中断。
- B 相保持普通 GPIO 输入，由 A 相中断读取 B 相判断旋转方向。
- 使用 `volatile` 累计左右编码器计数，并提供原子快照读取。
- 停用现有 1 ms A/B 状态轮询和四倍频状态表解码。
- 保留 `chassis_encoder_poll()` 空接口，避免修改主循环调用契约。
- 重新执行正反向极性验证和每命令单位目标计数标定。
- 中断测试通过后按 `12 → 15 → 18 → 20` 分档提高闭环速度。

## Impact

- Affected specs: `motor-speed-control`, `hardware-pinout`
- Affected code: `gpio_software_poll.syscfg`, `chassis_iface.c`, `chassis_iface.h`, generated SysConfig files
- Interrupt ownership: 新增 `GROUP1_IRQHandler()`，必须确认没有其他模块占用或合并处理
- Calibration: A 相双边沿为二倍频计数，现有 `CHASSIS_SPEED_COUNTS_PER_CMD_X100=76` 必须重新标定
