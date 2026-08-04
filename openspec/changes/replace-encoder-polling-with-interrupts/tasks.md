## 1. Implementation

- [x] 1.1 在 SysConfig 中为左右编码器 A 相启用双边沿中断。
- [x] 1.2 重新生成配置并确认 GPIOB IRQ 宏与 `GROUP1_IRQHandler()` 入口。
- [x] 1.3 将编码器累计计数改为 `volatile`，实现 A 相 ISR 读取 B 相判向。
- [x] 1.4 增加左右累计计数原子快照，供速度 PID 和调试接口读取。
- [x] 1.5 停用 1 ms 状态表轮询，保留兼容空接口。

## 2. Verification

- [x] 2.1 完整编译并链接工程。
- [x] 2.2 架空验证正向、反向计数符号和自动制动。
- [x] 2.3 重新标定 `CHASSIS_SPEED_COUNTS_PER_CMD_X100`。
- [ ] 2.4 落地验证低速直行和循迹。
- [x] 2.5 按 12、15、18、20 分档验证提速稳定性。
