# Change: Add Grayscale Sensor Module

## Why

当前项目的主控状态机、视觉 UART 和 dry-run 底盘流程已经稳定，但用户本人新的主线任务已经切换为灰度传感器开发。要推进基础行进和后续真车循迹联调，首先需要把八路灰度模块以稳定、可观测、可复用的方式接入 MSPM0 主控工程。

商家资料已经明确该模块为 `AD0/AD1/AD2 + OUT` 的 8 路复用式数字灰度模块，而不是 8 路并行 ADC 模块。若不先把这种模块形态和调试方式固化下来，后续很容易把“传感器接入”“偏差算法”“底盘控制”混在同一轮改动中，增加联调风险。

## What Changes

- 新增八路灰度传感器驱动模块，支持通过 `AD0/AD1/AD2` 选通并读取 `OUT` 数字输出。
- 新增灰度数据抽象接口，提供 8 路原始数字值和基础状态判断。
- 增加红光款模块的接入前提和 `ACTIVE_LEVEL` 可配置能力。
- 增加 DEBUG_UART 调试输出，便于验证 18mm 左右安装高度下的探头响应。
- 同步 README 中与灰度传感器接入直接相关的接线、调试和模块说明。

## Impact

- Affected specs: `grayscale-sensor`
- Affected code:
  - `gpio_software_poll.syscfg`
  - `main.c`
  - 新增灰度模块源文件，例如 `gray_sensor.c/.h`
  - 相关 README / 调试说明
- Affected collaborators:
  - 用户本人：负责 MSPM0 主控与灰度模块接入
  - 硬件 A：后续复用灰度状态/偏差结果做底盘联调

## Out of Scope

- 不实现底盘 PID、差速控制或真实电机闭环。
- 不在本 change 中实现路口动作表和完整循迹策略。
- 不修改现有 `NAV/ADV` 模式语义。
- 不处理云台或视觉同学负责的无刷电机开发。
- 不修改 `ti_msp_dl_config.*` 生成文件。
