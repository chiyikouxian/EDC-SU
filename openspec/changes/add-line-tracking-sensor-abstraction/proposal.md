# Change: Add Line-Tracking Sensor Abstraction

## Why

当前工程已经完成八路红光灰度模块接入，并通过实物测试验证可稳定运行。下一阶段的关键不是继续扩展底层 GPIO 读取，而是把 `X1~X8` 的原始数字量收敛成可供循迹算法和底盘同学复用的统一抽象，例如偏差 `error`、全白/疑似丢线、中心对线和多探头命中状态。

如果继续让上层直接消费 `gray_sensor_read_all()` 的原始 8 路值，后续很容易出现多个模块各自实现偏差计算、左右方向语义不统一、丢线判定不一致的问题。先建立“灰度状态到循迹抽象”的稳定边界，可以把传感器接入和底盘控制解耦，为后续硬件 A 联调和真车闭环打基础。

## What Changes

- 新增基于 8 路灰度状态的循迹抽象模块。
- 将 `X1~X8` 原始状态转换为统一的偏差 `error` 语义。
- 定义基础循迹状态，例如：检测到线、中心命中、疑似丢线、左右偏移方向。
- 增加 DEBUG_UART 调试输出或辅助字段，便于验证偏差和状态判断是否符合预期。
- 同步 README 中与循迹抽象直接相关的说明。

## Impact

- Affected specs: `line-tracking`
- Affected code:
  - 新增循迹抽象源文件，例如 `line_track.c/.h` 或 `track_state.c/.h`
  - `main.c` 调试输出
  - 可能复用 `gray_sensor.c/.h`
  - README / 调试说明
- Affected collaborators:
  - 用户本人：负责主控侧传感器抽象与接口统一
  - 硬件 A：后续复用该抽象结果对接底盘控制

## Out of Scope

- 不实现电机 PID、差速控制或真实底盘驱动。
- 不在本 change 中实现路口动作表、目标点路径选择或完整导航策略。
- 不修改 `chassis_iface.h` 外部合同。
- 不修改现有视觉 UART 协议和 `NAV/ADV` 模式语义。
- 不修改 `ti_msp_dl_config.*` 生成文件。
