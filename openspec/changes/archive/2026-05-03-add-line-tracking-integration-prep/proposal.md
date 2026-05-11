# Change: Add Line-Tracking Integration Preparation

## Why

`line_track.c/h` 已经将 8 路灰度原始值转换为有符号有界偏差和线状态（`error`, `line_detected`, `all_white`, `center_hit`），并通过 `L=` 调试字段完成了 bench 验证。但当前 `line_track` 的输出仅被 `main.c` 的调试打印消费——`app_state.c` 和 `chassis_iface.c` 完全不知道循迹数据的存在。

在硬件 A 同学接入真实底盘之前，需要先做一层"准备层"：把 `line_track` 的输出固化为一份主控侧可稳定提供的循迹数据合同，并设计一个极薄的桥接模块让底盘侧（或替代 `chassis_iface.c` 的实现者）知道能从主控拿到什么、以什么频率拿到、在哪种状态下有效。这层准备不驱动电机、不算 PID、不假设差速参数，只解决"数据能流到边界"的问题。

## What Changes

- 新增 `track_bridge.h`：定义主控侧可提供给底盘侧的循迹数据合同，包含 `TrackBridgeData_t` 结构体和获取接口。
- 新增 `track_bridge.c`：实现从 `LineTrackState_t` 到 `TrackBridgeData_t` 的转换，并维护一份最新的桥接数据副本供底盘侧轮询。
- 在 `main.c` 调试输出中增加 `B=` 字段（bridge），展示桥接层当前持有的循迹数据，便于 bench 验证。
- 在 `handle_running()` 中调用桥接更新（仅写入桥接数据，不改变状态机行为）。
- 同步 README：新增"循迹桥接"章节，文档化主控→底盘循迹数据合同和协作边界。
- 不修改 `chassis_iface.h`、不修改 `app_state.h`、不修改视觉 UART 协议。

## Impact

- Affected specs: `tracking-integration`（新增）
- Affected code:
  - 新增 `track_bridge.c/h`
  - `main.c` 调试输出（新增 `B=` 字段）
  - `app_state.c` 的 `handle_running()` 中增加一行桥接更新调用
  - README 循迹桥接章节
- Affected collaborators:
  - 用户本人（苏）：负责主控侧循迹数据合同定义和桥接实现
  - 硬件 A（沈）：后续在 `chassis_iface.c` 内部或替代实现中，通过 `track_bridge_get()` 获取循迹偏差和线状态

## 与硬件 A 的协作边界

| 边界 | 说明 |
|------|------|
| **主控侧提供** | `TrackBridgeData_t`：包含 `error`、`line_detected`、`all_white`、`center_hit`、`active_count`，以及最后一次更新的 tick 计数 |
| **主控侧不提供** | 电机 PWM 占空比、差速目标值、PID 参数、路口策略、路径动作表 |
| **底盘侧职责** | 读取 `track_bridge_get()` → 自行完成 PID/PD 计算 → 驱动电机 |
| **更新频率** | 每个主控逻辑 tick 最多更新一次，当前实现目标是与 `chassis_tick()` 同频；当前工程默认 `TICK_MS=10ms`，但下游不得把该字段硬编码为永久固定物理时间 |
| **有效状态** | 仅在 `APP_STATE_RUNNING` 期间执行 `track_bridge_update()`；其他状态下桥接数据保持最后值且 `stale` 标志置位，非 `RUNNING` 数据仅供观测或调试参考，不视为实时循迹输入 |

## 与现有 DRV/NAV/ADV 状态机的兼容性

- `handle_running()` 当前调用 `chassis_tick()` 后检查底盘状态。本次变更仅在 `chassis_tick()` 之前增加一行 `track_bridge_update()` 调用。
- 桥接数据写入与底盘状态检查完全解耦——底盘当前 dry-run 行为不受任何影响。
- `DRV`、`NAV`、`ADV` 三种模式均已进入 `handle_running()`，因此桥接数据在所有模式运行期间均可获取。
- 不新增状态、不修改状态转换、不改变超时或错误处理逻辑。

## Tick 语义约束

- `TrackBridgeData_t.tick` 表示最近一次桥接数据更新所对应的主控逻辑 tick 计数，用于表达时序先后和数据新鲜度。
- `tick` 是逻辑节拍字段，不单独承诺永久等价于固定物理毫秒数；如果未来 `TICK_MS` 调整，下游模块不得继续按固定 10ms 去换算绝对时间。
- 下游判断桥接数据是否可用时，应优先参考 `stale` 标志，并仅将 `tick` 用作辅助时序信息，而不是唯一有效性判断依据。

## Out of Scope

- 不实现电机 PID 或差速控制。
- 不在本 change 中做真实底盘参数整定或路径策略。
- 不修改 `chassis_iface.h` 合同。
- 不修改 `ti_msp_dl_config.*` 生成文件。
- 不在 `chassis_iface.c` 内部消费 `track_bridge_get()`——那是硬件 A 的后续工作。
