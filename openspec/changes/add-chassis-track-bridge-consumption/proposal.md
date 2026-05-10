# Change: Add Chassis Track-Bridge Consumption

## Why

`track_bridge` 已通过硬件验收，`TrackBridgeData_t` 的 7 字段合同已固化为正式 spec。硬件 A 也已确认底盘侧完成了 `track_bridge_get()` 消费适配（包括 PID 差速控制、丢线保护、找线、节点识别、刹车停车），并在 [HARDWARE_A_INTERFACE_SYNC_REPLY.md](../../../HARDWARE_A_INTERFACE_SYNC_REPLY.md) 中明确了双方协作口径。

当前 `chassis_iface.c` 仍然是 dry-run stub，不消费任何真实循迹数据。下一步需要把 Hardware A 的真实底盘实现合入本工程，以 `track_bridge_get()` 作为正式主输入路径，并统一参数化所有可调阈值，使整车联调可以在同一份代码上收敛。

## What Changes

- **替换 `chassis_iface.c`：** 用 Hardware A 的真实底盘实现替换 dry-run stub。桥接使能后，底盘内部通过 `track_bridge_get()` 获取循迹偏差和线状态，驱动 PID 差速控制。
- **新增编译开关 `CHASSIS_USE_TRACK_BRIDGE`：** 正式集成设为 `1`（主路径为 `track_bridge_get()`）；硬件 A 本地独立调试时设为 `0`（回退到 `track_read()`）。
- **新增可配置参数宏（默认值均来自 Hardware A 确认口径）：**
  - `CHASSIS_TRACK_ERROR_SCALE` (默认 7)：将 bridge error `[-100, +100]` 映射到底盘 PID 误差尺度。
  - `CHASSIS_LOST_LINE_SHORT_TICKS` (默认 6) / `CHASSIS_LOST_LINE_LONG_TICKS` (默认 150)：短时丢线原地找线，长时丢线刹车保护。
  - `CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD` (默认 6)：`active_count >= 6` 视为交叉/节点区域。
- **明确 stale 保护行为：** `stale == true` 时统一立即刹车，不区分 DRV/NAV/ADV 模式，不进入 PID，不做短时维持。
- **明确找线方向规则：** 依据最后有效 error 符号决定旋转方向——`last_line_error >= 0` 右旋，`last_line_error < 0` 左旋。
- **第一轮联调不纳入编码器闭环：** 编码器闭环由 Hardware A 本地通过 `CHASSIS_USE_ENCODER` 预留，默认关闭，不属于本次验收范围。
- **同步文档：** README 新增真实底盘接入说明、参数表、验收范围；更新 `HARDWARE_A_INTERFACE_SYNC_REPLY.md` 相关引用。

## Impact

- Affected specs: `tracking-integration` (MODIFIED — 增加底盘消费侧需求)
- Affected code:
  - `chassis_iface.c`：dry-run stub **替换**为 Hardware A 真实底盘实现
  - `chassis_iface.h`：可能新增少量底盘侧公开接口声明（如 `chassis_emergency_stop` 等），但不修改主控侧调用合同
  - `app_state.c` / `app_state.h`：不修改
  - `track_bridge.h` / `track_bridge.c`：不修改
  - `main.c`：可能新增底盘状态调试字段
  - README / HARDWARE_A_INTERFACE_SYNC_REPLY.md：同步
- Affected collaborators:
  - 硬件 A（沈）：提供真实底盘源码，确认参数默认值
  - 硬件 B（苏）：集成、参数宏管理、联调验收

## 正式主路径与 Fallback 边界

| 模式 | `CHASSIS_USE_TRACK_BRIDGE` | 输入来源 | 使用场景 |
|------|---------------------------|---------|---------|
| 正式集成 | `1` | `track_bridge_get()` | 整车联调、比赛 |
| 硬件 A 本地调试 | `0` | `track_read()`（底盘本地） | 硬件 A 独立编译、无 bridge 文件时的回退 |

`CHASSIS_USE_TRACK_BRIDGE` 由本工程统一管理，正式集成分支固定为 `1`。

## Error 缩放参数化原则

- `bridge->error` 范围 `[-100, +100]`，底盘原 PID 使用约 `[-700, +700]` 尺度。
- 过渡阶段保留 `CHASSIS_TRACK_ERROR_SCALE = 7`，使 `line.error = bridge->error * 7` 映射到 `[-700, +700]`。
- `CHASSIS_TRACK_ERROR_SCALE` 是**底盘侧宏**，后续整车联调时可单独调整；最终目标是将 PID 直接适配 `[-100, +100]`，届时 scale 设为 1 或移除。
- 该 scaling 不作为主控侧长期合同——主控永远只提供 `[-100, +100]`。

## Stale 保护动作

- `stale == true` → 立即执行刹车保护（`chassis_brake()`），不进入 PID 循迹循环。
- 所有模式（DRV / NAV / ADV）统一处理，不做区分。
- stale 解除后（进入 RUNNING 且 bridge 更新）恢复正常循迹。

## 丢线处理与找线方向

- `line_detected == false` 或 `all_white == true` 均视为丢线。
- 短时丢线（< `CHASSIS_LOST_LINE_SHORT_TICKS` ticks）：原地找线，方向依据 `last_line_error`：
  - `last_line_error >= 0` → 右旋
  - `last_line_error <  0` → 左旋
- 长时丢线（≥ `CHASSIS_LOST_LINE_LONG_TICKS` ticks）：执行刹车保护。
- 丢线期间 bridge 仍正常更新，恢复检测后自动退出丢线状态。

## 节点阈值参数化

- `active_count >= CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD`（默认 6）视为交叉/节点区域。
- 该阈值仍属于需要实车标定的经验值，做成可调宏便于后续调整。

## 第一轮验收不纳入编码器闭环

- 编码器闭环由硬件 A 本地预留（`CHASSIS_USE_ENCODER`），第一轮联调默认关闭。
- 第一轮验收范围：`track_bridge_get()` → error PID → 左右差速 → 循迹跑通。
- 编码器闭环待四轮驱动硬件和速度反馈稳定后单独打开，不阻塞本轮进度。

## Out of Scope

- 不新增 `TrackBridgeData_t` 字段。
- 不修改 `app_state.h` 外部接口。
- 不修改 `track_bridge.h/.c`。
- 不修改视觉 UART 协议。
- 不修改 `ti_msp_dl_config.*` 或 `gpio_software_poll.syscfg`。
- 不实现路口动作表或目标点路径策略（后续 change 单独覆盖）。
