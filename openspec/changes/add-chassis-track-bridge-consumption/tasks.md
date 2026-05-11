## 1. 底盘输入主路径切换

- [x] 1.1 引入真实底盘控制实现，当前 `chassis_iface.c` 已不再是纯 dry-run 定时器 stub。
- [x] 1.2 正式控制路径通过 `track_bridge_get()` 读取主控侧循迹数据。
- [x] 1.3 当前正式集成固定使用 bridge 路径；`CHASSIS_USE_TRACK_BRIDGE` 仍在 `chassis_iface.h` 中保留为接口意图说明。
- [ ] 1.4 补全 `CHASSIS_USE_TRACK_BRIDGE == 0` 的本地 `track_read()` fallback。当前代码没有可切换的本地传感器 fallback 分支。
- [x] 1.5 验证当前 `CHASSIS_USE_TRACK_BRIDGE == 1` 场景下，`chassis_iface.c` 不依赖硬件 A 本地传感器函数。

## 2. 参数宏引入与当前代码值同步

- [x] 2.1 定义 `CHASSIS_TRACK_ERROR_SCALE`，当前代码值为 `1`，底盘内部通过 `bridge->error * CHASSIS_TRACK_ERROR_SCALE` 映射 PID 输入。
- [x] 2.2 定义 `CHASSIS_LOST_LINE_SHORT_TICKS` 和 `CHASSIS_LOST_LINE_LONG_TICKS`，当前代码值分别为 `18` 和 `180`（`TICK_MS=10`，对应 180ms / 1800ms）。
- [x] 2.3 定义 `CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD`，当前代码值为 `6`。
- [x] 2.4 定义 `CHASSIS_USE_ENCODER`，当前代码值为 `0`，编码器闭环默认关闭。
- [ ] 2.5 让丢线短/长阈值真正参与控制逻辑。当前 `chassis_run_line()` 一旦丢线就立即找线，未使用 `CHASSIS_LOST_LINE_SHORT_TICKS` / `CHASSIS_LOST_LINE_LONG_TICKS` 做分级保护。
- [x] 2.6 参数宏集中在 `chassis_iface.h` 顶部，便于联调时统一调整。

## 3. Stale / 丢线 / 找线 / 节点行为落地

- [x] 3.1 实现 stale 保护：`stale == true` 时立即刹车并进入 `CHASSIS_ERROR`，跳过 PID。
- [x] 3.2 实现丢线找线：`line_detected == false` 或 `all_white == true` 时进入原地找线，方向依据 `last_line_error` 符号。
- [x] 3.3 实现丢线惯性滑行期：丢线后前 `CHASSIS_LOST_LINE_SHORT_TICKS`(18) 个 tick 保持上一拍修正量继续前进，超过后进入旋转找线。长时丢线刹车保护阈值 `CHASSIS_LOST_LINE_LONG_TICKS`(180) 已定义但未接入独立刹车逻辑。
- [x] 3.4 启用节点检测和目标停车：通过路线表驱动（`route_active`），`active_count >= CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD` 确认后按表分发动作，STOP 动作触发 `CHASSIS_FINISHED`。
- [x] 3.5 确认编码器闭环默认关闭，第一轮验收不强制要求。
- [x] 3.6 增加 `turn_hint` 转弯优先逻辑：检测到分支/转弯提示后短时原地转向，重新中心命中后回到 PID 循迹。

## 4. 主控侧集成与回归

- [x] 4.1 `app_state.c` 已在 `APP_STATE_RUNNING` 中按 app/control tick（当前 `TICK_MS=10ms`）执行 `gray_sensor_read_all()` -> `line_track_compute()` -> `track_bridge_update()` -> `chassis_tick()`。
- [x] 4.2 `chassis_iface.h` 主控合同 API 保持稳定：`chassis_init/tick/lock/unlock/find_line/follow_target/stop/get_status/get_target`。
- [x] 4.3 `DRV / NAV / ADV` 三模式状态机入口流程保持存在，底盘消费路径已切到 bridge 驱动。
- [ ] 4.4 修正基础行进模式计时/出发时序：题目要求最后一声蜂鸣开始计时并出发，当前代码在蜂鸣序列完成后才解锁并进入 RUNNING。

## 5. 调试可见性

- [x] 5.1 `main.c` 的 `C=` 字段通过 `chassis_get_status()` 反映当前底盘状态。
- [x] 5.2 `main.c` 已输出 `G=` 原始灰度、`L=` 实时循迹计算、`B=` bridge 锁存数据。
- [x] 5.3 `chassis_debug_log()` 已提供底盘侧状态、误差、丢线计数、节点计数、锁定状态等 UART 调试信息。

## 6. 文档同步

- [x] 6.1 README 已包含“真实底盘接入”章节，说明 bridge -> PID 差速的数据流。
- [x] 6.2 README 参数表已同步当前代码值：`CHASSIS_TRACK_ERROR_SCALE=1`、短/长丢线阈值 `18` / `180`（TICK_MS=10，对应 180ms / 1800ms），并标注长丢线保护尚未落地。
- [x] 6.3 已移除对 `HARDWARE_A_INTERFACE_SYNC_REPLY.md` 的引用（该文件从未创建，不阻塞当前进度）。

## 7. 验收

- [ ] 7.1 bench 验证：`CHASSIS_USE_TRACK_BRIDGE == 1` 编译通过，`B=` 数据正常流入底盘。
- [ ] 7.2 bench 验证：stale 刹车保护可用。
- [ ] 7.3 bench 验证：丢线 -> 找线 -> 恢复基本流程可用。
- [ ] 7.4 bench 验证：`turn_hint` 转弯优先在实际岔路/小正方形入口处方向正确。
- [ ] 7.5 bench 验证：节点检测 `active_count >= 6` 能稳定触发。
- [ ] 7.6 整车联调：DRV 模式 A -> 中间小正方形 -> C 循迹跑通并停车。
- [ ] 7.7 整车联调：NAV/ADV 模式下循迹跑通，需要视觉/云台配合。
- [x] 7.8 `openspec.cmd validate add-chassis-track-bridge-consumption --strict --no-interactive` 通过。
