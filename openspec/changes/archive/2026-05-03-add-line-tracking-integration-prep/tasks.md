## 1. 桥接模块设计与文档

- [x] 1.1 定义 `TrackBridgeData_t` 结构体，明确每个字段的语义、范围和有效条件。
- [x] 1.2 定义 `track_bridge.h` 公开接口（`track_bridge_update()`, `track_bridge_get()`, `track_bridge_debug_string()`）。
- [x] 1.3 在模块注释中写明：更新频率、有效状态、stale 条件、与硬件 A 的协作边界。
- [x] 1.4 确认 `track_bridge.h` 仅依赖 `app_common.h` 和 `line_track.h`，不引入底盘或电机依赖。

## 2. 桥接模块实现

- [x] 2.1 实现 `track_bridge.c`：维护一份静态 `TrackBridgeData_t` 副本。
- [x] 2.2 实现 `track_bridge_update()`：从 `gray_sensor` + `line_track` 计算最新桥接数据并写入静态副本。
- [x] 2.3 实现 `track_bridge_get()`：返回当前桥接数据的 const 指针（零拷贝、非阻塞）。
- [x] 2.4 实现 `track_bridge_debug_string()`：将桥接数据格式化为紧凑调试字符串。
- [x] 2.5 在 `track_bridge_update()` 中维护 tick 计数和 `stale` 标志：仅在 `APP_STATE_RUNNING` 调用时清除 `stale`，否则保持 stale。

## 3. 主循环集成

- [x] 3.1 在 `handle_running()` 中 `chassis_tick()` 之前插入 `track_bridge_update()` 调用。
- [x] 3.2 在 `main.c` 调试输出中新增 `B=` 字段，调用 `track_bridge_debug_string()`。
- [x] 3.3 验证 `B=` 输出与 `L=` 输出的一致性——桥接误差应等于 `LineTrackState_t.error`。
- [x] 3.4 bench 验证在 `IDLE`/`FDET`/`GSCAN`/`ERROR` 等非 RUN 状态下 `B=` 显示 stale 标记。

## 4. 文档同步

- [x] 4.1 在 README 新增"循迹桥接"章节，文档化：
  - `TrackBridgeData_t` 各字段语义
  - 主控→底盘数据流向
  - 更新频率与有效状态
  - 与硬件 A 的协作边界（主控提供什么、不提供什么）
- [x] 4.2 在 README 的调试输出示例和字段表中增加 `B=` 字段说明。
- [x] 4.3 在 README 的文件列表中增加 `track_bridge.c/h` 条目。

## 5. 调试与验证

- [x] 5.1 bench 验证：用手持黑线在探头阵列上左右移动，观察 `L=` 和 `B=` 字段一致性。
- [x] 5.2 bench 验证：在 DRV/NAV/ADV 三种模式下启动后，确认 `B=` 在 RUN 期间有数据、IDLE 期间显示 stale。
- [x] 5.3 bench 验证：全白场景下 `B=` 输出与 `L=---` 语义一致。
- [x] 5.4 回归验证：DRV/NAV/ADV 三模式 dry-run 流程无变化，状态机行为不变。
- [x] 5.5 `openspec.cmd validate add-line-tracking-integration-prep --strict --no-interactive` 通过。
