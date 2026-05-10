## 1. 底盘输入主路径切换

- [x] 1.1 引入 Hardware A 真实底盘源码，替换当前 `chassis_iface.c` dry-run stub。
- [x] 1.2 在底盘侧引入 `CHASSIS_USE_TRACK_BRIDGE` 编译开关（默认 1），主路径调用 `track_bridge_get()`。
- [ ] 1.3 补全 `track_read()` fallback 为真实可用路径：当前为 stub 占位（返回固定零值），仅在工程结构上保留了 `#else` 分支骨架。需硬件 A 合入本地传感器驱动后才能作为可用的本地独立调试路径。
- [x] 1.4 验证 `CHASSIS_USE_TRACK_BRIDGE == 1` 时 `chassis_iface.c` 不依赖硬件 A 本地传感器函数。

## 2. 参数宏引入与整理

- [x] 2.1 定义 `CHASSIS_TRACK_ERROR_SCALE`（默认 7），底盘内部通过 `bridge->error * CHASSIS_TRACK_ERROR_SCALE` 映射 PID 尺度。
- [x] 2.2 定义 `CHASSIS_LOST_LINE_SHORT_TICKS`（默认 6）和 `CHASSIS_LOST_LINE_LONG_TICKS`（默认 150），控制丢线层级。
- [x] 2.3 定义 `CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD`（默认 6），控制节点识别灵敏度。
- [x] 2.4 确保所有参数宏集中在一处（`chassis_iface.h` 顶部），方便联调时统一调整。
- [x] 2.5 在模块注释中说明每个宏的语义、单位和调参注意事项。

## 3. Stale / 丢线 / 找线 / 节点行为落地

- [x] 3.1 实现 stale 保护：`stale == true` → 立即刹车，跳过 PID，所有模式统一。
- [x] 3.2 实现短时丢线 → 原地找线（方向依据 `last_line_error` 符号：≥0 右旋，<0 左旋）。
- [x] 3.3 实现长时丢线 → 刹车保护。
- [x] 3.4 实现节点检测：`active_count >= CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD` → 进入节点处理逻辑。
- [x] 3.5 确认编码器闭环默认关闭（`CHASSIS_USE_ENCODER == 0`），第一轮验收不强制要求。

## 4. 主控侧集成与回归

- [x] 4.1 确认 `app_state.c` 无需修改即可编译（仅 chassis_iface.c 内部改写，合同 API 不变）。
- [x] 4.2 确认 `chassis_iface.h` 现有合同 API（`chassis_init/tick/lock/unlock/find_line/follow_target/stop/get_status/get_target`）行为语义不变。
- [x] 4.3 确认 `DRV / NAV / ADV` 三模式状态机流程不变，dry-run 阶段已结束，本次直接进入真实底盘循迹。

## 5. 调试可见性

- [x] 5.1 确认 `main.c` 的 `C=` 字段能反映真实底盘状态（通过 `chassis_get_status()` 返回 bridge 驱动的状态，不再是 dry-run 计时器推进）。
- [ ] 5.2 视需要增加底盘侧调试字段（如丢线计数、找线状态），但不强求；以 `C=` 和 `B=` 字段为最低可观测基线。

## 6. 文档同步

- [x] 6.1 在 README 新增"真实底盘接入"章节，说明：
  - 编译开关与参数宏列表
  - 主控→底盘数据流（`track_bridge_get()` → PID 差速）
  - 第一轮验收范围（循迹跑通，不含编码器闭环）
- [x] 6.2 更新 README "已完成"和"暂未完成"列表以反映底盘接入状态。
- [ ] 6.3 在 `HARDWARE_A_INTERFACE_SYNC_REPLY.md` 中标注本次 change 对应项已落地。

## 7. 验收

- [ ] 7.1 bench 验证：`CHASSIS_USE_TRACK_BRIDGE == 1` 编译通过，`B=` 数据正常流入底盘。
- [ ] 7.2 bench 验证：stale 刹车保护可用（手动触发 stale 后底盘立即停车）。
- [ ] 7.3 bench 验证：丢线→找线→恢复 基本流程可用。
- [ ] 7.4 bench 验证：节点检测 `active_count >= 6` 触发正确。
- [ ] 7.5 整车联调：DRV 模式下 A→C 循迹跑通。
- [ ] 7.6 整车联调：NAV/ADV 模式下循迹跑通（需视觉配合）。
- [ ] 7.7 `openspec.cmd validate add-chassis-track-bridge-consumption --strict --no-interactive` 通过。
