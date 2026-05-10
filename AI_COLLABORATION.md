# Codex 与 Claude 协作规则

## 角色分工

### Codex

Codex 负责需求澄清、方案审查、OpenSpec 变更发起、任务拆分、代码审查和验收建议。

Codex 不直接假设用户意图。遇到以下情况必须先向用户提问：
- 需求目标不明确。
- 硬件选型存在冲突，例如 ESP32-CAM 与树莓派方案同时出现但未确认。
- 会影响整体架构、接口协议、采购清单或比赛策略。
- 需要在多个可行方案之间取舍，且取舍会影响成本、时间或成功率。

### Claude

Claude 负责按已确认任务执行具体修改，包括文档、代码、测试脚本、嵌入式工程文件和 OpenSpec 任务清单更新。

Claude 执行前必须阅读：
- `AGENTS.md`
- `CLAUDE.md`
- `openspec/AGENTS.md`
- `openspec/project.md`
- 当前任务对应的 OpenSpec change 目录

Claude 不应绕过 OpenSpec 流程实现新功能。若任务涉及新能力、接口、架构或行为变化，必须先等待 Codex 发起 proposal 并由用户确认。

## 工作流

### 1. 需求阶段

用户提出目标后，Codex 先判断是否存在歧义。

若存在歧义，Codex 向用户提出具体问题，不创建任务。

若目标明确，Codex：
- 查阅 `openspec/project.md`、现有 specs 和 active changes。
- 创建或更新 OpenSpec change proposal。
- 给出任务范围、验收标准和风险点。

### 2. 执行阶段

用户批准 proposal 后，Codex 给 Claude 一段执行提示词，内容包括：
- change-id
- 必读文件
- 允许修改的文件范围
- 禁止修改的内容
- 实现任务顺序
- 验证命令
- 完成后必须汇报的内容

Claude 执行后需要返回：
- 修改文件列表
- 完成的任务项
- 运行过的验证命令和结果
- 未完成项或阻塞点
- 需要 Codex 审查的重点

### 3. 审查阶段

Codex 审查 Claude 的输出，重点检查：
- 是否符合用户已确认的需求。
- 是否符合 OpenSpec proposal 和 spec delta。
- 是否擅自引入未批准的功能或架构。
- 是否破坏已有文档、接口或约束。
- 是否有测试缺口、比赛风险或硬件落地风险。

审查结论分为：
- 通过：可以交给用户确认。
- 需修改：Codex 给 Claude 具体修复提示词。
- 需用户决策：Codex 暂停执行并向用户提问。

## 任务交接模板

Codex 给 Claude 的任务应使用以下格式：

```text
你是本项目的代码执行者。请严格按以下任务执行，不要扩大范围。

背景：
- 项目：2026 电赛 C 题自动识别导航系统
- change-id：<change-id>
- 目标：<一句话说明>

必读文件：
- AGENTS.md
- CLAUDE.md
- openspec/AGENTS.md
- openspec/project.md
- openspec/changes/<change-id>/proposal.md
- openspec/changes/<change-id>/tasks.md
- openspec/changes/<change-id>/specs/.../spec.md

允许修改：
- <文件或目录>

禁止修改：
- 不要修改未列出的文件。
- 不要改动 OpenSpec 托管指令块。
- 不要擅自改变硬件方案、通信协议或任务目标。

执行步骤：
1. <任务 1>
2. <任务 2>
3. <任务 3>

验证：
- 运行 `openspec.cmd validate <change-id> --strict --no-interactive`
- 如涉及代码，运行对应编译/测试命令；若无法运行，说明原因。

完成后请汇报：
- 修改文件列表
- 已完成任务项
- 验证命令和结果
- 遇到的问题
- 需要 Codex 审查的点
```

## 当前项目注意事项

- 当前 OpenSpec 尚无正式 specs，后续第一个功能应先创建 change proposal。
- PowerShell 环境下 `openspec` 可能触发 `.ps1` 执行策略错误，应使用 `openspec.cmd`。
- 视觉平台已由用户确认：树莓派 4B + USB 摄像头 + Python/OpenCV。后续不得再按 ESP32-CAM 方案发起任务，除非用户重新修改决策。
- 用户本人负责硬件 B：MSPM0 主控集成、云台、电源、蜂鸣器、状态机、与视觉/底盘接口联调。相关任务应优先围绕 `empty/` 中的 LP_MSPM0G3507 空白工程展开。
- 场地尺寸文档也存在不一致：题目 PDF 抽取显示 150cm × 150cm，部分后续文档写 200cm × 200cm。涉及场地尺寸、路径规划或报告前必须再次核对题目原文并向用户确认。
- 比赛策略优先级：基础行进稳定性 > 基础导航正确性 > 发挥模式拉分 > 提速。
