# Project Context

## Purpose
本项目用于 2026 年电子设计竞赛校内选拔赛 C 题“自动识别导航系统”的方案设计、任务拆分、采购、开发计划和后续规格化管理。

项目目标是设计并制作一辆搭载单个云台和单个摄像头的小车，在白色喷绘场地上完成：
- 基础行进：从 A 点出发，沿黑色实线经过中间小正方形到达 C 点。
- 基础导航：从圆心出发，识别 A4 指示纸上的圆形、等边三角形、长方形、等边五边形，并导航到对应 A/B/C/D 目标点。
- 发挥导航：车体静止，通过云台扫描摄像头视野盲区，找到指示纸并识别图形，再出发循迹到对应目标点。
- 设计报告：整理系统方案、理论分析、电路与程序设计、测试方案和测试结果。

## Tech Stack
- 主控：MSPM0 系列开发板优先，负责状态机、电机控制、循迹、蜂鸣器、按键/拨码、云台控制和串口通信。
- 视觉：已确认使用树莓派 4B + USB 摄像头 + Python/OpenCV 方案，负责图形识别并通过 UART 向 MSPM0 主控输出识别结果。
- 视觉算法：传统机器视觉优先，包括灰度化、二值化、形态学处理、轮廓检测、多边形近似、圆形度/边数/长宽比分类。
- 底盘：双轮差速小车，N20 编码器电机，TB6612FNG/DRV8833 等双路 H 桥驱动。
- 循迹：6 路或 8 路红外/灰度循迹阵列，使用加权偏差 + PID/PD 控制。
- 云台：一维水平舵机云台优先，用于发挥模式中扫描下半圆环盲区。
- 通信：视觉模块与主控通过 UART 串口通信，3.3V TTL 电平。
- MSPM0 空白工程：当前仓库已有 `empty/`，为 LP_MSPM0G3507 NoRTOS 空白工程，包含 `empty.c`、`empty.syscfg`、`ti_msp_dl_config.*` 以及 gcc/iar/keil/ticlang 工程文件；硬件 B 的主控流程开发应优先基于该工程增量实现。
- 文档：Markdown 为主，OpenSpec 用于后续需求、变更和实现任务管理。

## Project Conventions

### Code Style
- 嵌入式 C 代码应按模块拆分：`motor`、`track`、`vision_uart`、`gimbal`、`buzzer`、`state_machine` 等职责清晰。
- 控制逻辑避免大量阻塞延时；蜂鸣、循迹、编码器、云台扫描等应尽量基于定时器节拍或状态机实现。
- 命名优先表达硬件含义和业务含义，例如 `target_point`、`track_error`、`motor_lock()`、`gimbal_scan_step()`。
- 串口协议字段使用明确枚举值，避免魔法数字；目标点统一为 `A/B/C/D/X`，图形统一为 `CIRCLE/TRIANGLE/RECT/PENTAGON/NONE`。
- Markdown 文档使用中文编写，表格用于分工、接口、采购和测试数据。

### Architecture Patterns
- 系统按“视觉识别、主控状态机、底盘循迹、云台扫描、电源结构、测试报告”分层。
- 主控状态机是整车流程中心，负责模式切换、识别锁、蜂鸣计时、电机锁定、路径选择和停车。
- 视觉模块只输出识别结果和有效性，不直接决定底盘动作。
- 底盘模块只负责找线、循迹、路口识别、路径动作表和停车，目标点由主控传入。
- 发挥模式必须采用“电机锁死 -> 云台扫描 -> 识别确认 -> 蜂鸣 -> 解锁电机 -> 循迹”的安全流程。
- 路径策略以实际场地标定为准：从圆心到 A/B/C/D 的路径应记录为路口动作序列，而不是只凭题图推断。

### Testing Strategy
- 单模块先测：摄像头识别、串口通信、电机 PWM、编码器测速、红外循迹、舵机角度、蜂鸣器时序。
- 再做分阶段联调：基础行进 -> 基础导航 -> 发挥导航 -> 提速优化。
- 每个模式记录多次测试数据，包括成功/失败、用时、识别结果、目标点、脱线/误识别/过冲原因。
- 关键验收目标：
  - 基础行进 A 到 C 稳定完成，蜂鸣和计时逻辑正确。
  - 基础导航四种图形均能触发正确目标点。
  - 发挥模式识别阶段车体不移动，只通过云台搜索。
- 代码或参数在赛前最后阶段应冻结，优先保证稳定性而不是继续大改。

### Git Workflow
- 当前项目以本地文档和嵌入式工程协作为主，未明确远程 Git 流程。
- 建议按模块提交：`docs:`、`vision:`、`track:`、`gimbal:`、`control:`、`hardware:`、`test:`。
- 涉及新功能、架构或接口变化时，先创建 OpenSpec change proposal，获确认后再实现。
- 不在同一次变更中混合无关重构和功能实现。

## Domain Context
- 题目有效图形与目标点映射：
  - 圆形 -> A 点
  - 等边三角形 -> B 点
  - 长方形 -> C 点
  - 等边五边形 -> D 点
- 场地为白底黑线，黑色实线是唯一有效行驶路线；实际场地不显示题图上的解释性文字。
- 指示纸为 A4，垂直地面放置，纸中心距地面 15cm。
- 小车搭载的云台和摄像头必须与车体一体，且只能有一个云台和一个摄像头。
- 基础导航和发挥模式中，出发瞬间指示纸会被抽走；出发后只能依靠黑线循迹和预设路径策略。
- 参赛分工：
  - 陈：视觉识别，平台为树莓派 4B + USB 摄像头 + Python/OpenCV。
  - 沈：底盘、循迹、电机和路径控制。
  - 苏：硬件 B，负责 MSPM0 主控集成、云台、电源、蜂鸣器、状态机、与视觉/底盘接口联调和报告统筹。当前用户本人负责该方向。

## Important Constraints
- 小车外边缘长宽建议不超过 15cm × 15cm。
- 行驶过程中不能人工触碰，明显偏离黑线视为脱线。
- 启动前蜂鸣 3 声，最后 1 声开始时同步开始计时。
- 到达目标点后蜂鸣 1 声，同时结束计时。
- 发挥模式识别阶段车体不允许移动或旋转，必须锁死电机，只能转动云台。
- 视觉识别必须优先稳定，不应依赖复杂模型训练；传统视觉算法更适合固定图形场景。
- 电源设计必须考虑电机和舵机电流冲击，主控与摄像头供电要稳定，所有模块必须共地。
- 若 OpenSpec 创建了 change proposal，在 proposal 被确认前不要开始实现。

## External Dependencies
- OpenSpec CLI：用于规格、变更提案和任务清单管理。PowerShell 环境下优先使用 `openspec.cmd`，避免 `openspec.ps1` 被执行策略拦截。
- MSPM0 开发环境：TI Code Composer Studio / SysConfig / MSPM0 SDK，具体取决于所用开发板。
- 视觉环境：树莓派 OS、Python、OpenCV、USB 摄像头。
- 硬件模块：N20 编码器电机、TB6612FNG/DRV8833、电池与降压模块、红外循迹阵列、舵机云台、蜂鸣器、按键/拨码开关。

## Hardware Pinout Notes

- Current motor AIN2 assignment is `PA13`, not `PA26`.
- `GPIO_MOTOR_AIN2_PORT` SHALL be `GPIOA`, `GPIO_MOTOR_AIN2_PIN` SHALL be `DL_GPIO_PIN_13`, and `GPIO_MOTOR_AIN2_IOMUX` SHALL be `IOMUX_PINCM35`.
- `PA26` is not used for A wheel AIN2 in the current EDC-SU firmware because Hardware A testing showed A wheel reverse failure on the PA26 route.
- `PB13` remains `GPIO_MOTOR_PWMB`; this is a different port from `PA13` and is not a conflict.
- Encoder pins remain reserved but disabled by `CHASSIS_USE_ENCODER = 0`.
