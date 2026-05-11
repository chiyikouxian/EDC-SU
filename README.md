# EDC-SU 主控工程

本仓库是 2026 电赛 C 题“自动识别导航系统”的硬件 B 主控工程，基于 TI MSPM0G3507 / CCS Theia / SysConfig。

当前职责范围：

- 主控状态机
- 模式选择按键
- 启动按键
- 蜂鸣器提示
- 树莓派视觉 UART 通信
- 底盘循迹控制（PID + 路线表 + 传感器掩码）
- 后续云台与系统集成

## 当前进度

已完成并实测通过：

- **基础行进 A→C 全流程稳定通过**（路线表驱动，7 节点动作序列）
- 路线表驱动的节点动作分发（RIGHT/LEFT/STRAIGHT/STOP）
- 转弯传感器掩码机制，消除三岔路口对面线干扰
- 转弯后冷却期，防止转弯退出后立即反转
- PID 输出低通滤波，消除传感器抖动导致的抽搐
- 丢线惯性滑行期，避免短暂丢线触发旋转找线
- PA27 模式切换按键，低电平按下，空闲状态下循环切换 `DRV -> NAV -> ADV`
- PA25 启动按键，低电平按下，长按 3 秒触发启动
- PB15 有源蜂鸣器，低电平触发
- UART1 视觉通信，`PB6=TX`、`PB7=RX`、`9600 8N1`
- UART1 RX 纯中断接收，避免轮询与中断并发驱动同一解析器
- 视觉结果通过原子读取/清除接口从 UART ISR 交给主状态机
- 八路灰度传感器接入，PB0~PB3，AD0/AD1/AD2+OUT 复用式读取
- 循迹抽象层，最长黑带中心误差计算，有界偏差输出 [-100,+100]
- 循迹桥接准备层，主控→底盘数据合同，stale 生命周期正确
- 四路电机 PWM 驱动 + 加减速斜坡
- 底盘 PID 差速循迹（Kp=30, Ki=0, Kd=16）

暂未完成：

- A/B/D 目标路线表（结构已就绪，只需补充动作序列）
- 编码器速度闭环（`CHASSIS_USE_ENCODER=0`，后续联调打开）
- 云台 PWM 或步进电机实物控制
- 树莓派 OpenCV 实机识别程序接入
- 电源系统和整车联调

## 引脚分配

| 功能 | 外设/引脚 | 说明 |
|---|---|---|
| 模式按键 | PA27 GPIO 输入上拉 | 低电平按下 |
| 启动按键 | PA25 GPIO 输入上拉 | 低电平按下，长按 3 秒启动 |
| 蜂鸣器 | PB15 GPIO 输出 | 有源蜂鸣器，低电平响 |
| 调试串口 | UART0 PA10 TX / PA11 RX | 9600 8N1，输出状态 |
| 视觉串口 | UART1 PB6 TX / PB7 RX | 9600 8N1，接收树莓派视觉帧 |
| 灰度传感器 AD0 | PB0 GPIO 输出 | 通道选择位 0 (LSB) |
| 灰度传感器 AD1 | PB1 GPIO 输出 | 通道选择位 1 |
| 灰度传感器 AD2 | PB2 GPIO 输出 | 通道选择位 2 (MSB) |
| 灰度传感器 OUT | PB3 GPIO 输入上拉 | 8 路复用数字输出 |

## 灰度传感器

八路红光灰度模块 (`AD0/AD1/AD2 + OUT` 复用式数字输出)：

- 模块内部通过地址选通逐路切换 8 个探头，输出单路数字信号
- `X1`~`X8` 对应通道 0~7（`AD2/AD1/AD0 = 000~111`），依次从左到右排列
- 默认 `GRAY_ACTIVE_LEVEL=1`：探头在黑线上 → 红光被吸收 → 反射弱 → 比较器输出高电平
- 推荐探头距地面约 18mm
- 每通道建立延时 50μs，一轮 8 通道扫描约 400μs，不阻塞 10ms 主循环
- 在编译时定义 `GRAY_ACTIVE_LEVEL=0` 可适配白线/深色地面的非标准场地

### 调试输出

DEBUG_UART 每 500ms 状态行末尾追加 `G=` 字段：

```text
S=IDLE M=DRV T=- C=IDL G=X1:0 X2:0 X3:1 X4:1 X5:1 X6:0 X7:0 X8:0 L=E:+000 HIT B=E:+000 HIT
```

- 每路显示 `X<n>:<0|1>`，原始值直接对应探头数字输出
- `X3:1 X4:1 X5:1` 表示中间三个探头在黑线上，两侧在白区
- 调用 `gray_sensor_has_line()` / `gray_sensor_is_all_white()` 判断整体黑/白状态

## 循迹抽象（Line-Tracking Abstraction）

`line_track.c/h` 将 8 路灰度原始值转换为有符号有界偏差和高层线状态，供后续 PID 或规则控制层使用：

### 误差符号约定

| 符号 | 含义 | 物理意义 |
|------|------|----------|
| **正 (+) error** | 线在中心**右侧** | 底盘需转向右侧（顺时针/右轮减速） |
| **负 (−) error** | 线在中心**左侧** | 底盘需转向左侧（逆时针/左轮减速） |
| 零 | 线在中心 | 直行或微调 |

误差范围 **[-100, +100]**，单调变化：
- 线从 X1 移到 X8 → error 单调递增
- 线在单侧最边缘时达到满量程（±100）

### 状态输出

| 字段 | 含义 |
|------|------|
| `error` | 有符号偏差 `[-100, +100]` |
| `active_count` | 检测到线的探头数量 (0~8) |
| `line_detected` | 至少一个探头看到线 |
| `all_white` | 所有探头均为非激活电平（疑似丢线） |
| `center_hit` | `|error| ≤ 15` 且 `active_count ≥ 2`（中心对准） |

### 计算方式

加权质心法，纯整数运算：

```
centroid = Σ(pos × active) / active_count
error    = (centroid − 3.5) / 3.5 × 100
```

- 探头位置 `pos = 0..7`（X1=0, X8=7），物理中心在 3.5
- 全白（`active_count=0`）：`error=0, line_detected=false, all_white=true`
- 中心命中：`|error| ≤ 15 且 active_count ≥ 2`

### 调试输出

DEBUG_UART 每 500ms 状态行末尾追加 `L=` 和 `B=` 字段：

```text
S=IDLE M=DRV T=- C=IDL G=X1:0 X2:0 X3:0 X4:1 X5:1 X6:0 X7:0 X8:0 L=E:+000 HIT B=E:+000 HIT
```

- `E:sMMM`：有符号偏差，三位数字；正=线在右，负=线在左
- ` HIT` 后缀：中心命中（仅当 `center_hit=true` 时出现）
- `---`：全白/无线（`line_detected=false`）
- `L=` 为当前传感器帧的实时计算结果，`B=` 为桥接层锁存的最新 RUNNING 数据

```text
L=E:-042      → 线偏左，未居中
L=E:+000 HIT  → 线居中且命中
L=---         → 全白，疑似丢线
```

### 硬件验收结论

- 三模式 `DRV / NAV / ADV` 回归测试无异常，现有状态机流程保持不变
- `G=` 8 路通道顺序实测正确，`X1` 最左、`X8` 最右，无左右反接
- `L=` 方向约定实测正确：左侧为负、右侧为正，且黑线从 `X1 -> X8` 时误差单调递增
- `X4/X5` 同时命中时稳定输出 `L=E:+000 HIT`，中心命中判定与实现一致
- 全白场景稳定输出 `L=---`，与 `all_white / possible line lost` 语义一致

## 循迹桥接（Tracking Bridge）

`track_bridge.c/h` 是 `line_track` 和底盘之间的极薄准备层，把循迹数据固化为一份主控侧稳定输出的数据合同，供硬件 A 的真实底盘适配直接读取。

### 数据合同

| 字段 | 类型 | 含义 |
|------|------|------|
| `error` | `int16_t` | 有符号偏差 `[-100, +100]`，正=线在右侧，负=线在左侧 |
| `line_detected` | `bool` | 至少一个探头检测到线 |
| `all_white` | `bool` | 所有探头均为非激活电平（疑似丢线） |
| `center_hit` | `bool` | `|error| ≤ 15` 且 `active_count ≥ 2` |
| `active_count` | `uint8_t` | 检测到线的探头数量 (0~8) |
| `tick` | `uint32_t` | 最近一次桥接更新对应的主控逻辑 tick 计数（时序辅助字段，不永久承诺固定物理时长） |
| `stale` | `bool` | `true` = 数据非当前 RUN 周期产生，仅供观测/调试参考 |

### 更新频率与有效状态

- 桥接数据**仅在 `APP_STATE_RUNNING` 期间更新**，与 `chassis_tick()` 同频（每个主控逻辑 tick 一次）。
- 其他状态（IDLE / FDET / GSCAN / SBEEP / TSTOP / FBEEP / ERROR）不调用 `track_bridge_update()`，`stale` 标志保持置位。
- 下游判断桥接数据是否可用时，**优先参考 `stale` 标志**，`tick` 仅用作辅助时序信息。

### 与硬件 A 的协作边界

| 边界 | 说明 |
|------|------|
| **主控侧提供** | `TrackBridgeData_t`：偏差、线状态、中心命中、活跃探头数、tick、stale |
| **主控侧不提供** | 电机 PWM 占空比、差速目标值、PID 参数、路口策略、路径动作表 |
| **底盘侧职责** | 读取 `track_bridge_get()` → 自行完成 PID/PD 计算 → 驱动电机 |
| **接入方式** | 硬件 A 在 `chassis_iface.c` 内部或替代实现中调用 `track_bridge_get()`，获取 const 指针零拷贝读取 |

### API

```c
void track_bridge_init(void);                                              // 上电初始化，全部置零，stale=true
void track_bridge_update(const LineTrackState_t *state, uint32_t tick);    // 仅在 handle_running() 中调用
const TrackBridgeData_t *track_bridge_get(void);                           // 零拷贝、非阻塞、返回内部静态指针
const char *track_bridge_debug_string(void);                               // 调试用紧凑字符串
```

### 调试输出

```text
B=E:+035 HIT   → 线在右侧，中心命中，数据实时
B=E:-042       → 线在左侧，未居中，数据实时
B=---          → RUNNING 期间无线（全白），数据实时
B=STALE        → 非 RUNNING 状态，数据不可用于控制
```

### 硬件验收结论

- 非 RUNNING 状态（IDLE / FDET / GSCAN / SBEEP / TSTOP / FBEEP / ERROR）下 `B=STALE`，与 `stale` 语义一致
- RUNNING 状态下 `B=` 每 tick 正常刷新，误差与 `L=` 一致
- `L=` / `B=` 字段语义一致：误差符号、大小、` HIT` 后缀均同步
- `DRV / NAV / ADV` 三模式回归测试无异常，状态机流程和 dry-run 行为不变
- `enter_state` → `track_bridge_mark_stale` → `track_bridge_update` 的 stale 生命周期经硬件确认正确

## 真实底盘接入（Chassis Bridge Consumption）

`chassis_iface.c` 已从 dry-run stub 升级为 bridge-consuming 实现，以 `track_bridge_get()` 作为正式主输入路径。

### 数据流

```
track_bridge_get()                     [主控侧，零拷贝]
    ↓
chassis_read_line_input()              [底盘侧，error × CHASSIS_TRACK_ERROR_SCALE 映射]
    ↓
stale? → brake (立即刹车，全模式统一)   [安全保护，最高优先级]
    ↓ line detected
lost-line? → short: find-line / long: brake  [丢线层级]
    ↓ following
PID step → chassis_drive(forward, turn) [比例控制，首轮]
    ↓
motor_set_speed(L, R)                  [Hardware A TB6612 驱动]
```

### 编译开关与参数

| 宏 | 默认值 | 说明 |
|------|--------|------|
| `CHASSIS_USE_TRACK_BRIDGE` | `1` | 主输入路径：1=bridge, 0=stub fallback |
| `CHASSIS_TRACK_ERROR_SCALE` | `1` | bridge error [-100,+100] → PID 误差尺度 |
| `CHASSIS_BASE_SPEED` | `16` | 直线基础速度 |
| `CHASSIS_CURVE_SPEED` | `10` | 弯道降速 |
| `CHASSIS_FIND_SPEED` | `40` | 丢线旋转找线速度 |
| `CHASSIS_TURN_PRIORITY_SPEED` | `60` | 路口转弯原地旋转速度 |
| `CHASSIS_TURN_PRIORITY_TICKS` | `42` | 转弯最大持续 tick 数 |
| `CHASSIS_TURN_PRIORITY_MIN_TICKS` | `25` | 转弯盲转最小 tick 数（不检查退出条件） |
| `CHASSIS_TURN_COOLDOWN_TICKS` | `100` | 转弯后冷却期 tick 数 |
| `CHASSIS_LOST_LINE_SHORT_TICKS` | `18` | 丢线惯性滑行期（180ms），之后进入旋转找线 |
| `CHASSIS_LOST_LINE_LONG_TICKS` | `180` | 长时丢线刹车保护（1800ms） |
| `CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD` | `6` | active_count ≥ 此值视为节点/交叉 |
| `CHASSIS_CROSS_CONFIRM` | `4` | 节点确认帧数 |
| `CHASSIS_USE_ENCODER` | `0` | 编码器闭环，首轮默认关闭 |
| PID 参数 | Kp=30, Ki=0, Kd=16 | 输出限幅 ±30 |

### 安全规则

| 条件 | 动作 | 模式 |
|------|------|------|
| `stale == true` | 立即刹车，status=ERROR | DRV/NAV/ADV 统一 |
| 丢线 < 短时阈值 | 正常循迹 | — |
| 丢线 ≥ 短时阈值 | 原地找线，方向依 `last_line_error` 符号 | — |
| 丢线 ≥ 长时阈值 | 刹车保护，status=LINE_LOST | — |
| `active_count ≥ 6` | 节点计数 +1 | — |
| 节点数 ≥ 目标值 | status=TARGET_REACHED | — |

### 找线方向

- `last_line_error >= 0` → 右旋
- `last_line_error < 0` → 左旋

### 第一轮验收范围

- `track_bridge_get()` → error PID → 差速循迹跑通
- 不要求编码器闭环
- 不包含路口动作表/路径策略
- Motor 底层（TB6612 PWM）待硬件 A 合入

## 运行模式

| 模式 | 调试显示 | 当前行为 |
|---|---|---|
| 基础行进 | `DRV` | 不等待视觉结果，启动后按路线表循迹前往 C（已实测通过） |
| 基础导航 | `NAV` | 等待固定视角视觉结果，收到目标后循迹前往 A/B/C/D |
| 高级导航 | `ADV` | 进入云台扫描占位状态，收到视觉目标后停止扫描并循迹前往目标 |

## 状态输出

DEBUG_UART 每 500 ms 输出一行状态：

```text
S=IDLE M=DRV T=- C=IDL G=X1:0 X2:0 X3:0 X4:1 X5:1 X6:0 X7:0 X8:0 L=E:+000 HIT B=E:+000 HIT
```

字段含义：

| 字段 | 含义 |
|---|---|
| `S` | 主控状态：`IDLE/FDET/GSCAN/SBEEP/RUN/TSTOP/FBEEP/ERR` |
| `M` | 当前模式：`DRV/NAV/ADV` |
| `T` | 当前目标：`-/A/B/C/D` |
| `C` | 底盘状态：`IDL/FLIN/FOLL/RCHD/LOST/ERR` |
| `G` | 灰度探头：`X1:0 X2:0 ... X8:0` 每路独立 0/1 |
| `L` | 循迹偏差：`E:sMMM` 有符号误差，` HIT`=中心命中，`---`=无线 |
| `B` | 桥接数据：与 `L=` 同步的桥接层误差，`STALE`=非 RUN 状态 |

## 测试流程

### 1. 基础行进 DRV

1. 上电后默认 `DRV`
2. 长按 PA25 约 3 秒
3. 蜂鸣器响 3 声
4. 进入 `RUN`，按路线表循迹：RIGHT→LEFT→STRAIGHT→RIGHT→LEFT→RIGHT→STOP
5. 到达 C 点后自动停车
6. 蜂鸣器响 1 声，回到 `IDLE`

### 2. 基础导航 NAV

1. 短按 PA27 一次切换到 `NAV`
2. 长按 PA25 约 3 秒
3. DEBUG_UART 显示 `S=FDET`
4. 通过 UART1 发送视觉帧，例如：

```text
$RECT,C,1\r\n
```

5. 蜂鸣器响 3 声
6. 进入 `RUN M=NAV T=C C=FOLL`
7. 约 5 秒后 dry-run 模拟到达，完成蜂鸣后回到 `IDLE`

### 3. 高级导航 ADV

1. 短按 PA27 两次切换到 `ADV`
2. 长按 PA25 约 3 秒
3. DEBUG_UART 显示 `S=GSCAN`
4. 通过 UART1 发送视觉帧，例如：

```text
$CIRCLE,A,1\r\n
```

5. 云台扫描占位停止，蜂鸣器响 3 声
6. 进入 `RUN M=ADV T=A C=FOLL`
7. dry-run 到达后完成蜂鸣，回到 `IDLE`

## 视觉 UART 协议

格式：

```text
$SHAPE,TARGET,VALID\r\n
```

映射关系：

| 图形 | 目标点 | 示例 |
|---|---|---|
| `CIRCLE` | A | `$CIRCLE,A,1\r\n` |
| `TRIANGLE` | B | `$TRIANGLE,B,1\r\n` |
| `RECT` | C | `$RECT,C,1\r\n` |
| `PENTAGON` | D | `$PENTAGON,D,1\r\n` |
| `NONE` | 无目标 | `$NONE,X,0\r\n` |

注意：`\r\n` 表示真实的回车换行字节。串口助手中不要手动输入反斜杠字符，建议勾选 CRLF，或使用 HEX 发送。

## 底盘接口

`chassis_iface.c` 实现了完整的循迹控制，包括 PID 差速、路线表、传感器掩码和丢线保护。

主控只通过 `chassis_iface.h` 访问底盘：

```c
void chassis_lock(void);
void chassis_unlock(void);
void chassis_find_line(void);
void chassis_follow_target(TargetPoint_t target);
void chassis_stop(void);
ChassisStatus_t chassis_get_status(void);
```

## 路线表（Route Table）

基础行进 A→C 使用表驱动的节点动作序列：

| 节点 | 动作 | 物理位置 |
|------|------|----------|
| 1 | RIGHT | 出发后第一个路口右转 |
| 2 | LEFT | 左转进入小正方形上边 |
| 3 | STRAIGHT | 直行通过中间圆环连接处 |
| 4 | RIGHT | 右转出小正方形 |
| 5 | LEFT | 左转 |
| 6 | RIGHT | 右转接近C点 |
| 7 | STOP | 到达C点停车 |

节点检测：`active_count >= 6` 持续 4 个 tick 确认为节点。

转弯保护机制：
- 传感器掩码：右转时屏蔽 X1/X2，左转时屏蔽 X7/X8
- 盲转期：前 25 个 tick 不检查退出条件
- 冷却期：转弯退出后 100 个 tick 屏蔽新的 turn_hint
- STRAIGHT 动作：屏蔽 turn_hint 触发和 bias，防止被岔路带偏

## 主要文件

| 文件 | 作用 |
|---|---|
| `main.c` | 主入口，初始化模块，10 ms 主循环，DEBUG_UART 状态输出 |
| `app_state.c/h` | 主控状态机 |
| `mode_key.c/h` | PA27 模式按键 |
| `start_key.c/h` | PA25 长按启动 |
| `buzzer.c/h` | PB15 非阻塞蜂鸣器序列 |
| `vision_uart.c/h` | UART1 中断接收与视觉帧解析 |
| `chassis_iface.c/h` | 底盘接口合同，bridge-consuming 真实实现（原 dry-run stub 已替换） |
| `gimbal.c/h` | 云台接口占位 |
| `gray_sensor.c/h` | 八路灰度传感器接入与状态抽象 |
| `line_track.c/h` | 循迹抽象，质心加权误差计算，有界偏差输出 |
| `track_bridge.c/h` | 循迹桥接准备层，固化主控→底盘数据合同 |
| `gpio_software_poll.syscfg` | SysConfig 外设和引脚配置 |

## 下一步计划

建议优先级：

1. 补充 A/B/D 目标路线表（只需新增动作序列数组）
2. 实现云台最小动作闭环：回中、扫描步进、识别后停止
3. 与视觉同学联调树莓派 OpenCV 真实输出帧
4. 加入编码器速度闭环，提升循迹精度和一致性
5. 做整车供电、线缆、启动顺序和异常保护测试

## Git 提交说明

当前初始提交包含：

- CCS Theia 工程文件
- MSPM0 主控源码
- SysConfig 配置
- OpenSpec 变更文档
- 团队文档和题目资料

`Debug/` 构建产物已通过 `.gitignore` 排除。
