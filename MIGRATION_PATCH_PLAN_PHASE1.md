# 涓诲伐绋嬫渶灏忚縼绉昏ˉ涓佽鍒掞紙绗竴闃舵锛氬紑鐜數鏈?+ track_bridge 宸＄嚎锛?
> 鐗堟湰锛?026-05-07 (淇鐗?鈥?淇濈暀 EDC 鍐呴儴鐘舵€佹満)
> 鐩爣锛氬皢 EDC-SHEN-Car 鐪熷疄搴曠洏鑳藉姏杩佸叆涓诲伐绋嬶紝绗竴闃舵浠呭紑鐜數鏈?+ track_bridge 宸＄嚎锛宔ncoder 鍏抽棴銆?> 绾︽潫锛氫笉鍋氫唬鐮佷慨鏀癸紝浠呰緭鍑鸿鍒掋€?> 缂栫爜锛歎TF-8

## 1. 闇€瑕佹柊澧炵殑鏂囦欢娓呭崟

### 1.1 浠?EDC-SHEN-Car 鐩存帴澶嶅埗锛? 涓枃浠讹級

| 婧愯矾寰?(EDC) | 鐩爣璺緞 (涓诲伐绋? | 璇存槑 |
|-------------|-------------------|------|
| `motor.h` | `motor.h` | 鏃犱緷璧栵紝鐩存帴澶嶅埗 |
| `motor.c` | `motor.c` | 渚濊禆 `ti_msp_dl_config.h`锛圙PIO_MOTOR_* 瀹忥級 |
| `pid.h` | `pid.h` | 鏃犱緷璧栵紝鐩存帴澶嶅埗 |
| `pid.c` | `pid.c` | 鏃犲閮ㄤ緷璧栵紝鐩存帴澶嶅埗 |

### 1.2 闇€瑕佷慨鏀圭殑鐜版湁鏂囦欢锛? 涓柊澧烇紝浠呰鏄庡悎骞朵綅缃級

| 鏂囦欢 | 鍔ㄤ綔 | 璇存槑 |
|------|------|------|
| `chassis_iface.c` | **鍚堝苟鍐欏叆**锛堥潪瑕嗙洊锛?| 璇﹁ 搂3 |
| `chassis_iface.h` | **鍚堝苟鍐欏叆**锛堥潪瑕嗙洊锛?| 琛ュ叏鎵╁睍鎺ュ彛澹版槑锛堣瑙?搂3.5锛?|
| `gpio_software_poll.syscfg` | **鎵嬪姩鏂板 GPIO 瀹炰緥** | 璇﹁ 搂2 |
| `main.c` | **鏂板 motor_update_pwm() 璋冪敤** | 璇﹁ 搂4.4 |

### 1.3 鏆備笉寮曞叆鐨勬枃浠?
| EDC 鏂囦欢 | 鍘熷洜 |
|----------|------|
| `track.c` / `track.h` | legacy fallback锛屼富宸ョ▼濮嬬粓 `CHASSIS_USE_TRACK_BRIDGE=1`锛屼笉闇€瑕?|
| `gray_sensor.c` / `gray_sensor.h` | 涓诲伐绋嬪凡鏈夛紝涓斾笌 EDC 涓€鑷达紙宸茬‘璁わ級 |
| `line_track.c` / `line_track.h` | 涓诲伐绋嬪凡鏈夛紝涓斾笌 EDC 涓€鑷达紙宸茬‘璁わ級 |
| `track_bridge.c` / `track_bridge.h` | 涓诲伐绋嬪凡鏈夛紝涓斾笌 EDC 涓€鑷达紙宸茬‘璁わ級 |
| `empty.c` | EDC 鐙珛娴嬭瘯 main锛屼笉杩涘叆涓诲伐绋?|

---

## 2. syscfg 闇€鏂板鐨?GPIO_MOTOR pin 娓呭崟

### 2.1 鏂板 GPIO 瀹炰緥锛欸PIO_MOTOR

鍦ㄥ綋鍓?`gpio_software_poll.syscfg` 涓柊澧炰竴涓?GPIO 瀹炰緥锛堝悕绉?`GPIO_MOTOR`锛夛紝鍖呭惈 13 涓?pin锛?
| $name | 绠¤剼 | 鏂瑰悜 | 璇存槑 |
|-------|------|------|------|
| AIN1 | PA8 | Output | TB6612 A 閫氶亾 IN1 |
| AIN2 | PA13 | Output | TB6612 A 閫氶亾 IN2 |
| BIN1 | PB9 | Output | TB6612 B 閫氶亾 IN1 |
| BIN2 | PB19 | Output | TB6612 B 閫氶亾 IN2 |
| STBY | PA15 | Output | TB6612 STBY 浣胯兘 |
| PWMA | PA28 | Output | TB6612 A 閫氶亾 PWM |
| PWMB | PB13 | Output | TB6612 B 閫氶亾 PWM |
| CIN1 | PB17 | Output | TB6612 C 閫氶亾 IN1 |
| CIN2 | PB18 | Output | TB6612 C 閫氶亾 IN2 |
| DIN1 | PB24 | Output | TB6612 D 閫氶亾 IN1 |
| DIN2 | PA24 | Output | TB6612 D 閫氶亾 IN2 |
| PWMC | PB4 | Output | TB6612 C 閫氶亾 PWM |
| PWMD | PB20 | Output | TB6612 D 閫氶亾 PWM |

### 2.2 涓嶈兘鍔ㄧ殑鐜版湁澶栬

浠ヤ笅 syscfg 瀹炰緥蹇呴』鍘熸牱淇濈暀锛屼笉寰椾慨鏀圭鑴氭垨鍚嶇О锛?
| 瀹炰緥 | $name | 绠¤剼 | 鍘熷洜 |
|------|-------|------|------|
| GPIO1 | BUZZER | PB15 | 涓诲伐绋嬬嫭鏈夛紝铚傞福鍣?|
| GPIO2 | MODE_KEY | PA27 | 涓诲伐绋嬬嫭鏈夛紝妯″紡閿?|
| GPIO3 | START_KEY | PA25 | 涓诲伐绋嬬嫭鏈夛紝鍚姩閿?|
| GPIO4 | GRAY_SENSOR | PB0-PB3 | 涓ゅ伐绋嬩竴鑷达紝浣嗕笉鑳芥敼瀹炰緥鍚?|
| UART0 | DEBUG_UART | PA10/PA11 | 涓ゅ伐绋嬩竴鑷?|
| UART1 | VISION_UART | PB6/PB7 | 涓诲伐绋嬬嫭鏈夛紝瑙嗚閫氫俊 |

### 2.3 鏆備笉鏂板锛欸PIO_ENCODER

绗竴闃舵 encoder 鍏抽棴 (`CHASSIS_USE_ENCODER=0`)锛屼笉闇€瑕佹柊澧?encoder 绠¤剼銆俿yscfg 涓?GPIO_ENCODER 瀹炰緥鏆傜紦锛屽悗缁樁娈靛啀鍔犮€?
### 2.4 鎿嶄綔姝ラ

1. 鍦?CCS Theia 涓墦寮€ `gpio_software_poll.syscfg`
2. 鏂板涓€涓?GPIO 瀹炰緥锛屽懡鍚嶄负 `GPIO_MOTOR`
3. 鍦?Associated Pins 涓€愪竴娣诲姞 13 涓?pin锛屾柟鍚戝拰鍚嶅涓婅〃
4. 淇濆瓨骞堕噸鏂扮敓鎴?`ti_msp_dl_config.h`
5. 楠岃瘉鐢熸垚鐨勫畯锛歚GPIO_MOTOR_AIN1_PORT`銆乣GPIO_MOTOR_AIN1_PIN` 绛変笌 EDC `ti_msp_dl_config.h` 涓殑瀹氫箟涓€鑷?
**鍥炴粴**锛氬鏋滅敓鎴愮殑瀹忓悕绉颁笌 EDC `motor.c` 涓殑寮曠敤涓嶅尮閰嶏紝淇敼 syscfg 涓殑 `$name` 浣垮叾涓€鑷达紝鎴栬皟鏁?`motor.c` 涓殑瀹忓紩鐢ㄣ€?
---

## 3. chassis_iface.c 鍚堝苟绛栫暐

### 3.1 鐜扮姸瀵规瘮

| 缁村害 | 涓诲伐绋?stub (366琛? | EDC 鐪熷疄瀹炵幇 (~750琛? |
|------|--------------------|-----------------------|
| 鍐呴儴鐘舵€佺被鍨?| 鐩存帴鐢?`ChassisStatus_t` | **鐙珛 `chassis_state_t` + 鏄犲皠鍑芥暟锛堜繚鐣欙紝浣滀负鍐呴儴鐘舵€佹満鍩哄簳锛?* |
| 鐘舵€佸彉閲?| `status` (ChassisStatus_t) | **`chassis_state` (chassis_state_t)锛堜繚鐣欙紝涓嶆浛鎹负 ChassisStatus_t锛?* |
| 鑺傜偣妫€娴?| 涓婂崌娌垮幓鎶?`was_in_node` | 杩炵画纭 + 閿佸瓨 `cross_count/cross_latched` |
| PID | 绠€鍖?P (`line.error/10`) | 瀹屾暣 PID (`pid_t` + `pid_update()`) |
| 涓㈢嚎澶勭悊 | 鐙珛鍑芥暟 `chassis_handle_lost_line()` | 鍐呰仈鍦?`chassis_run_line()` |
| 鐩爣鑺傜偣鏁?| `target_to_node_count()` 2/4/6/8 | `target_finish_nodes[]` 1/1/2/1 |
| motor 璋冪敤 | stub 绌哄嚱鏁?| 鐪熷疄 `motor_set_speed()` / `motor_brake()` 绛?|
| 鎵剧嚎 | `chassis_rotate_*`(50) | `chassis_rotate_*`(CHASSIS_FIND_SPEED=16) |
| bridge 娑堣垂 | `chassis_read_line_input(line*)` 鍐欏叆鍙傛暟 | `chassis_read_line_input()` 杩斿洖 struct |
| stale 妫€鏌?| 鐙珛 `chassis_check_stale()` | 鍐呰仈 `if (line.stale) { ... }` |
| 璋冭瘯鏃ュ織 | 鏃?| `chassis_debug_log()` UART 杈撳嚭 |
| encoder | 鏃?| `#if CHASSIS_USE_ENCODER` 瀹屾暣棰勭暀 |

### 3.2 蹇呴』淇濈暀鐨勪富宸ョ▼閫昏緫

浠ヤ笅涓诲伐绋?stub 涓殑閫昏緫**蹇呴』淇濈暀**锛屼笉寰楄 EDC 浠ｇ爜瑕嗙洊锛?
| 淇濈暀椤?| 浣嶇疆 (stub 琛屽彿) | 鍘熷洜 |
|--------|-----------------|------|
| `chassis_get_target()` 鈥?杩斿洖 `current_target` | L347-350 | 鍚堝悓 API锛岀鍚嶅拰璇箟涓嶅彉 |
| `chassis_lock()` 鈥?璁?`locked=true` + 鍒硅溅 | L296-300 | app_state 鐨?locked 璇箟 |
| `chassis_unlock()` 鈥?璁?`locked=false` | L302-305 | 瀵瑰簲 unlock |
| `chassis_stop()` 鈥?鍒硅溅 + `status=IDLE` | L335-340 | 鍚堝悓璇箟锛氱珛鍗冲仠姝紝鐘舵€佸洖 IDLE |
| `chassis_follow_target()` 鈥?TARGET_NONE 鈫?ERROR | L319-333 | app_state 渚濊禆姝ゆ鏌?|
| `chassis_find_line()` 鈥?榛樿 target=C | L307-317 | DRV 妯″紡鍏ュ彛 |
| debug helpers | L354-365 | bench test 杈呭姪锛屼繚鐣?|
| `chassis_line_input_t` 缁撴瀯浣?| L25-31 | 鍐呴儴绫诲瀷锛孍DC 瀹氫箟鏇村畬鏁达紙澶?active_count/stale 瀛楁锛?|

娉ㄦ剰锛氫互涓嬩富宸ョ▼ stub 閫昏緫**涓嶄繚鐣?*锛岀敱 EDC 瀹炵幇鏇夸唬锛?
| 鏇夸唬椤?| stub 鍋氭硶 | EDC 鏇夸唬鏂规 | 鍘熷洜 |
|--------|----------|-------------|------|
| 鍐呴儴鐘舵€佸彉閲?| 鐩存帴鐢?`ChassisStatus_t status` | `chassis_state_t chassis_state` | EDC 鍐呴儴鐘舵€佹満鏇村畬鏁达紙LOCKED/IDLE/LINE/FIND_LINE/CROSS/FINISHED/ERROR/LINE_LOST锛夛紝鐘舵€佽浆鎹㈤€昏緫宸茬粡杩囬獙璇?|
| `chassis_get_status()` | 鐩存帴杩斿洖 `status` | 閫氳繃 `chassis_map_state_to_status(chassis_state)` 鏄犲皠 | 淇濈暀 EDC 鐘舵€佹満瀹屾暣鎬х殑鍚屾椂锛屽澶栦粛杩斿洖鍚堝悓 `ChassisStatus_t` |
| `chassis_check_stale()` | 鐙珛 stale 妫€鏌ュ嚱鏁?| 鍐呰仈鍦?`chassis_run_line()` 绗竴浼樺厛绾?| EDC 宸插疄鐜?stale 鈫?ERR 鏃犳潯浠舵槧灏勶紙鍦?FIND_LINE 涔嬪墠锛夛紝绗﹀悎鍚堝悓 |

### 3.3 浠?EDC 杩佸叆鐨勯€昏緫

浠ヤ笅 EDC 閫昏緫**蹇呴』杩佸叆**涓诲伐绋?`chassis_iface.c`锛?
| 杩佸叆椤?| EDC 浣嶇疆 | 璇存槑 |
|--------|---------|------|
| `#include "motor.h"` | L2 | 鏂板渚濊禆 |
| `#include "pid.h"` | L3 | 鏂板渚濊禆 |
| 搴曠洏閫熷害/闃堝€煎畯 | L22-32 | CHASSIS_BASE_SPEED 绛夛紝褰撳墠 stub 涓‖缂栫爜 60/50 |
| `pid_t line_pid` 闈欐€佸彉閲?| L41 | 鏇挎崲 stub 涓殑绠€鍖?P 鎺у埗 |
| `chassis_line_input_t` 瀛楁鎵╁睍 | L66-72 | 澧炲姞 `active_count` / `stale` 瀛楁锛坰tub 宸叉湁浣嗗瓧娈典笉鍚岋級 |
| `clamp_speed()` | L79-84 | 閫熷害闄愬箙 |
| `chassis_read_line_input()` 鈥?bridge 璺緞 | L214-244 | 鏇挎崲 stub 涓殑绠€鍖栫増 |
| `chassis_apply_speed()` 鈥?鏃?encoder 鍒嗘敮 | L356-359 | 鐩存帴璋冪敤 `motor_set_speed(clamp_speed(L), clamp_speed(R))` |
| `chassis_drive()` 鈥?鐪熷疄瀹炵幇 | L451-465 | 鏇挎崲 stub 绌哄嚱鏁帮紝璋冪敤 `chassis_apply_speed()` |
| `chassis_forward/backward/turn_left/turn_right/rotate_left/rotate_right` | L467-506 | 鏇挎崲鍏ㄩ儴 stub |
| `chassis_brake()` 鈥?鐪熷疄瀹炵幇 | L430-438 | 璋冪敤 `motor_brake()` + PID reset |
| `chassis_emergency_stop()` 鈥?鐪熷疄瀹炵幇 | L419-428 | 璋冪敤 `motor_lock()` + PID reset |
| `chassis_debug_log()` | L174-212 | UART 璋冭瘯杈撳嚭锛堜粎鐘舵€佸彉鍖栨椂鎵撳嵃锛?|
| `chassis_state_t` 鏋氫妇瀹氫箟 | L33-42 | **瀹屾暣杩佸叆**锛孍DC 鍐呴儴鐘舵€佹満鍩哄簳锛? 鐘舵€?vs stub 鐨?6 鐘舵€侊級 |
| `chassis_map_state_to_status()` | L86-107 | **蹇呴』杩佸叆**锛屽皢鍐呴儴 `chassis_state_t` 鏄犲皠鍒板悎鍚?`ChassisStatus_t` |
| `chassis_state_name()` | L143-157 | 杈呭姪璋冭瘯鍑芥暟锛宍chassis_debug_log()` 渚濊禆 |
| `chassis_status_name()` | L159-171 | 杈呭姪璋冭瘯鍑芥暟锛宍chassis_debug_log()` 渚濊禆 |
| 涓㈢嚎/鎵剧嚎/鑺傜偣/鐩爣鍋滆溅閫昏緫 | 鏁翠綋 | 浠?`chassis_run_line()` 鍐呰仈閫昏緫鍚堝苟 |
| `chassis_set_target(chassis_target_t)` 瀹屾暣瀹炵幇 | L384-402 | 鏇挎崲 stub 涓殑 `(void)target` |
| `chassis_follow_target_ex()` | L704-708 | 淇濈暀鎵╁睍 API |
| `chassis_run_line()` 瀹屾暣瀹炵幇 | L541-625 | 鏍稿績 tick 閫昏緫 |
| `chassis_get_node_count()` | L679-682 | 鏇挎崲 stub 涓繑鍥?`node_count` |
| `chassis_is_finished()` | L684-687 | 鐘舵€佹鏌?|
| `chassis_encoder_reset/get_counts` | L627-669 | 淇濈暀浠ｇ爜璺緞锛宔ncoder=0 鏃惰繑鍥?0 |
| `chassis_init()` 涓皟鐢?`motor_init()` | L365 | 鏇挎崲 stub 涓棤 motor 鍒濆鍖栫殑鐗堟湰 |

### 3.4 浠?EDC 鏆傜紦杩佸叆鐨勯€昏緫

| 鏆傜紦椤?| 鍘熷洜 | 浣曟椂鍚敤 |
|--------|------|---------|
| `#include "track.h"` 鍙?`#else` fallback 璺緞 | 涓诲伐绋嬪缁?`CHASSIS_USE_TRACK_BRIDGE=1`锛屼笉闇€瑕?legacy fallback | 姘镐笉锛堝闇€瑕佺嫭绔嬭皟璇曠‖浠?A锛岀敤 EDC 宸ョ▼锛?|
| `#if CHASSIS_USE_ENCODER` 鍐呯殑鍏ㄩ儴浠ｇ爜 | 绗竴闃舵 encoder=0 | 绗簩闃舵锛宔ncoder 纭欢绋冲畾鍚?|
| `GROUP1_IRQHandler()` | encoder 涓柇澶勭悊 | 鍚?encoder |
| `speed_pid_apply()` / `encoder_*` 鍑芥暟 | encoder 闂幆 | 鍚?encoder |

### 3.5 chassis_iface.h 闇€琛ュ叏鐨勫０鏄?
涓诲伐绋?`chassis_iface.h` 褰撳墠缂哄皯浠ヤ笅 EDC 宸叉湁鐨勫０鏄庯紙鎵╁睍鎺ュ彛锛夈€傞渶琛ュ叏锛?
```c
/* 宸插湪涓诲伐绋?.h 涓湁澹版槑锛屼絾瀹炵幇闇€鏇挎崲 */
void chassis_brake(void);                  /* stub 鈫?鐪熷疄 motor_brake */
void chassis_emergency_stop(void);          /* stub 鈫?鐪熷疄 motor_lock */

/* 涓诲伐绋?.h 涓己澶憋紝闇€鏂板 */
void chassis_follow_target_ex(int target);  /* 缂哄け */
void chassis_run_line(void);               /* 缂哄け */
int  chassis_get_node_count(void);         /* 缂哄け */
int  chassis_is_finished(void);            /* 缂哄け */
```

**`chassis_set_target` 绛惧悕绛栫暐**锛?
涓诲伐绋?`chassis_iface.h` 涓０鏄庝负 `void chassis_set_target(int target);`锛堟墿灞曟帴鍙ｏ紝`int` 鍙傛暟锛夈€?EDC 涓０鏄庝负 `void chassis_set_target(chassis_target_t target);`锛坄chassis_target_t` 鏋氫妇鍙傛暟锛夈€?
绗竴闃舵绛栫暐锛?*鍦ㄤ富宸ョ▼ .h 涓繚鎸?`int` 绛惧悕涓嶅彉**锛屽湪 .c 瀹炵幇涓仛鍐呴儴杞崲锛?
```c
void chassis_set_target(int target)
{
    chassis_target_t ct;
    /* clamp int 鈫?chassis_target_t */
    if (target < (int)CHASSIS_TARGET_A || target > (int)CHASSIS_TARGET_D) {
        ct = CHASSIS_TARGET_C;
    } else {
        ct = (chassis_target_t)target;
    }
    /* 浠ヤ笅涓?EDC chassis_set_target(chassis_target_t) 閫昏緫涓€鑷?*/
    chassis_target = ct;
    node_count = 0;
    cross_count = 0;
    cross_latched = 0;
    lost_count = 0;
    pid_reset(&line_pid);
    /* encoder reset 鍦?encoder=0 鏃舵棤鎿嶄綔 */
}
```

杩欐牱涓诲伐绋嬪凡鏈夌殑璋冪敤鏂癸紙`chassis_follow_target_ex(int)` 绛夛級鏃犻渶淇敼绛惧悕锛孍DC 鍐呴儴浠?`chassis_target_t` 缁х画杩愪綔銆?
### 3.6 鍚堝苟鎿嶄綔椤哄簭

```
1. 澶囦唤涓诲伐绋?chassis_iface.c 鈫?chassis_iface_stub_backup.c
2. 浠?EDC chassis_iface.c 澶嶅埗瀹屾暣鍐呭鍒颁富宸ョ▼
3. 鍥炴 搂3.2 鐨勪繚鐣欓」锛堢敤澶囦唤鏂囦欢瀵规瘮鎭㈠锛?4. 鍒犻櫎 搂3.4 鐨勬殏缂撻」锛坋ncoder block / track fallback锛?   娉ㄦ剰锛歝hassis_state_t + chassis_map_state_to_status() 涓嶅湪鍒犻櫎娓呭崟涓?5. 璋冩暣 #include 娓呭崟锛堝幓鎺?track.h锛屽姞涓?motor.h / pid.h锛?6. 缂栬瘧锛屼慨澶嶅畯/绗﹀彿涓嶅尮閰?7. 鍔熻兘楠岃瘉
```

---

## 4. 绗竴闃舵楠屾敹鏍囧噯

### 4.1 缂栬瘧閫氳繃

- `tiarmclang` 缂栬瘧闆堕敊璇浂璀﹀憡
- 鎵€鏈夌洰鏍囨枃浠堕摼鎺ユ垚鍔燂細`chassis_iface.o`銆乣motor.o`銆乣pid.o`銆乣gray_sensor.o`銆乣line_track.o`銆乣track_bridge.o`銆乣app_state.o`銆乣main.o` 绛?- 鐢熸垚鐨勭鍙蜂腑 `GPIO_MOTOR_*` 瀹忓潎鏉ヨ嚜 syscfg 鐢熸垚锛屾棤鏈畾涔夊紩鐢?
### 4.2 鏁版嵁娴佹椂搴忛獙璇?
鍦?`handle_running()` (app_state.c:131-165) 涓‘璁わ細

```c
/* 1. bridge update 鈥?蹇呴』鍦?chassis_tick 涔嬪墠 */
gray_sensor_read_all(gv);
line_track_compute(gv, &lt);
track_bridge_update(&lt, state_ticks);

/* 2. chassis tick 鈥?娑堣垂鍒氭洿鏂扮殑 bridge */
chassis_tick();

/* 3. 鐘舵€佽疆璇?*/
ChassisStatus_t cs = chassis_get_status();
```

楠屾敹鏂规硶锛氬湪 `track_bridge_update()` 鍚庢彃鍏ヤ复鏃舵柇瑷€鎴栧湪璋冭瘯鍣ㄤ腑璁炬柇鐐癸紝纭 `bridge.stale == false` 鏃惰繘鍏?`chassis_tick()`銆?
### 4.3 chassis_get_status() 鐘舵€佽鐩?
閫氳繃 UART 涓插彛 `C=` 瀛楁楠岃瘉浠ヤ笅鐘舵€佸潎鑳借繑鍥烇細

| 鐘舵€?| 瑙﹀彂鏉′欢 | 涓插彛棰勬湡 |
|------|---------|---------|
| FOLL | 姝ｅ父宸＄嚎涓?| `C=FOLL` |
| FLIN | 鐭殏鍋忕榛戠嚎 6 tick | `C=FLIN`锛堝簳鐩樻棆杞級 |
| LOST | 鎸佺画鍋忕 150 tick | `C=LOST`锛堝埞杞︼級 |
| ERR | bridge stale锛堥潪 RUNNING 鐘舵€佹垨鏈?update锛?| `C=ERR` |
| RCHD | 鍒拌揪 target_finish_nodes 鑺傜偣鏁?| `C=RCHD`锛堝埞杞︼紝`S=TSTOP`锛?|

**鍐呴儴鐘舵€佹満绛栫暐**锛堜慨姝ｏ級锛?
绗竴闃舵**淇濈暀** EDC 鐨?`chassis_state_t` 鍐呴儴鐘舵€佹満锛屼笉閲囩敤"鐩存帴鐢?`ChassisStatus_t` 鍙橀噺鏇夸唬"鐨勬柟妗堛€傜悊鐢憋細

1. EDC 鍐呴儴鐘舵€佹満锛? 鐘舵€侊細LOCKED/IDLE/LINE/FIND_LINE/CROSS/FINISHED/ERROR/LINE_LOST锛夋瘮鍚堝悓 `ChassisStatus_t`锛? 鐘舵€侊細IDLE/FLIN/FOLL/RCHD/LOST/ERR锛夋洿缁嗙矑搴︺€?2. 鍐呴儴鐘舵€佽浆鎹㈤€昏緫锛堜涪绾?鈫?鐭椂 FLIN / 闀挎椂 LOST銆佽矾鍙ｇ‘璁?鈫?CROSS銆佽妭鐐瑰埌杈?鈫?FINISHED 绛夛級宸茬粡鍦?EDC 宸ョ▼涓€氳繃 UART 瀹炴祴楠岃瘉銆?3. 鍒犻櫎鍐呴儴鐘舵€佹満浼氱牬鍧忔墍鏈夊凡楠岃瘉鐨勭姸鎬佽浆鎹㈣矾寰勶紝椋庨櫓杩滃ぇ浜庢敹鐩娿€?
**C= 瀛楁閫傞厤绛栫暐**锛?
`chassis_get_status()` 閫氳繃 `chassis_map_state_to_status(chassis_state)` 杩斿洖鍚堝悓 `ChassisStatus_t`銆傛槧灏勮〃锛?
| 鍐呴儴 `chassis_state_t` | 鍚堝悓 `ChassisStatus_t` | 璇存槑 |
|------------------------|------------------------|------|
| `CHASSIS_LOCKED` | `CHASSIS_STATUS_IDLE` | 閿佹瑙嗕负 IDLE |
| `CHASSIS_IDLE` | `CHASSIS_STATUS_IDLE` | |
| `CHASSIS_FIND_LINE` | `CHASSIS_STATUS_FINDING_LINE` | **FLIN** |
| `CHASSIS_LINE` | `CHASSIS_STATUS_FOLLOWING` | **FOLL** |
| `CHASSIS_CROSS` | `CHASSIS_STATUS_FOLLOWING` | 璺彛浠嶅睘浜庡贰绾夸腑 |
| `CHASSIS_FINISHED` | `CHASSIS_STATUS_TARGET_REACHED` | **RCHD** |
| `CHASSIS_ERROR` | `CHASSIS_STATUS_ERROR` | **ERR** |
| `CHASSIS_LINE_LOST` | `CHASSIS_STATUS_LINE_LOST` | **LOST** |

涓诲伐绋?`main.c` 璋冭瘯杈撳嚭涓?`C=` 瀛楁閫氳繃 `chassis_names[chassis_get_status()]` 鑾峰彇鍚嶇О銆備笂琛ㄦ槧灏勪繚璇?`C=` 杩斿洖 IDL/FLIN/FOLL/RCHD/LOST/ERR 鍏鍊硷紝涓庝富宸ョ▼鐜版湁 `chassis_names[]` 鏁扮粍涓€鑷淬€?
**C= 瀛楁闂涓嶅簲閫氳繃鍒犻櫎鍐呴儴鐘舵€佹満瑙ｅ喅**锛岃€屽簲閫氳繃浠ヤ笅鏂瑰紡閫傞厤锛?
- 淇濈暀 `chassis_map_state_to_status()` 瀹屾暣鏄犲皠
- 涓诲伐绋?`main.c` 涓?`chassis_names[]` 鏁扮粍涓嶅彉锛? 鍏冪礌瀵瑰簲 6 涓?`ChassisStatus_t` 鍊硷級
- 濡傞渶鏇寸粏绮掑害鐨勫簳鐩樺唴閮ㄧ姸鎬佽瀵燂紙濡傚尯鍒?LINE vs CROSS锛夛紝鍦?`chassis_debug_log()` 鐨?`"CH:"` 琛屼腑鏌ョ湅 `chassis_state_name()` 杈撳嚭锛? 鐘舵€佺缉鍐欙級

### 4.4 motor_update_pwm() 绋冲畾璋冪敤鍏ュ彛

EDC `motor.c` 浣跨敤杞?PWM 璋冨埗锛?0 姝?phase锛夛紝`motor_update_pwm()` 蹇呴』姣?1ms 璋冪敤涓€娆°€?
褰撳墠涓诲伐绋?`main.c` 寰幆涓猴細

```c
while (1) {
    start_key_scan();
    app_state_tick();           // 10ms 鍛ㄦ湡锛堝唴閮ㄧ敤 state_ticks 鎺у埗锛?    // debug output 500ms
    delay_cycles(CPUCLK_FREQ / 1000 * TICK_MS);  // 寤舵椂 10ms
}
```

**闂**锛氬綋鍓嶅惊鐜綋姣?10ms 鎵ц涓€娆★紝浣?`motor_update_pwm()` 闇€瑕佹瘡 1ms 璋冪敤銆傞渶瑕佸皢涓诲惊鐜敼涓?1ms 鍛ㄦ湡锛屾垨灏?PWM 鏇存柊鍐呰仈鍒?`motor_set_speed()` 璋冪敤涓€?
**寤鸿鏂规**锛氬皢 `main.c` 寰幆鏀逛负 1ms 鍛ㄦ湡锛屽唴閮ㄧ敤璁℃暟鍣ㄥ垎棰戯細

```c
while (1) {
    motor_update_pwm();                     // [姣?1ms] PWM 鐩镐綅鎺ㄨ繘

    start_key_scan();                       // [姣?1ms] 鎸夐敭鎵弿浠嶄繚鎸?
    if (++tick_cnt >= 10) {                 // [姣?10ms]
        tick_cnt = 0;
        app_state_tick();                   // 10ms 閫昏緫
    }

    if (++debug_cnt >= 500) {               // [姣?500ms]
        debug_cnt = 0;
        // 璋冭瘯杈撳嚭
    }

    delay_cycles(CPUCLK_FREQ / 1000 * 1);   // 寤舵椂 1ms锛堝師 10ms 鈫?1ms锛?}
```

杩欐槸 main.c 鐨勫敮涓€鏀瑰姩鐐广€傚叧閿槸 `motor_update_pwm()` 鍦ㄦ瘡 1ms 杩唬涓皟鐢紝`app_state_tick()` 浠嶄繚鎸?10ms 涓€娆°€?
濡傛灉涓嶆兂鏀瑰彉涓诲惊鐜矑搴︼紝澶囬€夋柟妗堬細鍦?`chassis_drive()` 姣忔琚皟鐢ㄦ椂绔嬪嵆鎵ц涓€娆?`motor_update_pwm()`锛堝嵆 PWM 鏇存柊璺熼殢 motor_set_speed 璋冪敤棰戠巼 = 10ms锛夈€備絾杩欎細瀵艰嚧 PWM 杞皟鍒跺け鏁堬紙phase 鎺ㄨ繘閫熷害鍙樻參锛夛紝瀵逛簬绗竴闃舵寮€鐜凡瓒冲銆?
---

## 5. 椋庨櫓鍜屽洖婊氱偣

### 5.1 椋庨櫓鐭╅樀

| # | 椋庨櫓 | 姒傜巼 | 褰卞搷 | 缂撹В鎺柦 |
|---|------|------|------|---------|
| R1 | GPIO_MOTOR pin 涓庣‖浠?A 瀹為檯鎺ョ嚎涓嶄竴鑷?| 涓?| 鐢垫満涓嶈浆鎴栬浆閿欐柟鍚?| 鍏堝湪 EDC 宸ョ▼涓‘璁?pin 鏄犲皠琛ㄤ笌瀹為檯鎺ョ嚎涓€鑷达紝鍐嶅啓鍏ヤ富宸ョ▼ syscfg |
| R2 | `motor_update_pwm()` 璋冪敤棰戠巼涓嶈冻瀵艰嚧鐢垫満鏃犲姏 | 楂?| 杞?PWM 澶辨晥 | 閲囩敤 搂4.4 鐨?1ms 寰幆鏂规 |
| R3 | `chassis_set_target(int)` vs `chassis_target_t` 绛惧悕涓嶅尮閰?| 涓?| 缂栬瘧璀﹀憡鎴栫洰鏍囪祴鍊奸敊璇?| 绗竴闃舵鍦?`chassis_set_target()` 鍐呴儴鍋氳寖鍥存鏌?+ clamp |
| R4 | `chassis_map_state_to_status()` 鏄犲皠閬楁紡鎴栨槧灏勯敊璇紝瀵艰嚧 `C=` 瀛楁杩斿洖鍊间笌 stub 棰勬湡涓嶅悓 | 涓?| `C=` 瀛楁杩斿洖閿欒鐨勭姸鎬佸悕 | 鍦?搂4.3 鏄犲皠琛ㄥ熀纭€涓婏紝鍚堝苟鍚庣珛鍗抽€氳繃 UART 杈撳嚭楠岃瘉 6 涓悎鍚岀姸鎬佺殑 `C=` 鍊硷紱`chassis_debug_log()` 鐨?`"CH:"` 琛屽悓鏃惰緭鍑哄唴閮ㄧ姸鎬佸悕鍜屽悎鍚岀姸鎬佸悕鐢ㄤ簬浜ゅ弶楠岃瘉 |
| R5 | `chassis_debug_log()` 浣跨敤鐨?`DEBUG_UART_INST` 涓?main.c 鍏辩敤锛屽彲鑳戒骇鐢熷瓧绗︿氦閿?| 浣?| UART 杈撳嚭涔辩爜 | 宸叉敼涓轰粎鐘舵€佸彉鍖栨椂鎵撳嵃锛堜笉鍛ㄦ湡鎬ц緭鍑猴級锛屼笌 main.c 500ms 杈撳嚭鍐茬獊姒傜巼浣?|
| R6 | 鑺傜偣鏁?`target_finish_nodes[]` 涓庣湡瀹炶禌閬撲笉绗?| 涓?| 杩囨棭鎴栬繃鏅氬仠杞?| 浣跨敤 EDC 宸叉爣瀹氱殑 `{1,1,2,1}`锛屽悗缁疄杞﹁仈璋冨啀璋冩暣 |
| R7 | PID 鍙傛暟 (kp=18, ki=0, kd=10) 涓庝富宸ョ▼璧涢亾涓嶅尮閰?| 涓?| 宸＄嚎涓嶇ǔ鎴栭渿鑽?| 淇濈暀 EDC 榛樿鍙傛暟锛屽彲閫氳繃瀹忔垨 `pid_init()` 璋冪敤鍙傛暟璋冩暣 |
| R8 | 涓诲伐绋嬬嫭鏈夌殑澶栬 (buzzer/key/vision_uart/gimbal) 琚鍒?| 浣?| 鍔熻兘缂哄け | 杩佺Щ鏃跺彧鍔?chassis_iface.c/h銆乵ain.c銆乻yscfg锛屼笉纰板叾浠栨枃浠?|

### 5.2 鍥炴粴鐐?
| 鍥炴粴鐐?| 瑙﹀彂鏉′欢 | 鍥炴粴鎿嶄綔 |
|--------|---------|---------|
| RP1 鈥?syscfg 淇敼鍚?| 鐢熸垚鐨?`ti_msp_dl_config.h` 涓?GPIO_MOTOR 瀹忕己澶辨垨鍛藉悕涓嶅尮閰?| Git revert syscfg锛屽鐓?EDC syscfg 閲嶆柊鎵嬪姩娣诲姞 GPIO 瀹炰緥 |
| RP2 鈥?motor.c 缂栬瘧澶辫触 | `GPIO_MOTOR_*` 瀹忔湭瀹氫箟 | 妫€鏌?syscfg 鐢熸垚鐨勫畯鍚嶆槸鍚︿笌 motor.c 寮曠敤涓€鑷达紝涓嶄竴鑷村垯淇敼 syscfg $name |
| RP3 鈥?chassis_iface.c 鍚堝苟鍚庣紪璇戝け璐?| 绗﹀彿鍐茬獊銆佺己澶?include | 瀵规瘮澶囦唤鏂囦欢锛岄€愭淇 |
| RP4 鈥?杩愯鏃剁數鏈轰笉杞?| 鎺ョ嚎鎴?pin 閰嶇疆閿欒 | 鍥為€€鍒?EDC 宸ョ▼鍗曠嫭楠岃瘉纭欢锛岀‘璁ゆ纭?pin 鍚庡啀鍚堝苟 |
| RP5 鈥?app_state 鐘舵€佹満琛屼负寮傚父 | `chassis_get_status()` 杩斿洖鍊间笉绗﹀悎 stub 棰勬湡 | 鎭㈠ stub 涓殑 `chassis_get_status()` 瀹炵幇璺緞 |

### 5.3 寤鸿鐨勫垎姝ユ彁浜ょ瓥鐣?
```
Commit 1: 鏂板 motor.c/h + pid.c/h锛堟棤琛屼负鍙樺寲锛屼粎缂栬瘧閫氳繃锛?Commit 2: syscfg 鏂板 GPIO_MOTOR锛堥獙璇佸畯鐢熸垚姝ｇ‘锛?Commit 3: chassis_iface.c 鍚堝苟锛堜繚鐣?EDC chassis_state_t 鐘舵€佹満 + chassis_map_state_to_status() 鏄犲皠锛屽洖妞?stub 鍚堝悓淇濈暀椤癸級
Commit 4: main.c 鏀逛负 1ms 寰幆 + motor_update_pwm()
Commit 5: 鑱旇皟楠岃瘉鍚?tuning 鍙傛暟瀹?```

姣忎釜 commit 鍙嫭绔嬬紪璇戝拰鍥炴粴銆?
---

## 闄勫綍 A锛氭枃浠跺鐓ф竻鍗?
| 涓诲伐绋嬫枃浠?| 鍔ㄤ綔 | 鏉ユ簮 |
|-----------|------|------|
| `motor.h` | **鏂板** | EDC `motor.h` |
| `motor.c` | **鏂板** | EDC `motor.c` |
| `pid.h` | **鏂板** | EDC `pid.h` |
| `pid.c` | **鏂板** | EDC `pid.c` |
| `chassis_iface.h` | **鍚堝苟** | 涓诲伐绋嬬幇鏈?+ EDC 鎵╁睍鎺ュ彛 |
| `chassis_iface.c` | **鍚堝苟** | EDC 涓哄熀搴?+ 涓诲伐绋嬩繚鐣欓」 |
| `gpio_software_poll.syscfg` | **淇敼** | 鏂板 GPIO_MOTOR 瀹炰緥 |
| `main.c` | **淇敼** | 鏂板 motor_update_pwm() 璋冪敤 + 1ms 寰幆 |
| `app_state.c` | **涓嶅姩** | 鈥?|
| `app_common.h` | **涓嶅姩** | 鈥?|
| `gray_sensor.c/h` | **涓嶅姩** | 鈥?|
| `line_track.c/h` | **涓嶅姩** | 鈥?|
| `track_bridge.c/h` | **涓嶅姩** | 鈥?|
| `buzzer.c/h` | **涓嶅姩** | 鈥?|
| `mode_key.c/h` | **涓嶅姩** | 鈥?|
| `start_key.c/h` | **涓嶅姩** | 鈥?|
| `vision_uart.c/h` | **涓嶅姩** | 鈥?|
| `gimbal.c/h` | **涓嶅姩** | 鈥?|

## 闄勫綍 B锛氬畯閰嶇疆绗竴闃舵榛樿鍊?
```c
#define CHASSIS_USE_TRACK_BRIDGE           1    /* 濮嬬粓浣跨敤 bridge */
#define CHASSIS_TRACK_ERROR_SCALE          7    /* bridge error[-100,+100] 鈫?PID [-700,+700] */
#define CHASSIS_LOST_LINE_SHORT_TICKS      6    /* 60ms */
#define CHASSIS_LOST_LINE_LONG_TICKS       150  /* 1500ms */
#define CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD 6
#define CHASSIS_USE_ENCODER                0    /* 绗竴闃舵鍏抽棴 */
#define CHASSIS_BASE_SPEED                 22   /* 姝ｅ父宸＄嚎閫熷害 */
#define CHASSIS_MAX_SPEED                  45
#define CHASSIS_CURVE_SPEED                18   /* 寮亾閫熷害 */
#define CHASSIS_FIND_SPEED                 16   /* 鎵剧嚎鏃嬭浆閫熷害 */
#define CHASSIS_CROSS_CONFIRM              4    /* 璺彛杩炵画纭 tick 鏁?*/
#define PID_KP 18   /* 姣斾緥 */
#define PID_KI 0    /* 绉垎锛堢涓€闃舵涓?0锛?*/
#define PID_KD 10   /* 寰垎 */
```
