# Phase 1 杩佺Щ缁撴灉 鈥?涓诲伐绋?
> 杩佺Щ鏃ユ湡锛?026-05-07
> 瀹炴満楠屾敹鏃ユ湡锛?026-05-07
> 鐘舵€侊細Phase 1 瀹屾垚锛屽疄鏈洪獙鏀堕€氳繃
> 鏉ユ簮宸ョ▼锛欵DC-SHEN-Car锛圚ardware A 搴曠洏锛?> 鐩爣宸ョ▼锛歡pio_software_poll_LP_MSPM0G3507_nortos_ticlang
> 绾︽潫锛氬彧鍋氭枃妗ｈ褰曪紝涓嶄慨鏀逛换浣?.c/.h 鎺у埗閫昏緫銆?
## 1. 宸插畬鎴愮殑 4 涓縼绉绘楠?
### Commit 1 鈥?鏂板 motor.c/h + pid.c/h

| 鏂囦欢 | 鏉ユ簮 | 璇存槑 |
|------|------|------|
| `motor.c` | EDC-SHEN-Car | 鍥涜矾 TB6612 椹卞姩銆佽蒋 PWM 璋冨埗銆佹柟鍚戞帶鍒躲€佸埞杞?婊戣/閿佹 |
| `motor.h` | EDC-SHEN-Car | motor 鎺ュ彛澹版槑 |
| `pid.c` | EDC-SHEN-Car | 澧為噺寮?PID锛屽甫绉垎闄愬箙鍜岃緭鍑洪檺骞?|
| `pid.h` | EDC-SHEN-Car | `pid_t` 缁撴瀯浣撳拰鎺ュ彛澹版槑 |
| `Debug/makefile` | 淇敼 | ORDERED_OBJS 鍜?clean 鍔犲叆 `pid.o` |

缁撴灉锛歱id.c 缂栬瘧閫氳繃銆俶otor.c 寰?Commit 2 syscfg 鏇存柊鍚庢柟鍙紪璇戙€?
### Commit 2 鈥?syscfg 鏂板 GPIO_MOTOR

| 鏂囦欢 | 璇存槑 |
|------|------|
| `gpio_software_poll.syscfg` | 鏂板 GPIO5 瀹炰緥 `GPIO_MOTOR`锛?3 涓?output pin |
| `Debug/ti_msp_dl_config.h` | sysconfig CLI 閲嶆柊鐢熸垚锛屾柊澧?39 涓?`GPIO_MOTOR_*` 瀹?|
| `Debug/ti_msp_dl_config.c` | sysconfig CLI 閲嶆柊鐢熸垚锛屾柊澧?`SYSCFG_DL_GPIO_MOTOR_init()` |
| `Debug/makefile` | ORDERED_OBJS 鍜?clean 鍔犲叆 `motor.o` |

GPIO_MOTOR pin 娓呭崟锛?
```
AIN1=PA8   AIN2=PA13  BIN1=PB9   BIN2=PB19  STBY=PA15
PWMA=PA28  PWMB=PB13  CIN1=PB17  CIN2=PB18  DIN1=PB24
DIN2=PA24  PWMC=PB4   PWMD=PB20
```

鏈慨鏀圭殑鐜版湁澶栬锛欱UZZER(PB15), MODE_KEY(PA27), START_KEY(PA25), GRAY_SENSOR(PB0-PB3), DEBUG_UART(PA10-PA11), VISION_UART(PB6-PB7)銆?
鏆備笉鏂板锛欸PIO_ENCODER锛圥hase 2锛夈€?
缁撴灉锛歮otor.c 缂栬瘧閫氳繃銆傚叏宸ョ▼ 14 涓簮鏂囦欢缂栬瘧 + 閾炬帴閫氳繃銆?
### Commit 3 鈥?chassis_iface.c 鐪熷疄搴曠洏閫昏緫

| 鏂囦欢 | 璇存槑 |
|------|------|
| `chassis_iface.c` | 浠?EDC chassis_iface.c 涓哄熀搴曢噸鍐欙紙~420 琛岋級锛屾帴鍏ョ湡瀹炲簳鐩樻帶鍒?|
| `chassis_iface.h` | 鏂板 encoder stub 澹版槑鍜?`chassis_get_state()` 澹版槑 |

杩佸叆鐨?EDC 閫昏緫锛?
- `chassis_state_t` 鍐呴儴鐘舵€佹満锛? 鐘舵€侊細LOCKED/IDLE/LINE/FIND_LINE/CROSS/FINISHED/ERROR/LINE_LOST锛?- `chassis_map_state_to_status()` 鏄犲皠鍒板悎鍚?`ChassisStatus_t`锛? 鐘舵€侊級
- `chassis_read_line_input()` 浠?bridge 璺緞锛坄track_bridge_get()`锛夛紝鏃?legacy fallback
- stale 鏃犳潯浠舵渶楂樹紭鍏堢骇 鈫?CHASSIS_ERROR 鈫?CHASSIS_STATUS_ERROR
- 涓㈢嚎鐭椂 6 tick 鈫?FIND_LINE锛堟棆杞壘绾匡級锛岄暱鏃?150 tick 鈫?LINE_LOST锛堝埞杞︿繚鎶わ級
- 璺彛杩炵画纭 4 tick + active_count >= 6 鈫?鑺傜偣璁℃暟 鈫?杈惧埌 target 鈫?RCHD
- PID (`pid_t` + `pid_update()`) + 宸€熸帶鍒?(`motor_set_speed`)
- `motor_brake` / `motor_lock` / `motor_unlock` / `motor_coast` 鎺ュ叆
- `chassis_debug_log()` 浠呯姸鎬佸彉鍖栨椂杈撳嚭 `"CH:"` 琛?
鎺掗櫎鐨勯€昏緫锛?
- encoder 鍏ㄩ儴浠ｇ爜锛坄CHASSIS_USE_ENCODER == 0`锛屽畯宸插叧闂級
- `GROUP1_IRQHandler()`锛堜緷璧?GPIO_ENCODER 绠¤剼锛孭hase 2锛?- legacy track fallback锛坄track.h` / `track_read()`锛屼富宸ョ▼濮嬬粓浣跨敤 bridge锛?- `abs_int`锛堜粎 encoder 浣跨敤锛?
淇濈暀鐨勪富宸ョ▼鍚堝悓锛?
- `chassis_set_target(int target)` 澶栭儴绛惧悕涓嶅彉锛屽唴閮?clamp `int` 鈫?`chassis_target_t`
- `chassis_follow_target()` TARGET_NONE 鈫?ERROR 妫€鏌ワ紙app_state 渚濊禆锛?- `chassis_debug_simulate_line_lost()` / `chassis_debug_simulate_error()` 鍙摼鎺?
**chassis_get_status() 瀹屾暣鏄犲皠琛?*锛?
| 鍐呴儴 `chassis_state_t` | 鍚堝悓 `ChassisStatus_t` | `C=` |
|------------------------|------------------------|------|
| LOCKED / IDLE | IDLE | IDL |
| FIND_LINE | FINDING_LINE | FLIN |
| LINE / CROSS | FOLLOWING | FOLL |
| FINISHED | TARGET_REACHED | RCHD |
| ERROR | ERROR | ERR |
| LINE_LOST | LINE_LOST | LOST |

鏈慨鏀规枃浠讹細app_state.c銆乵ain.c銆乬ray_sensor.c/h銆乴ine_track.c/h銆乼rack_bridge.c/h銆乻yscfg銆?
缁撴灉锛?3 婧愭枃浠剁紪璇?+ 閾炬帴閫氳繃锛? errors, 0 new warnings锛夈€?
### Commit 4 鈥?main.c 璋冨害

| 鏂囦欢 | 璇存槑 |
|------|------|
| `main.c` | 寰幆浠?10ms 鏀逛负 1ms锛屾柊澧?`motor_update_pwm()` 璋冪敤鍜?10ms 鍒嗛鍣?|

璋冨害缁撴瀯锛?
```
while (1) {
    motor_update_pwm();                 鈫?[姣?1ms] 杞?PWM 10 姝ョ浉浣嶆帹杩?
    start_key_scan();                   鈫?[姣?1ms] 鍘熸湁璇箟涓嶅彉

    tick_10ms_cnt++;
    if (tick_10ms_cnt >= 10) {         鈫?鍒嗛鍣?脳10
        tick_10ms_cnt = 0;
        app_state_tick();               鈫?[姣?10ms] 鐘舵€佹満鏃堕棿璇箟涓嶅彉
    }

    debug_cnt++;
    if (debug_cnt >= 500) {            鈫?500 脳 1ms = 500ms
        debug_cnt = 0;
        /* S=/M=/T=/C=/G=/L=/B= 鍏ㄩ噺蹇収 */
    }

    delay_cycles(CPUCLK_FREQ / 1000);  鈫?1ms
}
```

**TICK_MS=10 浠嶈〃绀?app_state_tick 鐨勯€昏緫鍛ㄦ湡**銆俶ain loop 鐜板湪鏄?1ms锛屼絾 `app_state_tick()` 璋冪敤棰戠巼鐢?`TICK_10MS_DIVIDER=10` 鍒嗛鍣ㄤ繚璇佷粛涓?10ms 涓€娆°€俙app_state_tick()` 鍐呴儴浣跨敤鐨?`state_ticks`銆乣RUN_TIMEOUT_TICKS`銆乣SCAN_STEP_INTERVAL` 绛夊潎鍩轰簬 `TICK_MS=10` 璁＄畻锛屾椂闂磋涔変繚鎸佷笉鍙樸€?
缁撴灉锛氱紪璇?+ 閾炬帴閫氳繃锛? errors, 0 warnings锛夈€?
---

## 2. 褰撳墠鏋勫缓缁撴灉

```
缂栬瘧鍣細TI ARM Clang 4.0.4.LTS (tiarmclang)
鐩爣锛? MSPM0G3507, Cortex-M0+, Thumb
浼樺寲锛? -O0 (debug)

婧愭枃浠?(14 涓?锛?  app_state.c        OK
  buzzer.c           OK
  chassis_iface.c    OK  (Phase 1 merged)
  gimbal.c           OK
  ti_msp_dl_config.c OK  (syscfg generated)
  startup_mspm0g350x_ticlang.c OK
  gray_sensor.c      OK
  line_track.c       OK  (1 pre-existing warning: array-parameter)
  main.c             OK  (Phase 1 adapted)
  mode_key.c         OK
  motor.c            OK  (from EDC)
  pid.c              OK  (from EDC)
  start_key.c        OK
  track_bridge.c     OK
  vision_uart.c      OK

璀﹀憡锛? pre-existing 鈥?line_track.c array-parameter mismatch (declaration vs definition)
閿欒锛?

閾炬帴锛歡pio_software_poll_LP_MSPM0G3507_nortos_ticlang.out 鈫?175,376 bytes
```

---

## 3. 褰撳墠杩愯鑳藉姏

### 3.1 鏁版嵁娴?
```
main.c [1ms]
  鈹斺攢 motor_update_pwm()                    鈫?PWM 鐩镐綅鎺ㄨ繘

main.c [10ms 鍒嗛]
  鈹斺攢 app_state_tick()
       鈹斺攢 handle_running()                 鈫?RUNNING 鐘舵€?            鈹溾攢 gray_sensor_read_all(gv)    鈫?鐏板害浼犳劅鍣紙涓诲伐绋嬪師閫昏緫锛?            鈹溾攢 line_track_compute(gv, &lt) 鈫?瀵荤嚎璁＄畻锛堜富宸ョ▼鍘熼€昏緫锛?            鈹溾攢 track_bridge_update(&lt, tick) 鈫?鏇存柊妗ユ帴锛坰tale 娓呴櫎锛?            鈹溾攢 chassis_tick()              鈫?搴曠洏 tick锛圥hase 1 杩佸叆锛?            鈹?   鈹斺攢 chassis_run_line()
            鈹?        鈹溾攢 chassis_read_line_input() 鈫?track_bridge_get()
            鈹?        鈹溾攢 stale? 鈫?ERR (鏃犳潯浠舵渶楂樹紭鍏堢骇)
            鈹?        鈹溾攢 FIND_LINE? 鈫?鏃嬭浆鎵剧嚎
            鈹?        鈹溾攢 涓㈢嚎? 鈫?鐭椂 FLIN / 闀挎椂 LOST
            鈹?        鈹溾攢 鑺傜偣? 鈫?CROSS 鈫?node++ 鈫?RCHD
            鈹?        鈹斺攢 PID(line.error) 鈫?motor_set_speed(L,R)
            鈹斺攢 chassis_get_status()        鈫?杞鐘舵€?```

### 3.2 宸插叿澶囩殑鑳藉姏

| 鑳藉姏 | 鐘舵€?|
|------|------|
| `app_state` RUNNING 鏇存柊 `track_bridge` | 涓诲伐绋嬪師閫昏緫锛屾湭淇敼 |
| `chassis_tick()` 娑堣垂 `track_bridge_get()` | Phase 1 杩佸叆 |
| `chassis_get_status()` 杩斿洖 FOLL | 姝ｅ父宸＄嚎涓?|
| `chassis_get_status()` 杩斿洖 FLIN | 鐭椂涓㈢嚎 6 tick锛?0ms锛?|
| `chassis_get_status()` 杩斿洖 LOST | 闀挎椂涓㈢嚎 150 tick锛?500ms锛?|
| `chassis_get_status()` 杩斿洖 ERR | stale 鈫?ERROR锛堥潪 RUNNING 鐘舵€侊級 |
| `chassis_get_status()` 杩斿洖 RCHD | 鑺傜偣鏁拌揪鏍?|
| `motor_update_pwm()` 1ms 鍏ュ彛 | 杞?PWM 10 姝ョ浉浣嶆帹杩?|
| 鍥涜矾 TB6612 椹卞姩 | `motor_set_speed(L,R)` 鈫?`motor_set_wheels()` |
| PID 寰抗 | `pid_update(&line_pid, error)` + 宸€?|
| 涓诲姩鍒硅溅 | `motor_brake()` |
| 搴曠洏閿佹/瑙ｉ攣 | `motor_lock()` / `motor_unlock()` |
| 鍘熷湴鏃嬭浆鎵剧嚎 | 渚濇嵁 `last_line_error` 鏂瑰悜 |
| encoder 闂幆 | 鍏抽棴锛坄CHASSIS_USE_ENCODER=0`锛?|

### 3.3 鏆備笉鍏峰鐨勮兘鍔涳紙Phase 2+锛?
| 鑳藉姏 | 鍘熷洜 |
|------|------|
| encoder 闂幆 | `CHASSIS_USE_ENCODER=0`锛孏PIO_ENCODER 鏈厤缃?|
| encoder 涓柇澶勭悊 | `GROUP1_IRQHandler` 鏈縼鍏?|
| legacy track fallback | 涓诲伐绋嬪缁堜娇鐢?bridge锛屼笉闇€瑕?|

---

## 4. 鐑у綍鍚庨獙鏀舵楠?
### 4.1 涓婄數涓插彛妫€鏌?
鐑у綍鍚庝笂鐢碉紝涓插彛锛圖EBUG_UART, UART0, 9600 baud锛夊簲杈撳嚭锛?
```
S=IDLE M=DRV T=- C=IDL G=X1:0 X2:0 ... X8:0 L=--- B=STALE
```

楠岃瘉锛?- `S=IDLE` 鈥?涓绘帶鐘舵€佺┖闂?- `M=DRV` 鈥?榛樿鍩烘湰寰抗妯″紡
- `T=-` 鈥?鏃犵洰鏍?- `C=IDL` 鈥?搴曠洏 IDLE
- `G=` 鈥?8 璺伆搴﹁鏁颁负 0/1
- `L=---` 鈥?line_track 鏃犳娴?- `B=STALE` 鈥?bridge 鏈洿鏂帮紙闈?RUNNING 鐘舵€侊級

### 4.2 鐏板害浣嶅簭楠岃瘉

鐢ㄤ竴涓粦鑳跺甫/榛戠焊閬尅鍗曚釜鎺㈠ご锛岃瀵熶覆鍙?`G=` 琛岋細
- 閬尅 X1锛堟渶宸︿晶鎺㈠ご锛夆啋 `G=X1:1 X2:0 ... X8:0`
- 閬尅 X8锛堟渶鍙充晶鎺㈠ご锛夆啋 `G=X1:0 X2:0 ... X8:1`

纭浣嶅簭锛歑1=宸? X8=鍙? active=1 琛ㄧず榛戠嚎銆?
### 4.3 DRV 妯″紡 RUN 鐘舵€?FOLL 楠岃瘉

1. 鍒囧埌 DRV 妯″紡锛坄M=DRV`锛?2. 闀挎寜 START_KEY (PA25) ~3s
3. 鍚埌 3 澹拌渹楦?鈫?`S=SBEEP`
4. 铚傞福缁撴潫 鈫?`S=RUN M=DRV T=C C=FLIN`锛堣繘鍏ユ壘绾匡級
5. 灏嗗皬杞︽斁鍦ㄩ粦绾夸笂 鈫?搴曠洏鏃嬭浆鐩村埌鎵惧埌绾?鈫?`C=FOLL`
6. `L=` / `B=` 闅忛粦绾夸綅缃彉鍖栵紝error 姝ｈ礋涓庢柟鍚戜竴鑷?7. 楠岃瘉 `pwm` 璁℃暟鍣ㄦ寔缁闀?
### 4.4 鍏ㄧ櫧涓㈢嚎 FLIN 鈫?LOST 楠岃瘉

1. 灏忚溅姝ｅ父 `C=FOLL` 宸＄嚎涓?2. 灏嗗皬杞︽姮绂婚粦绾匡紙鎵€鏈夋帰澶磋鐧斤級
3. 瑙傚療涓插彛锛?   - 6 tick 鍐咃紙绾?60ms锛夛細`C=FLIN`锛堝簳鐩樻棆杞級
   - 150 tick 鍚庯紙绾?1500ms锛夛細`C=LOST`锛堝埞杞︼級
4. `S=ERR` 鈥?app_state 妫€娴嬪埌 LOST 鍚庤繘鍏ラ敊璇姸鎬?
### 4.5 鑺傜偣 RCHD 楠岃瘉

1. 灏忚溅宸＄嚎涓粡杩囧崄瀛楄矾鍙ｏ紙鎴栧榛戠嚎鍖哄煙锛?2. `active_count >= 6` 鎸佺画 4 tick 鈫?鑺傜偣璇嗗埆
3. `node` 璁℃暟澧炲姞
4. 褰撳墠 Target C 闇€瑕?2 涓妭鐐癸紙`target_finish_nodes[CHASSIS_TARGET_C] = 2`锛?5. 绗?2 涓妭鐐瑰悗 鈫?`C=RCHD` 鈫?`S=TSTOP` 鈫?瀹屾垚铚傞福 鈫?`S=IDLE`

### 4.6 stale 鈫?ERR 浜哄伐娉ㄥ叆楠岃瘉

1. 灏忚溅姝ｅ父宸＄嚎 `S=RUN C=FOLL`
2. 涓存椂娉ㄩ噴 `app_state.c` 绗?140 琛岀殑 `track_bridge_update(&lt, state_ticks);`
3. 閲嶆柊缂栬瘧鐑у綍
4. 杩涘叆 RUNNING 鍚庯紝涓嬩釜 tick 鐨?`B=STALE`
5. `C=ERR`锛堝埞杞︼級
6. `S=ERR`锛坅pp_state 妫€娴嬪埌 ERROR锛?7. 鎭㈠浠ｇ爜鍚庨噸鏂扮紪璇?
### 4.7 鐢垫満鍔ㄤ綔鍏堣楠岃瘉锛堝缓璁湪寰抗鍓嶅畬鎴愶級

濡傛灉搴曠洏棣栨涓婄數锛屽缓璁厛鐢ㄦ渶灏忎唬鐮侀獙璇佺數鏈烘柟鍚戯細

- 鍓嶈繘锛氬乏鍙宠疆鍚屽悜鍚岄€燂紝杞﹀墠杩?- 宸︽棆锛氬乏杞悗閫€銆佸彸杞墠杩涳紝杞﹂€嗘椂閽堟棆杞?- 鍙虫棆锛氬乏杞墠杩涖€佸彸杞悗閫€锛岃溅椤烘椂閽堟棆杞?- 鍒硅溅锛氱數鏈虹珛鍗冲仠姝?
鑻ユ柟鍚戠浉鍙嶏紝妫€鏌?syscfg 涓?`MOTOR_A_POLARITY` 绛夊畯瀹氫箟锛屾垨淇敼 motor.c 涓殑鏋佹€у父閲忋€?
---

## 5. 瀹氭椂璇箟璇存槑

| 鍙傛暟 | 鍊?| 鍚箟 |
|------|-----|------|
| `TICK_MS` | 10 | `app_state_tick()` 璋冪敤鍛ㄦ湡锛坅pp_common.h 瀹氫箟锛屾湭淇敼锛?|
| main loop 鍛ㄦ湡 | 1ms | `delay_cycles(CPUCLK_FREQ / 1000)` |
| `motor_update_pwm()` | 1ms | PWM 10 姝ョ浉浣嶆帹杩?|
| `start_key_scan()` | 1ms | 鎸夐敭鎵弿锛堝師涓?10ms锛?|
| `app_state_tick()` | 10ms | 閫氳繃 `TICK_10MS_DIVIDER=10` 鍒嗛淇濊瘉 |
| 璋冭瘯杈撳嚭 | 500ms | `DEBUG_INTERVAL_MS=500`锛?ms 脳 500锛?|
| 涓㈢嚎鐭椂闃堝€?| 60ms | `CHASSIS_LOST_LINE_SHORT_TICKS=6` 脳 10ms per tick |
| 涓㈢嚎闀挎椂闃堝€?| 1500ms | `CHASSIS_LOST_LINE_LONG_TICKS=150` 脳 10ms per tick |
| 璺彛纭 | 40ms | `CHASSIS_CROSS_CONFIRM=4` 脳 10ms per tick |

**鍏抽敭**锛歚app_state_tick()` 鍙婂叾鍐呴儴 `state_ticks` 浠嶄互 10ms 涓哄崟浣嶃€俙RUN_TIMEOUT_TICKS = 120 脳 TICKS_PER_SEC` 鐨勮秴鏃惰绠椾笉鍙樸€備涪绾垮拰鑺傜偣闃堝€煎熀浜?`chassis_tick()` 鐨勮皟鐢ㄦ鏁帮紙姣?10ms 涓€娆★級锛屾椂闂磋涔変笉鍙樸€?
---

## 6. 褰撳墠鏂囦欢瀵圭収

| 鏂囦欢 | 鐘舵€?| 鏉ユ簮 |
|------|------|------|
| `motor.c` | 鏂板 | EDC-SHEN-Car |
| `motor.h` | 鏂板 | EDC-SHEN-Car |
| `pid.c` | 鏂板 | EDC-SHEN-Car |
| `pid.h` | 鏂板 | EDC-SHEN-Car |
| `chassis_iface.c` | 閲嶅啓锛堝悎骞讹級 | EDC 涓哄熀搴?+ 涓诲伐绋嬪悎鍚岄€傞厤 |
| `chassis_iface.h` | 淇敼锛堟柊澧炲０鏄庯級 | 涓诲伐绋嬬幇鏈?+ encoder stub 澹版槑 |
| `gpio_software_poll.syscfg` | 淇敼锛堟柊澧?GPIO5锛?| 鎵嬪姩鏂板 GPIO_MOTOR 瀹炰緥 |
| `main.c` | 淇敼锛?ms 寰幆锛?| 鏂板 motor_update_pwm + 鍒嗛鍣?|
| `Debug/makefile` | 淇敼 | 鏂板 motor.o, pid.o |
| `Debug/ti_msp_dl_config.h` | 閲嶆柊鐢熸垚 | sysconfig CLI |
| `Debug/ti_msp_dl_config.c` | 閲嶆柊鐢熸垚 | sysconfig CLI |
| `app_state.c` | 淇敼锛? 澶勶級 | DRV 妯″紡 RUN 鍏ュ彛缁熶竴涓?`chassis_follow_target(target)` |
| `app_common.h` | 鏈慨鏀?| 鈥?|
| `gray_sensor.c/h` | 鏈慨鏀?| 鈥?|
| `line_track.c/h` | 鏈慨鏀?| 鈥?|
| `track_bridge.c/h` | 鏈慨鏀?| 鈥?|
| `buzzer.c/h` | 鏈慨鏀?| 鈥?|
| `mode_key.c/h` | 鏈慨鏀?| 鈥?|
| `start_key.c/h` | 鏈慨鏀?| 鈥?|
| `vision_uart.c/h` | 鏈慨鏀?| 鈥?|
| `gimbal.c/h` | 鏈慨鏀?| 鈥?|

---

## 7. 瀹炴満楠屾敹缁撴灉

### 7.1 Phase 1 鏈€缁堢姸鎬?
| 椤圭洰 | 鐘舵€?|
|------|------|
| 涓诲伐绋嬬湡瀹炲簳鐩?Phase 1 杩佺Щ | 瀹屾垚 |
| 鏋勫缓锛坱iarmclang 缂栬瘧 + 閾炬帴锛?| 閫氳繃锛? errors |
| 瀹炴満楠屾敹 | 閫氳繃 |
| `track_bridge_update()` | 宸叉仮澶嶏紙`app_state.c:140` 姝ｅ父璋冪敤锛?|
| stale 浜哄伐娉ㄥ叆鐗堟湰 | 鏈繚鐣欙紙楠岃瘉鍚庡凡鎭㈠鍘熶唬鐮侊級 |
| encoder | 鏈惎鐢紙`CHASSIS_USE_ENCODER=0`锛?|

### 7.2 宸查獙璇佺姸鎬侀摼璺?
浠ヤ笅鍧囧湪瀹炴満涓婇€氳繃 UART 涓插彛锛圖EBUG_UART, 9600 baud锛夐€愰」楠岃瘉銆?
| 鍦烘櫙 | 涓插彛鐜拌薄 | app_state 鍚庣画 |
|------|---------|---------------|
| **IDLE / SBEEP** | `B=STALE` 鈥?bridge 鏈洿鏂帮紝绗﹀悎鍚堝悓锛堥潪 RUNNING 鐘舵€?bridge 姘镐负 stale锛?| 鈥?|
| **RUN 鏈夌嚎** | `C=FOLL`锛宍B=E:+xxx` 鎴?`B=E:-xxx` 鈥?bridge 杈撳嚭瀹炴椂璇樊锛屽簳鐩樺惊绾垮墠杩?| 姝ｅ父鎸佺画 |
| **RUN 鐭椂涓㈢嚎** | 鎶榛戠嚎 6 tick 鍐?鈫?`C=FLIN` 鈥?搴曠洏鍘熷湴鏃嬭浆鎵剧嚎 | 鎸佺画 FLIN 鐩村埌鎵惧埌绾挎垨瓒呮椂 |
| **RUN 闀挎椂涓㈢嚎** | 鎸佺画鍏ㄧ櫧 150 tick 鈫?`C=LOST` 鈥?搴曠洏鍒硅溅 | `app_state` 妫€娴嬪埌 LOST 鈫?`chassis_stop()` 鈫?`chassis_lock()` 鈫?`S=ERR` |
| **RUN 鑺傜偣杈惧埌** | 缁忚繃 2 涓矾鍙?鈫?`C=RCHD` 鈥?搴曠洏鍒硅溅 | `app_state` 妫€娴嬪埌 RCHD 鈫?`S=TSTOP` 鈫?瀹屾垚铚傞福 鈫?`S=IDLE` |
| **RUN stale 娉ㄥ叆** | 涓存椂娉ㄩ噴 `track_bridge_update()` 鈫?`CH: st=ERR ext=ERR stale=1` | `app_state` 妫€娴嬪埌 ERR 鈫?`chassis_stop()` 鈫?`chassis_lock()` 鈫?`S=ERR` |

**stale 娉ㄥ叆鍚庣殑 CH: 琛岀ず渚?*锛?```
CH: st=ERR ext=ERR err=0 line=0 white=1 active=0 stale=1 lost=0 node=0 target=2 lock=0
```

### 7.3 淇璁板綍

**闂**锛欴RV 妯″紡浠?`handle_start_beep()` 杩涘叆 RUNNING 鏃惰皟鐢?`chassis_find_line()`锛岃鍑芥暟涓嶆竻闆?`node_count` / `cross_count` / `cross_latched` / `lost_count`銆傚鏋滄鍓嶅簳鐩樺浜庡叾浠栫姸鎬佷笖璁℃暟鍣ㄩ潪闆讹紝浼氭硠婕忓埌鏂颁竴娆¤繍琛屻€?
**淇**锛圼app_state.c:120-123](app_state.c#L120-L123)锛夛細

```c
// 淇敼鍓?if (run_mode == RUN_MODE_BASIC_DRIVE) {
    chassis_find_line();         // 涓嶉噸缃鏁板櫒
} else {
    chassis_follow_target(target); // 閲嶇疆璁℃暟鍣?}

// 淇敼鍚?chassis_follow_target(target);   // DRV / NAV / ADV 缁熶竴璺緞
```

**鍘熷洜**锛歚chassis_follow_target()` 鍐呮竻闆跺叏閮ㄨ鏁板櫒 + 璁剧疆 `chassis_needs_pid_reset=1`锛屼繚璇佹瘡娆¤繘鍏?RUNNING 鏃惰鏁板櫒浠庨浂寮€濮嬨€?
**楠岃瘉缁撴灉**锛欴RV 妯″紡杩涘叆 RUNNING 鍚庯紝UART 杈撳嚭 `node=0`銆傜粡杩囩 2 涓矾鍙ｅ悗 `node=2` 瑙﹀彂 `C=RCHD`銆傚娆￠噸澶嶆祴璇曪紝RCHD 瑙﹀彂涓€鑷淬€?
### 7.4 stale 浜哄伐娉ㄥ叆鏂规硶涓庢仮澶?
**娉ㄥ叆鏂规硶**锛?1. 涓存椂娉ㄩ噴 `app_state.c` 涓?`handle_running()` 鐨勪互涓嬭锛?   ```c
   // track_bridge_update(&lt, state_ticks);  /* 涓存椂娉ㄩ噴 */
   ```
2. 閲嶆柊缂栬瘧鐑у綍銆?3. 杩涘叆 DRV 妯″紡锛屽惎鍔ㄨ渹楦ｇ粨鏉熷悗杩涘叆 RUNNING銆?4. 棣栦釜 `chassis_tick()` 鍐?`chassis_read_line_input()` 璇诲彇 bridge锛宍stale=1`銆?5. `chassis_run_line()` 绗竴浼樺厛绾?stale 妫€鏌?鈫?`motor_brake()` + `chassis_state = CHASSIS_ERROR`銆?6. `chassis_get_status()` 杩斿洖 `CHASSIS_STATUS_ERROR`銆?7. `handle_running()` 妫€娴嬪埌 ERR 鈫?`chassis_stop()` 鈫?`chassis_lock()` 鈫?`enter_state(APP_STATE_ERROR)`銆?8. 涓插彛杈撳嚭锛歚CH: st=ERR ext=ERR stale=1`锛岄殢鍚?`S=ERR`銆?
**鎭㈠**锛氬彇娑堟敞閲婏紝鎭㈠ `track_bridge_update(&lt, state_ticks);`锛岄噸鏂扮紪璇戙€傚綋鍓嶄富宸ョ▼浠ｇ爜涓琛屽凡鎭㈠锛?*涓嶄繚鐣欐敞閲婄増鏈?*銆?
### 7.5 涓嬩竴闃舵寤鸿

Phase 1 鎺ュ彛鍚堝悓楠岃瘉鍏ㄩ儴閫氳繃銆傚悗缁笉鍐嶄慨鏀规帴鍙ｅ悎鍚岋紝杩涘叆瀹炶溅杩愬姩璋冨弬闃舵銆?
**璋冨弬椤哄簭寤鸿**锛?
1. **鏋剁┖楠岃瘉鍥涜疆鏂瑰悜**
   - 鏋剁┖灏忚溅锛岀敤鏈€灏忔祴璇曚唬鐮佸崟鐙┍鍔ㄦ瘡涓疆瀛?   - 纭 A/B/C/D 鍥涗釜 TB6612 閫氶亾鏂瑰悜涓庣墿鐞嗚疆瀛愪竴鑷?   - 鑻ユ柟鍚戠浉鍙嶏紝璋冩暣 `motor.c` 涓?`MOTOR_A_POLARITY` 绛?4 涓瀬鎬у畯

2. **浣庨€熷湴闈㈤獙璇佸墠杩?鏃嬭浆鏂瑰悜**
   - 浣庨璋冪敤 `chassis_forward(low_speed)`锛岀‘璁よ溅鍓嶈繘
   - 浣庨璋冪敤 `chassis_rotate_left/right(low_speed)`锛岀‘璁ゆ棆杞柟鍚戞纭?   - error 姝ｈ礋涓庤浆鍚戝叧绯婚獙璇侊紙绾垮亸鍙?鈫?error>0 鈫?杞﹀簲鍙宠浆锛?
3. **璋?CHASSIS_BASE_SPEED / CHASSIS_FIND_SPEED**
   - `CHASSIS_BASE_SPEED`锛堝綋鍓?22锛夛細姝ｅ父宸＄嚎閫熷害锛屽お浣庝笉璧般€佸お楂樺啿鍑?   - `CHASSIS_FIND_SPEED`锛堝綋鍓?16锛夛細鎵剧嚎鏃嬭浆閫熷害锛屽お鎱㈡壘绾挎參銆佸お蹇槗杩囧啿
   - `CHASSIS_CURVE_SPEED`锛堝綋鍓?18锛夛細寮亾閫熷害

4. **璋?PID 鍙傛暟**
   - 褰撳墠榛樿锛歬p=18, ki=0, kd=10锛堜粎 PD锛岀Н鍒嗛」涓?0锛?   - 鍏堣皟 kp锛氬贰绾挎椂宸﹀彸鎽嗗姩骞呭害澶?鈫?鍑忓皬锛涘搷搴旀參 鈫?澧炲ぇ
   - 鍐嶈皟 kd锛氭姂鍒堕渿鑽?   - 鏈€鍚庤皟 ki锛堝闇€娑堥櫎绋虫€佽宸級

5. **鑺傜偣闃堝€?*
   - `CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD`锛堝綋鍓?6锛夛細鍦ㄧ湡瀹炶禌閬撲笂棣栨缁忚繃璺彛鏃惰瀵?active_count锛屾寜闇€璋冩暣
   - `target_finish_nodes[]`锛堝綋鍓?`{1,1,2,1}`锛夛細鎸夎禌閬撳疄闄呰妭鐐规暟鏍囧畾

6. **Encoder 闂幆锛圥hase 2锛?*
   - 鍓嶆彁锛氬洓杞?encoder 纭欢鎺ョ嚎纭锛宻yscfg 鏂板 GPIO_ENCODER 瀹炰緥
   - 鎵撳紑 `CHASSIS_USE_ENCODER=1`
   - 杩佸叆 `GROUP1_IRQHandler` 鍜?encoder 閫熷害闂幆閫昏緫

**涓嶅啀淇敼鐨勬帴鍙ｅ悎鍚?*锛?- `chassis_iface.h` 8 涓悎鍚?API 绛惧悕涓嶅彉
- `app_state` 8 鐘舵€佹満涓嶅彉
- `track_bridge` 7 瀛楁鍚堝悓涓嶅彉
- gray_sensor 鈫?line_track 鈫?track_bridge 鈫?chassis 鏁版嵁娴佹柟鍚戜笉鍙?