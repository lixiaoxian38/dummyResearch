---
name: dummy-arm-ops
description: >-
  Dummy arm home-Linux operations: CDC serial exclusivity, eye-in-hand
  tracking (optical center + 20 cm, optional orientation with J4 free / J6 locked),
  HUD, joint soft-box, and mandatory stow before power-off or any restart that
  drops motor torque (dry_run toggle, kill bridge, rebuild live). Use when
  starting/stopping tracking, dry_run, calibrating, jogging, shutting down, 断电,
  收工, 收起, 重启, or when the arm will not follow the board.
---

# Dummy 臂操作（家里 Linux）

重活只在家里跑。单位笔记本只 SSH。控制走 USB CDC `/dev/ttyACM0`（`1209:0d32`），不要和 Fibre `dummy_servo_hardware` 抢口。

## 断电 / 会脱力的重启（必须先收起）

上电后的收起位（FW 度）：**`[0, -75, 180, 0, 0, 0]`**（`scripts/home/poses/stow.json`）。

杀 CDC bridge、改/重启 dry_run、重拉 Servo/tracker、`stop.sh`、断电都会让电机**瞬间脱力**。先把臂收到这个位，再动进程：

```bash
bash scripts/home/stow.sh
```

`stow.sh`：pause Servo → 停 tracker / bridge / 滑条 GUI → 独占串口回 stow。

- 跳过：`SKIP_STOW=1 bash scripts/home/stop.sh`（仅当臂已经在 stow）
- 没有 `/dev/ttyACM0`：等 USB，**不要在伸出姿态下杀 bridge**

## 姿态

| 名字 | FW 度 | 用途 |
|---|---|---|
| `stow` | `0, -75, 180, 0, 0, 0` | 断电前收起 |
| `handeye` | `0, 0, 90, 0, 0, 0` | 标定板跟随起始 |
| `charger` | `0, -40, 120, 0, +70, 0` | 充电头跟随起始（朝桌面；本机 J5 正角低头） |
| `handeye_calib` | `-8.7, 20, 90, 0, 60, 0` | 旧手眼标定起始 |
| `lookdown` | `0, -48, 125, 0, -80, 0` | 更俯的朝下候选 |

```bash
# 串口必须空闲
python3 scripts/home/cdc_home_seven.py --preset handeye
python3 scripts/home/cdc_home_seven.py --preset stow
```

## 串口独占

同时只能有一个写 `/dev/ttyACM0`：`cdc_servo_bridge`、滑条 GUI、`cdc_home_seven`、Fibre HW。Fibre 和 CDC 不要同口。

HUD 三个按钮（只停 tracker，**不要杀 bridge**）：

| 按钮 | 行为 |
|---|---|
| **开始跟随** | LIVE 跟随（离当前源起始位远时先自动回：板=`handeye`，充电头=`charger`；关闭六轴拖动） |
| **回初始** | 取消跟随，回到当前源起始位（板 handeye / 充电头朝桌面） |
| **停止收起** | 取消跟随，收到折叠 `stow` |
| **六轴拖动** | 暂停跟随，打开滑条（经 `/cdc_servo_bridge/jog_fw`，不占串口）；再点关闭 |

回位走 `/cdc_servo_bridge/goto_handeye` 或 `goto_stow`（绕过 soft-box，走完后 hold Servo）。只有 `stow.sh` / 断电 / 滑条 GUI / `cdc_home_seven` 才停 bridge。

## 桌面一键（Tracking Hub）

双击桌面 **Dummy 跟随控制台**，或 `bash scripts/home/start_tracking_hub.sh`。  
按需拉起 ROS2 + MoveIt Servo（无 Fibre）+ D415 + CDC + 检测 + HUD。**不自动 LIVE、不 git pull、不起 Stream API。** 图标丢失时：`bash scripts/home/install_desktop_launcher.sh`。

## 启动跟随（按源回对应起始位）

| 源 | 起始 FW | 文件 |
|---|---|---|
| 标定板 | `[0, 0, 90, 0, 0, 0]` | `poses/handeye_start.json` |
| 充电头 | `[0, -40, 120, 0, +70, 0]` | `poses/charger_start.json`（J5=+70° 朝桌面） |
| VLM | `[3, 0, 90, 0, +75, 0]` | `poses/vlm_start.json`（当前拖到位取整；开始跟随 / 回初始 / 换指令都回这里） |

每次开 LIVE，离起始位远时会先经 bridge `goto_handeye` / `goto_charger` 再跟。

现阶段跟随：**喷枪瞄准**（板 20 cm / 充电头 10 cm）+ **Hold / CDC Quiet**。运动由 **WDLS** → Servo JointJog；J4 自由、J6 锁。  
HUD：`标定板 | 充电头`。  
参数：`dummy_vision/config/follow_live.yaml` + `follow_session.yaml`。定案：`docs/跟随优化定案.md`。  
总线：`/tracking/target_pose` + `/tracking/target_quality`。  
手眼标定 `robot_effector_frame:=link5_1_1`（相机在 J5 壳体）。

```bash
# 推荐：桌面 Tracking Hub，或
bash scripts/home/start_tracking_hub.sh
# 窗口内：点「开始跟随」（折叠态会自动先回初始）
```

HUD 芯片：CAL=手眼TF · DET=板 · TGT=目标新鲜 · SPRAY=喷枪对准(Hold) · CHG=充电头 · MOV=运动 · RUN=跟随 · SRV=Servo。  
**是否对准**：青色 AIM 准心 ↔ 绿色板中心点；`aim vs board: 0px` 且 SPRAY 亮、偏心/Δz≈0 即到位。`/tracking/status` 为 JSON。

## 臂「看着该动却不动」

跟随：只平移对中，**锁 J4 和 J6**（Servo 会把 J4 当平移捷径，第一次 LIVE 无意义地拧了近 180°）。J5 从 0° 起可动，−85°…75°。到边只夹该轴；J1 不再设 ±120 假墙。J3 不低于 20°。对准 |xy|<8mm、|z-20cm|<15mm。日志：`/tmp/dummy_track_ctrl/runs/follow_*.jsonl`。
2. Servo 是否 pause / `code≠0`。
3. tracker 是否 `BOARD LOST`（HUD 本地能看见但 ROS TF 可能丢）。

## 「Servo 冲突 / 臂挂了只能重启」

两层问题，不要混：

1. **ROS 层打架（软件能挡住）**  
   MoveIt Servo 还在发 `/servo_node/command`，同时 goto/收起也在写串口。J6 会被残余指令拉开。  
   处理：pause Servo → 停 tracker → bridge `hold_servo`。HUD 回初始/收起已经这么做。**不要为了收起杀 bridge。**

2. **USB CDC 卡死（Write timeout）**  
   `/dev/ttyACM*` 还在，但谁写都超时。杀 ROS、重启 Servo **没用**。电机没指令就脱力，按钮「没反应」。  
   **不必重启电脑。** 拔插机械臂 USB（或 `bash scripts/home/cdc_recover.sh`），再拉起**唯一**一个 `cdc_servo_bridge`。  
   触发：杀 bridge 后立刻重开串口、两个进程抢口、对 CDC `flush()`/tcdrain。

## 安全

软限位只是软件。手离急停。不要对 CDC 狂发指令。USB 掉线先等 `/dev/ttyACM*` 回来再写串口。
