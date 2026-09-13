---
name: dummy-arm-ops
description: >-
  Dummy arm home-Linux operations: CDC serial exclusivity, eye-in-hand flange
  tracking (J6 axis through board center, flange plane 20 cm, no ArUco yaw), HUD,
  joint soft-box, and mandatory stow before power-off or any restart that drops
  motor torque (dry_run toggle, kill bridge, rebuild live). Use when
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
| `handeye` | `0, 0, 90, 0, 0, 0` | 跟随起始（J1/J2=0，J3=90，J4/J5 从 0° 起） |
| `handeye_calib` | `-8.7, 20, 90, 0, 60, 0` | 旧手眼标定起始 |
| `lookdown` | `0, -48, 125, 0, -80, 0` | 旧朝下候选 |

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
| **开始跟随** | 从当前姿态持续 LIVE 跟随 |
| **回初始** | 取消跟随，完整回到 `handeye` |
| **停止收起** | 取消跟随，收到折叠 `stow` |

回位走 `/cdc_servo_bridge/goto_handeye` 或 `goto_stow`（绕过 soft-box，走完后 hold Servo）。只有 `stow.sh` / 断电 / 滑条 GUI / `cdc_home_seven` 才停 bridge。

## 启动标定板跟随（必须先回 handeye）

起始位（FW）：**`[0, 0, 90, 0, 0, 0]`**（`scripts/home/poses/handeye_start.json`，J5 从 0° 起，跟随中不锁）。  
每次开 LIVE / 标定板跟踪，先独占串口回这个位，再开 bridge + tracker。

现阶段跟随：**画面中心对准板心，相机到板 20 cm**（`control_frame:=optical`）。不转姿态。J6 穿轴等光轴和 −Y 对齐后再开。  
手眼标定 `robot_effector_frame:=link5_1_1`，点动只用 J1–J5。

```bash
# 串口空闲后
python3 scripts/home/cdc_home_seven.py --preset handeye
python3 -u scripts/home/cdc_servo_bridge.py --ros-args -p joint_soft_half_range_deg:=0.0
# 另开（或直接: bash scripts/home/start_board_track.sh）
ros2 run dummy_vision aruco_servo_tracker_node --ros-args \
  -p dry_run:=false -p follow_orientation:=false \
  -p control_frame:=optical -p hold_current_distance:=false \
  -p desired_marker_in_ee_z:=0.20 \
  -p ws_around_current:=0.15 -p max_linear_vel:=0.05
python3 scripts/vision/cam_live_view.py
```

无正式标定时距离/对中是近似的；板尽量正对法兰。软盒 clip 时先 HUD **回初始**。

## 臂「看着该动却不动」

跟随：只平移对中，**锁 J4 和 J6**（Servo 会把 J4 当平移捷径，第一次 LIVE 无意义地拧了近 180°）。J5 从 0° 起可动，上限 75°。到边只夹该轴；J1 不再设 ±120 假墙。J3 不低于 20°。对准 |xy|<5mm、|z-20cm|<12mm。日志：`/tmp/dummy_track_ctrl/runs/follow_*.jsonl`。
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
