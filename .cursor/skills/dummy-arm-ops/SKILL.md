---
name: dummy-arm-ops
description: >-
  Dummy arm home-Linux operations: CDC serial exclusivity, eye-in-hand optical
  tracking, HUD aim, joint soft-box, and mandatory stow before power-off or any
  restart that drops motor torque (dry_run toggle, kill bridge, rebuild live).
  Use when starting/stopping tracking, dry_run, calibrating, jogging, shutting
  down, 断电, 收工, 收起, 重启, or when the arm will not follow the board.
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
| `handeye` | `-8.7, 20, 90, 0, 60, 0` | 手眼标定/跟随起始 |
| `lookdown` | `0, -48, 125, 0, -80, 0` | 旧朝下候选 |

```bash
# 串口必须空闲
python3 scripts/home/cdc_home_seven.py --preset handeye
python3 scripts/home/cdc_home_seven.py --preset stow
```

## 串口独占

同时只能有一个写 `/dev/ttyACM0`：`cdc_servo_bridge`、滑条 GUI、`cdc_home_seven`、Fibre HW。Fibre 和 CDC 不要同口。

HUD **回初始 / 停止收起 / 开始跟随** 只停 tracker，**不要杀 bridge**。回位走 `/cdc_servo_bridge/goto_handeye` 或 `goto_stow`（绕过 soft-box，到点后重新居中）。只有 `stow.sh` / 断电 / 滑条 GUI / `cdc_home_seven` 才停 bridge。

## 启动标定板跟随（必须先回 handeye）

起始位（FW）：**`[-8.7, 20, 90, 0, 60, 0]`**（`scripts/home/poses/handeye_start.json`）。  
每次开 LIVE / 标定板跟踪，先独占串口回这个位，再开 bridge + tracker。

```bash
# 串口空闲后
python3 scripts/home/cdc_home_seven.py --preset handeye
python3 -u scripts/home/cdc_servo_bridge.py --ros-args -p joint_soft_half_range_deg:=25.0
# 另开
ros2 run dummy_vision aruco_servo_tracker_node --ros-args \
  -p dry_run:=false -p follow_orientation:=false \
  -p control_frame:=optical -p hold_current_distance:=true \
  -p ws_around_current:=0.08
python3 scripts/vision/cam_live_view.py   # AIM=光轴主点，绿点=板心
```

HUD 绿点到 AIM ≈ 0 px 才算跟到中心。`|err|` 是末端笛卡尔误差，不是像素误差。

## 臂「看着该动却不动」

1. 看 bridge 是否 `soft_box clip`：盒子中心是启动时（或上次 goto 后）的关节。HUD **回初始** 会经 bridge 回 handeye 并 recenter，不要杀进程。
2. Servo 是否 pause / `code≠0`。
3. tracker 是否 `BOARD LOST`（HUD 本地能看见但 ROS TF 可能丢）。

## 安全

软限位只是软件。手离急停。不要对 CDC 狂发指令。USB 掉线先等 `/dev/ttyACM0` 回来再写串口。
