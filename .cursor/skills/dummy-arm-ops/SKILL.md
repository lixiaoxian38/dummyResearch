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
| **开始跟随** | 从当前姿态持续 LIVE 跟随（会关闭六轴拖动） |
| **回初始** | 取消跟随，完整回到 `handeye` |
| **停止收起** | 取消跟随，收到折叠 `stow` |
| **六轴拖动** | 暂停跟随，打开滑条（经 `/cdc_servo_bridge/jog_fw`，不占串口）；再点关闭 |

回位走 `/cdc_servo_bridge/goto_handeye` 或 `goto_stow`（绕过 soft-box，走完后 hold Servo）。只有 `stow.sh` / 断电 / 滑条 GUI / `cdc_home_seven` 才停 bridge。

## 桌面一键（Tracking Hub）

双击桌面 **Dummy 跟随控制台**，或 `bash scripts/home/start_tracking_hub.sh`。  
按需拉起 ROS2 + MoveIt Servo（无 Fibre）+ D415 + CDC + 检测 + HUD。**不自动 LIVE、不 git pull、不起 Stream API。** 图标丢失时：`bash scripts/home/install_desktop_launcher.sh`。

## 启动标定板跟随（必须先回 handeye）

起始位（FW）：**`[0, 0, 90, 0, 0, 0]`**（`scripts/home/poses/handeye_start.json`，J5 从 0° 起，跟随中不锁）。  
每次开 LIVE / 标定板跟踪，先独占串口回这个位，再开 bridge + tracker。

现阶段跟随：**画面中心对准目标，相机到目标 20 cm**（`control_frame:=optical`）。靠近目标会把线速度按误差缩小，避免绕圈。J5 跟随下限 −85°（俯仰应由腕部承担，不要用 J1 绕圈 / J3 折肘硬顶 −70°）。  
HUD 两个开关（**跟随中须先停再切**）：目标 `标定板 | 螺母`，运动 `只中心 | 中心+J4`（后者放开 J4、锁 J6）。默认只中心。  
参数：`dummy_vision/config/follow_live.yaml` + HUD 写入的 `follow_session.yaml`。  
总线：`/tracking/target_pose` + `/tracking/target_quality`。到位 HUD **SPRAY**。  
手眼标定 `robot_effector_frame:=link5_1_1`（相机在 J5 壳体）；Servo EE 仍是 `link6_1_1`。

```bash
# 串口空闲后
python3 scripts/home/cdc_home_seven.py --preset handeye
python3 -u scripts/home/cdc_servo_bridge.py --ros-args -p joint_soft_half_range_deg:=0.0
# 另开（或直接: bash scripts/home/start_board_track.sh）
ros2 run dummy_vision aruco_servo_tracker_node --ros-args \
  -r __node:=aruco_servo_tracker \
  --params-file dummy_moveit_ws/dummy_vision/config/follow_live.yaml
python3 scripts/vision/cam_live_view.py
```

HUD 芯片：CAL=手眼TF · DET=板 · TGT=到位 · SPRAY=喷枪对准 · NUT=螺母 · MOV=运动 · RUN=跟随 · SRV=Servo。  
绿点到 AIM ≈ 0 且 Δz≈0 即到位。`/tracking/status` 为 JSON 状态。

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
