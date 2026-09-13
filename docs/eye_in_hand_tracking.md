# 眼在手上：标定 + ArUco 末端追踪

> 本期默认 **dry_run**（只发目标位姿 / TF，不驱动真机）。CDC 接好后再关 dry_run。

## 依赖（家里 Linux 一次性）

```bash
source /opt/ros/jazzy/setup.bash   # 按本机 ROS 发行版改
sudo apt install \
  ros-${ROS_DISTRO}-realsense2-camera \
  ros-${ROS_DISTRO}-easy-handeye2 \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-tf-transformations

cd ~/Projects/dummyResearch/dummy_moveit_ws
colcon build --packages-select dummy_vision
source install/setup.bash
```

标定板：打印 **A4 PDF**（与 ROS 默认参数一致）：

```bash
# 已生成在仓库内；需要重新生成时：
python3 scripts/vision/generate_aruco_a4_pdf.py
# 输出: scripts/vision/aruco_markers/aruco_original_id0_50mm_a4.pdf
```

- 字典 **DICT_ARUCO_ORIGINAL**，id **0**，黑色方块边长 **50 mm**
- 打印选 **100% / 实际大小**，用尺子量黑色方块必须为 5 cm
- ROS 参数：`marker_length:=0.05`，`marker_id:=0`

## 远程 / 视觉 dry-run

相机插好后：

```bash
cd ~/Projects/dummyResearch/dummy_moveit_ws
source install/setup.bash
ros2 launch dummy_vision eye_in_hand_track.launch.py \
  dry_run:=true start_servo:=false start_realsense:=true
```

无相机、只测手眼静态 TF + tracker 参数时：

```bash
ros2 launch dummy_vision eye_in_hand_publish.launch.py publish_optical_stub:=true
# 另开终端跑 detector / tracker，并自己提供图像话题
```

验收：

- `ros2 topic echo /aruco_camera_pose` 有位姿
- `ros2 run tf2_ros tf2_echo camera_color_optical_frame camera_marker`
- `ros2 topic echo /tracking/desired_ee_pose` 随板移动/倾斜变化
- RViz 看 `desired_ee` TF

## 回家：手眼标定（eye_in_hand）

**先打印标定板：** [`scripts/vision/aruco_markers/aruco_original_id0_50mm_a4.pdf`](../scripts/vision/aruco_markers/aruco_original_id0_50mm_a4.pdf)（100% 比例，黑色方块 50mm）

**起始关节位（FW 度，已固化）：** `handeye` = `[-8.7, 20, 90, 0, 60, 0]`  
见 [`scripts/home/poses/handeye_start.json`](../scripts/home/poses/handeye_start.json)。标定前：

```bash
# 串口需空闲（先关 cdc_servo_bridge / 滑条 GUI）
python3 scripts/home/cdc_home_seven.py --preset handeye
```

一键（推荐，脚本内会先回 handeye）：

```bash
bash scripts/home/calibrate_eye_in_hand.sh
# 已在目标位可跳过回零: SKIP_HOME=1 bash scripts/home/calibrate_eye_in_hand.sh
python3 scripts/vision/check_calibration_ready.py   # 可选：检查 TF / 话题
```

手动步骤：

1. 回 handeye 起始位（上），再启动机械臂 TF（MoveIt / `servo_streaming` 或 `demo`），保证 `base_link`、`link5_1_1`（相机所在，J5 外壳）在 TF 树中。相机不随 J6 转，标定 `robot_effector_frame` 用 `link5_1_1`，不要用法兰 `link6_1_1`。点动只动 J1–J5。
2. 标定板固定在桌上，相机在腕部，保证多数姿态能看见板。
3. 启动：

```bash
ros2 launch dummy_vision eye_in_hand_calibrate.launch.py
```

4. 在 easy_handeye2 界面多次 **Take Sample**（臂姿尽量分散）→ **Compute** → **Save**  
   文件：`~/.ros2/easy_handeye2/calibrations/dummy_eih_calib.calib`
5. 之后用正式外参：

```bash
ros2 launch dummy_vision eye_in_hand_track.launch.py \
  use_easy_handeye:=true dry_run:=true
```

开发阶段可用占位外参：[`dummy_moveit_ws/dummy_vision/config/handeye_static.yaml`](../dummy_moveit_ws/dummy_vision/config/handeye_static.yaml)（**不能**当正式抓取用）。

## 回家：真机跟随（需 CDC 已进 ROS）

```bash
# 终端 1：MoveIt Servo + 硬件桥
ros2 launch dummy_moveit_config servo_streaming.launch.py
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger

# 终端 2：视觉追踪（关 dry_run）
ros2 launch dummy_vision eye_in_hand_track.launch.py \
  dry_run:=false start_servo:=false use_easy_handeye:=true \
  follow_orientation:=true
```

或一条命令带上 Servo：

```bash
ros2 launch dummy_vision eye_in_hand_track.launch.py \
  dry_run:=false start_servo:=true use_easy_handeye:=true
```

安全建议：先 `follow_orientation:=false` 只跟位置；确认方向后再开姿态。手离急停近一点。

## 关键参数（tracker）

| 参数 | 默认 | 含义 |
|---|---|---|
| `dry_run` | true | 不发 `/servo_node/delta_twist_cmds` |
| `control_frame` | optical | 现阶段：`optical`＝HUD 画面中心对板心、相机距板 20 cm。`ee`＝J6 轴穿板心（光轴对齐后再用） |
| `hold_current_distance` | false | true 时（多用于 optical）只纠 XY、锁当前深度 |
| `desired_marker_in_ee_z` | 0.20 | 法兰平面到板的目标距离 (m)，默认 20 cm |
| `ws_around_current` | 0.15 | 期望 EE 相对当前位的软半径 (m) |
| `follow_orientation` | true | 法兰平面与板平行（只齐法向，不跟 ArUco yaw）。HUD LIVE 暂关，避免转 30° 丢板 |
| `replan_period_sec` | 2.0 | 看清后再锁一个目标；到点或到期才重规划，避免每帧换路 |
| `keep_in_view_xy` | 0.10 | 板偏出此半径则停（不反向回中） |
| `trace_dir` | `/tmp/dummy_track_ctrl/runs` | 每次跟随一份 JSONL：目标、关节、Servo 解、轴偏差 |
| `max_linear_vel` | 0.05 | 线速度上限 (m/s) |
| `max_angular_vel` | 0.25 | 角速度上限 (rad/s) |
| `ws_*` | 见节点 | 期望 EE 软工作空间 |

## 节点一览

| 节点 | 作用 |
|---|---|
| `aruco_detector_node` | PnP + 发布 `camera_marker` TF / 位姿话题 |
| `aruco_servo_tracker_node` | 相对位姿误差 → Servo Twist（或 dry_run） |
| `aruco_tracker_node` | **已废弃**（旧点到点 MoveIt） |

## 与 CDC 的关系

真机连续跟位依赖 `dummy_servo_hardware` 发 CDC `>j…`。当前仍可能是 Fibre；**未接好前请保持 `dry_run:=true`**。
