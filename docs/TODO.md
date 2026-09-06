# 当前进度

> 每次收工前更新此文件，方便换设备 / 新 Cursor session 快速恢复上下文。

## 进行中

- [ ] 把 CDC ASCII 接到 ROS：`dummy_servo_hardware` 改为发 `>j1..j6`（替代 Fibre `move_j`）
- [ ] 经济款 **J3 方向取反**：核对固件 / 线序 / ROS 方向补偿只在一处生效（见 `docs/ref固件与经济款DH笔记.md`）
- [ ] **眼在手上追踪（分阶段）** — 说明见 `docs/eye_in_hand_tracking.md`
  - [x] 视觉 dry-run：`aruco_detector` Rodrigues 姿态 + marker TF；`aruco_servo_tracker`；launch / 静态外参
  - [ ] 本地标定采集：`eye_in_hand_calibrate.launch.py` + easy_handeye2 Save `dummy_eih_calib`
  - [ ] CDC 后真机追踪：光轴/画面中心跟随（`control_frame:=optical`，已改代码；USB 掉线后待重插再测）
- [ ] 单位 Windows：Tailscale + Cursor Remote SSH
- [ ] 电源建议日常用 **12V ≥6A**（20V 能动但更抖）

## 已完成（续）

- [x] **根因**：DummyStudio 走 USB **CDC 串口 ASCII**，不是 Fibre bulk；Fibre `move_j` 只会抖不跟位
- [x] Linux 串口验证：`!START` → `#CMDMODE 2` → `#GETJPOS` → `>j…`，Joint2 真机跟随
- [x] 六轴拖动 GUI：`scripts/home/dummy_slider_gui.py`（已实测可拖）
- [x] 线索入库：`docs/windows_cdc_control.md` / `.json`
- [x] REF 固件 + 经济款 DH/J3 笔记：`docs/ref固件与经济款DH笔记.md`
- [x] Windows D 盘持久挂载脚本：`scripts/home/setup_windows_mounts.sh`（ntfs-3g）
- [x] D415 / ROS Jazzy / Fibre 发现 / 不自动 HOME 等（见前序提交）
- [x] 眼在手上视觉链路代码：`dummy_vision` detector / servo tracker / launch（默认 dry_run）

## 已完成

- [x] 仓库推送到 GitHub：`lixiaoxian38/dummyResearch`
- [x] 工作方式：家里 Linux Server + 单位 Windows Remote SSH + Tailscale
- [x] 本机克隆 `/home/lxx/Projects/dummyResearch`；git / ssh / Tailscale 就绪

## 备注

- 主机：`lxx01` / 用户 `lxx` / 路径 `/home/lxx/Projects/dummyResearch`
- 控制口：`/dev/ttyACM0`，`1209:0d32`，115200，协议见 `docs/windows_cdc_control.md`
- 「7」字复位：`0, -73, 180, 0, 0, 0`
- **手眼标定起始位 `handeye`（FW）**：`[-8.7, 20, 90, 0, 60, 0]` — `python3 scripts/home/cdc_home_seven.py --preset handeye`；见 `scripts/home/poses/handeye_start.json`
- **收起 `stow`（上电位）**：`[0, -75, 180, 0, 0, 0]` — 断电或任何会脱力的重启前先 `bash scripts/home/stow.sh`
- **启动跟随先回 `handeye`**：`[-8.7, 20, 90, 0, 60, 0]` — `bash scripts/home/start_board_track.sh`
- GUI：`python3 scripts/home/dummy_slider_gui.py`
- 挂载：D=`/mnt/win_data`，C=`/mnt/windows`（需 fstab + ntfs-3g）
- 结构件：**经济款**；J3 方向见 `docs/ref固件与经济款DH笔记.md`
- 眼在手上：`ros2 launch dummy_vision eye_in_hand_track.launch.py dry_run:=true`
