# 当前进度

> 每次收工前更新此文件，方便换设备 / 新 Cursor session 快速恢复上下文。

## 进行中

- [ ] 把 CDC ASCII 接到 ROS：`dummy_servo_hardware` 改为发 `>j1..j6`（替代 Fibre `move_j`）
- [ ] 经济款 **J3 方向取反**：核对固件 / 线序 / ROS 方向补偿只在一处生效（见 `docs/ref固件与经济款DH笔记.md`）
- [ ] **眼在手上追踪（分阶段）** — 说明见 `docs/eye_in_hand_tracking.md`
  - [x] 视觉 dry-run：`aruco_detector` + `aruco_servo_tracker` + launch
  - [x] 本地标定：easy_handeye2 Save（`link5_1_1→camera_link`，约 1.9/−4.8/1.3 cm）
  - [x] LIVE 光轴跟随：画面中心对板 + 相机距板 **20 cm**；参数统一 `config/follow_live.yaml`
  - [x] 操作台 HUD：`cam_live_view.py`（CAL/DET/TGT/MOV/RUN/SRV + `/tracking/status`）
  - [x] HUD 集成六轴拖动：`pause` + `/cdc_servo_bridge/jog_fw` 滑条（不占第二串口）
  - [x] TargetPose 总线：`/tracking/target_pose` + `/tracking/target_quality`（mux 选板/螺母）
  - [x] HUD 开关：标定板|螺母 × 只中心|中心+J4（须先停跟再切；写入 `follow_session.yaml`）
  - [x] quality 门控 0.4、速度×quality、丢检 coast 0.4 s；去掉光轴 4 cm slide
  - [x] 螺母检测：深度质心 + 六角轮廓，HUD 洋红十字；选「螺母」后接到 tracker
  - [ ] 真机验收：到位不绕圈（近距降增益）；J5 可俯到 −85° 而不是钉在 −70 用 J3 补；螺母十字稳定后再 LIVE；限速暂不改
  - [ ] 实验：`control_frame:=ee`（J6 轴对板心 / 法兰 20 cm）— 待光轴与 −Y 对齐后再开
- [ ] 单位 Windows：Tailscale + Cursor Remote SSH
- [ ] 电源建议日常用 **12V ≥6A**（20V 能动但更抖）

## 已完成（续）

- [x] CDC ASCII 根因与六轴 GUI / `cdc_servo_bridge`
- [x] 眼在手上视觉链路 + A4 标定板 PDF
- [x] 跟随起始 `handeye`=`[0,0,90,0,0,0]`；收起 `stow`；断电先 stow
- [x] LIVE 启动自动发布 easy_handeye（缺文件则 static 占位）

## 备注

- 主机：`lxx01` / 路径 `/home/lxx/Projects/dummyResearch`
- 控制口：`/dev/ttyACM0`，协议见 `docs/windows_cdc_control.md`
- **跟随**：桌面「Dummy 跟随控制台」或 `bash scripts/home/start_tracking_hub.sh`（再点 HUD「开始跟随」）
- **参数**：`dummy_moveit_ws/dummy_vision/config/follow_live.yaml`
- **手眼**：`~/.ros2/easy_handeye2/calibrations/dummy_eih_calib.calib`（effector=`link5_1_1`）
- **收起**：`bash scripts/home/stow.sh`（断电 / 杀 bridge 前必须）
