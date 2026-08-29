# 当前进度

> 每次收工前更新此文件，方便换设备 / 新 Cursor session 快速恢复上下文。

## 进行中

- [ ] `move_j` / Servo 键盘 jog 仍「抖动但不跟位」——明天用 Windows DummyStudio 对比
- [ ] 电源建议改回 **12V ≥6A**（20V 能动但更抖；`get_voltage()`≈3.3 仅为逻辑电参考）
- [ ] 眼在手上 TF + ArUco 跟随闭环
- [ ] 单位 Windows：Tailscale + Cursor Remote SSH

## 已完成（续）

- [x] D415 连接 + RealSense SDK；ArUco demo 成功
- [x] 安装 ROS 2 Jazzy + colcon；Servo 冒烟启动
- [x] Dummy 口 `1209:0d32`；跳过 USB `Device.reset()` 后 Fibre 稳定
- [x] 连接时仅 `set_enable`、**不自动 HOME**；可读六轴角度
- [x] 串口 `!START` 成功；Fibre `homing()` 真机可展开成「7」字
- [x] 键盘 jog topic 对齐 `/servo_node/...`；`env.sh` 兼容 `set -u`
- [x] 脚本：`keyboard_jog.sh`、`arm_start_and_nudge.sh`、udev 规则草稿

## 已完成

- [x] 仓库推送到 GitHub：`lixiaoxian38/dummyResearch`
- [x] 工作方式：家里 Linux Server + 单位 Windows Remote SSH + Tailscale
- [x] 本机克隆 `/home/lxx/Projects/dummyResearch`；git / ssh / Tailscale 就绪
- [x] GitHub SSH remote 可 push

## 备注

- 主机：`lxx01` / 用户 `lxx` / 路径 `/home/lxx/Projects/dummyResearch`
- Tailscale IP：`100.75.232.68`；LAN：`192.168.31.172`
- ROS：**Jazzy**（文档里常写 Humble）
- Dummy：Fibre `usb`；串口 `!START` 需 `dialout`（或 sudo）；ACM=`/dev/ttyACM0`
- 已知：多开 launch/RViz 会污染 `/joint_states`；勿整段 `sudo` 跑 Fibre（缺 pyusb）
