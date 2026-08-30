# 当前进度

> 每次收工前更新此文件，方便换设备 / 新 Cursor session 快速恢复上下文。

## 进行中

- [ ] 把 CDC ASCII 接到 ROS：`dummy_servo_hardware` 改为发 `>j1..j6`（替代 Fibre `move_j`）
- [ ] 经济款 **J3 方向取反**：核对固件 / 线序 / ROS 方向补偿只在一处生效（见 `docs/ref固件与经济款DH笔记.md`）
- [ ] 眼在手上 TF + ArUco 跟随闭环
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

## 已完成

- [x] 仓库推送到 GitHub：`lixiaoxian38/dummyResearch`
- [x] 工作方式：家里 Linux Server + 单位 Windows Remote SSH + Tailscale
- [x] 本机克隆 `/home/lxx/Projects/dummyResearch`；git / ssh / Tailscale 就绪

## 备注

- 主机：`lxx01` / 用户 `lxx` / 路径 `/home/lxx/Projects/dummyResearch`
- 控制口：`/dev/ttyACM0`，`1209:0d32`，115200，协议见 `docs/windows_cdc_control.md`
- 「7」字复位：`0, -73, 180, 0, 0, 0`
- GUI：`python3 scripts/home/dummy_slider_gui.py`
- 挂载：D=`/mnt/win_data`，C=`/mnt/windows`（需 fstab + ntfs-3g）
- 结构件：**经济款**；J3 方向见 `docs/ref固件与经济款DH笔记.md`
