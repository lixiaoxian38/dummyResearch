# REF 固件 / CDC·Fibre / 经济款 DH 笔记

> 资料来源（本机 Windows D 盘挂载）：  
> - `/mnt/win_data/dummy/Dummy相关资料/ref_42_35固件/`  
> - `/mnt/win_data/dummy/Dummy相关资料/模型的更改处/`（**必读 `说明.txt`**）  
> - 源码包：`dummy-ref-core-fw-20260222.zip`（已抽读 `ascii_protocol` / `dummy_robot` / `6dof_kinematic` / `interface_usb`）  
> - 上位机实测协议：`docs/windows_cdc_control.md`  
>
> **本机型号：经济款。** DH 杆长可与任同学版共用仿真；**J3 方向必须取反**（见 §5）。

---

## 1. 「刷板」到底刷什么

不是把「转到多少度」烧进 Flash。日常角度/速度仍是运行时 USB 命令。

| 板子 | 对应文件 | 职责 |
| --- | --- | --- |
| **REF 主控**（STM32F4，USB 接到电脑） | `Core-STM32F4-fw-*.bin` | 运动学、命令解析、USB（CDC+Fibre）、经 CAN 指挥各关节 |
| **Ctrl-Step 驱动板**（每个关节附近） | `Ctrl-Step-STM32-fw*.bin` | 单关节电机闭环；REF 用 CAN 下发目标 |

源码里电机控制环、限位、FK/IK、`MoveJ`、ASCII / Fibre 协议都在 REF 固件中。  
**手眼 / VLA 不必先刷板**；只有换结构件版本、加夹爪减速比、厂商要求升级时才换对应 `.bin`。

---

## 2. 同一份固件里同时有 Fibre 和 CDC

`interface_usb.cpp` 里两路端点并存：

- **CDC**（`CDC_OUT_EP`）→ 文本 ASCII（DummyStudio / 本仓库滑条 GUI）
- **Native / ODrive 风格 bulk**（`ODRIVE_OUT_EP`）→ Fibre 对象树（仓库 `ref_tool` / `move_j`）

二者最终都进同一个 `DummyRobot::MoveJ()`。因此：

- **不是「硬件不支持 Fibre」**，板子两种协议都实现了；
- Linux 上 Fibre `move_j` 只抖不跟，更像是 **传输口冲突 / 命令模式 / 主机侧用法**，不是没刷带 Fibre 的固件；
- **实时拖动、手眼、VLA 底层控制：走 CDC ASCII**（已验证）。

---

## 3. CDC ASCII 协议（与 Studio / 本仓库 GUI 一致）

口：`/dev/ttyACM*`，VID:PID `1209:0d32`，**115200** 8N1。完整实测流程见 `docs/windows_cdc_control.md`。

本仓库已验证顺序（`scripts/home/dummy_slider_gui.py`）：

1. `!START` → `Started ok`（使能）
2. `#CMDMODE 2`（可打断，滑条才能拖）
3. `#GETJPOS` 读六轴角度（度）
4. `>j1,j2,j3,j4,j5,j6`（可带速度）实时目标

源码 `UserApp/protocols/ascii_protocol.cpp` 摘要：

| 前缀 | 作用 |
| --- | --- |
| `!` | 使能/急停/回零等：`!START` / `!STOP` / `!HOME` / `!RESET` / `!DISABLE`；夹爪 `!HAND_*`（视固件） |
| `#` | 查询/模式：`#GETJPOS`、`#GETLPOS`、`#CMDMODE n`、电机 PID/加减速等 |
| `>` / `&` | 关节目标入队（**度**）：`>j1,j2,j3,j4,j5,j6[,speed]` |
| `@` | 笛卡尔 `MoveL`（需固件 IK） |

### 命令模式 `#CMDMODE`（关键）

| 值 | 含义 | 拖滑条 |
| --- | --- | --- |
| **1** | 顺序到点，等到位再接下一条 | 不适合实时拖 |
| **2** | **可打断到点**（Studio / GUI 用这个） | **必须设 2** |
| **3** | 连续轨迹（加速度更高、速度比降低） | 流式/轨迹 |
| **4** | 电机调参 | 调试用 |

「7」字姿态（源码 `REST_POSE`）：约 `0, -75, 180, 0, 0, 0`（实测常用 `0, -73, 180, 0, 0, 0`）。

---

## 4. Fibre 在固件里暴露的对象（摘要）

`dummy_robot.h` 的协议定义里有 `move_j`、`set_enable`、`homing`、`set_rgb_mode`、各 `joint_n` 等。  
与 ASCII 的 `>` 一样底层调 `MoveJ`；**本机验证以 CDC 为准，Fibre 不当主控制。**

---

## 5. 经济款结构件与 DH / J3（对本机最重要）

路径：`/mnt/win_data/dummy/Dummy相关资料/模型的更改处/`

### `说明.txt` 原文要点（经济款）

1. **J2–J3**：由「弯」改「直」；**DH 各关节长度完全不变**；**仅 J3 方向取反**  
   - 改法 A：改 DH 表里 J3 方向  
   - 改法 B：直接调 J3 **电机线序**（等效反转）
2. **J5 / J6**：结构加固/分体降本；与减速器、相邻件相对位姿不变 → **DH 不变**
3. **仿真**：可直接用 **任同学版模型**（几何/杆长一致）

经济款 STEP：`模型的更改处/经济款结构件/`

| 文件 | 含义（从命名推断） |
| --- | --- |
| `J23.step` / `J23盖.step` | J2–J3 直联结构（相对弯联的改动点） |
| `J4反.step` | J4 相关结构 |
| `J5.step` / `J6.step` | 腕部结构改动（DH 长度仍不变） |
| `435支架-J6.step` | J6/末端支架 |

任同学对照：`模型的更改处/任同学结构件/`（`J5.step`、`J6.step`）。

### 固件目录 `说明文档.txt` ↔ 盘上文件名

目录：`/mnt/win_data/dummy/Dummy相关资料/ref_42_35固件/`

| 盘上文件（以 `/mnt/win_data/.../ref_42_35固件/` 为准） | 说明文档含义 |
| --- | --- |
| `dummy-ref-core-fw.rar` / `dummy-ref-core-fw-20260222.zip` | REF 源码；文档写 **DH 可能需要改 J3 方向** |
| `Core-STM32F4-fw-inversej3.bin` / `Core-STM32F4-fw_inverse3_v3.bin` | **小宅版**结构件默认（带 inverse J3） |
| `Core-STM32F4-fw_S70.bin` / `Core-STM32F4-fw-S65-V3.bin` | **任同学版**结构件默认 |
| `Core-STM32F4-fw-RTX-V3.bin` | 含夹爪；文档称 J6 额外减速比等 |
| `Ctrl-Step-STM32-fw.bin` / `Ctrl-Step-STM32-fw2.bin` | 驱动板默认 |

> 说明文件名是 `说明文档.txt`（与「模型的更改处/说明.txt」不是同一个文件）。

源码包 `dummy_robot.cpp` 中连杆长度构造（单位 m，以包内为准）：

```text
DOF6Kinematic(0.134, 0.035, 0.146, 0.117, 0.052, 0.0625)
#           L_BASE D_BASE L_ARM L_FOREARM D_ELBOW L_WRIST
```

各轴还有 `CtrlStepMotor(..., inverse, reduction, ...)` 方向标志。  
**经济款相对任同学/默认树：杆长不变，差的是 J3 正方向**——不要理解成「整臂换一套 DH 长度」。

### 对本仓库 ROS / MoveIt 的含义

- URDF 目前按一套轴方向建树；经济款对齐真机时，**优先查 Joint3 `axis` 符号或驱动层方向补偿数组第 3 项**。
- 现有硬件层已有类似 `rad_direct_diff = [1,1,1,1,-1,-1]` 及 J3 偏置——那是 **URDF↔固件历史约定**，**不等于已经做完「经济款 J3 取反」**。
- 接手时用真机 `#GETJPOS` + 正运动学/相机核验 J3 符号；**固件 / 线序 / ROS 三处只留一处取反**，避免双重取反。
- 官方说仿真可用任同学模型 → MoveIt 碰撞/外观可先借用；**控制符号以经济款说明为准单独改。**

---

## 6. 任务选型（手眼 / VLA）

| 层级 | 选择 | 原因 |
| --- | --- | --- |
| 研究栈 | ROS2 + MoveIt + LeRobot / stream API | TF、规划、采集 |
| 底层关节 | **CDC `>` + `#CMDMODE 2`** | 与固件实时路径、Studio 一致 |
| Fibre | 仅遗留/调试 | 本机跟位不可靠 |
| 刷板 | 非默认路径 | 先确认现网固件是否已 inverse J3 |

下一步工程：把 `dummy_servo_hardware` 的 Fibre `move_j` 换成 CDC ASCII，并在桥接层显式处理经济款 **J3 方向**。

---

## 7. 安全与操作注意（资料 `reademe.txt`）

- 文档冲突时 **以厂商文档为准**。
- 有「REF→J1 线序接反会烧」一类警告——改线序/刷固件前对照官方接线，**不要为了反 J3 改错 J1**。
- 电源：日常建议 **12V ≥6A**（20V 能动但更抖）。

---

## 8. 本机快速对照表

| 项 | 值 |
| --- | --- |
| 结构件 | **经济款**（J23 直联，J3 方向取反） |
| D 盘资料 | `/mnt/win_data/dummy/Dummy相关资料/` |
| 控制口 | `/dev/ttyACM0`，`1209:0d32`，115200 |
| 推荐协议 | CDC ASCII（`docs/windows_cdc_control.md`） |
| 滑条 GUI | `python3 scripts/home/dummy_slider_gui.py` |
| 源码包 | `ref_42_35固件/dummy-ref-core-fw-20260222.zip` |

---

## 9. 待核实清单

- [ ] 当前板子实际烧的是哪个 `Core-*.bin`（是否已含 inverse J3）
- [ ] 经济款 J3 取反落在：固件 / 线序 / ROS 方向补偿 **仅一处**
- [ ] URDF Joint3 `axis` 与真机 `#GETJPOS` 增量方向是否一致
- [ ] CDC 桥进 `dummy_servo_hardware` 后，再采手眼标定数据
