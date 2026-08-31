# Dummy Research — 开发工作方式

本仓库：`https://github.com/lixiaoxian38/dummyResearch.git`

> **回家傻瓜式操作 → 直接看 [`HOME.md`](HOME.md)**（`git pull` + `bash scripts/home/go.sh`）

## 角色分工

| 设备 | 系统 | 角色 | 做什么 |
|---|---|---|---|
| **家里台式机** | Linux | **Server（主机）** | 机械臂 USB、ROS2、MoveIt2、stream API、D435 相机 |
| **单位笔记本** | Windows | **Client（远程）** | 日常办公 + Cursor Remote SSH 连家里写代码 |

原则：**重活全在家里 Linux 跑，单位笔记本只远程连过去，不必切 Linux。**

---

## 一、网络：Tailscale 虚拟组网

两台设备各连各的 WiFi，用 Tailscale 组成虚拟局域网。

### 家里 Linux（一次性）

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
# 记下 Tailscale IP，例如 100.64.1.5
```

### 单位 Windows（一次性）

1. 安装 Tailscale：https://tailscale.com/download/windows
2. 登录**同一个** Tailscale 账号
3. 记下家里机器的 Tailscale IP

### 验证连通

单位 Windows PowerShell：

```powershell
ping 100.64.1.5
curl http://100.64.1.5:8001/api/joints
```

---

## 二、家里 Linux：SSH + 机械臂服务

### 安装 SSH

```bash
sudo apt install openssh-server
sudo systemctl enable --now ssh
```

### 克隆仓库（第一次）

```bash
git clone https://github.com/lixiaoxian38/dummyResearch.git ~/projects/dummy
cd ~/projects/dummy
```

### 编译 ROS 工作空间

```bash
cd ~/projects/dummy/dummy_moveit_ws
colcon build
source install/setup.bash
```

### 启动机械臂（每次调试）

**终端 1 — MoveIt + Servo 流式控制：**

```bash
cd ~/projects/dummy/dummy_moveit_ws
source install/setup.bash
ros2 launch dummy_moveit_config servo_streaming.launch.py
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
```

**终端 2 — HTTP 流式 API（LeRobot / 远程控制）：**

```bash
cd ~/projects/dummy/dummy_moveit_ws
source install/setup.bash
python dummy_server/server/stream_api_server.py
```

确认 API 监听 `0.0.0.0:8001`（不要只绑 `127.0.0.1`），远程才能连。

### 防火墙（如启用 ufw）

```bash
sudo ufw allow in on tailscale0
sudo ufw allow 8001/tcp
```

---

## 三、单位 Windows：Cursor Remote SSH

**详细步骤见 [`docs/CURSOR_REMOTE.md`](docs/CURSOR_REMOTE.md)**

### 快速配置

**家里 Linux（一次性）：**
```bash
bash scripts/remote/setup_home_ssh.sh
```

**单位 Windows（一次性）：**
```powershell
powershell -ExecutionPolicy Bypass -File scripts\remote\setup_work_cursor.ps1
```

### 每次远程开发

1. 确认家里台式机在线（Linux 已启动）
2. 单位开 Tailscale
3. Cursor → `Ctrl+Shift+P` → **Remote-SSH: Connect to Host** → `dummy-home`
4. **Open Folder** → `/home/你的用户名/projects/dummy`

在 Cursor 里改代码、跑终端、调 ROS，**实际执行都在家里 Linux**。

---

## 四、Git 同步（两台设备）

```bash
# 开工前
git pull

# 收工前
git add -A
git commit -m "说明本次改动"
git push
```

| 设备 | 路径建议 |
|---|---|
| 家里 Linux | `~/projects/dummy` |
| 单位 Remote SSH | 同上（连的是家里目录） |

---

## 五、Cursor Session 规则

| 场景 | 做法 |
|---|---|
| 单位 SSH 写代码 | **Local（Remote SSH）** — 代码在家里跑 |
| 跨单位↔家里续聊 Agent | **Cloud Agent**，或 **Move to Cloud** |
| 家里本地 Cursor 开发 | **Local**，新 session 先读本文档和 `docs/TODO.md` |
| 需要后台长任务 | **Cloud** |

**重要：Local session 不能跨设备继承。** 单位 SSH 的对话，回家本地 Cursor 看不到。要续聊用 Cloud；不续聊就把进度写进 `docs/TODO.md`。

---

## 六、LeRobot 遥操作

### 在家里同一局域网

```bash
conda activate lerobot
cd ~/projects/dummy/lerobot

lerobot-teleoperate \
  --robot.type=dummy_stream \
  --robot.id=my_awesome_arm \
  --robot.ip=127.0.0.1 \
  --robot.port=8001 \
  --teleop.type=mobile \
  --teleop.id=my_awesome_teleop
```

### 从单位远程遥操作（家里服务已启动）

把 IP 改成家里 Tailscale 地址：

```bash
lerobot-teleoperate \
  --robot.type=dummy_stream \
  --robot.id=my_awesome_arm \
  --robot.ip=100.64.1.5 \
  --robot.port=8001 \
  --teleop.type=mobile \
  --teleop.id=my_awesome_teleop
```

---

## 七、典型一天

### 在单位（Windows）

```
1. 开 Tailscale
2. Cursor Remote SSH → dummy-home → 打开 ~/projects/dummy
3. 写代码 / 改 lerobot / 看 ROS 日志
4. git commit && git push
5. Windows 正常办公不受影响
```

### 在家里（Linux）

```
1. git pull
2. 启动 MoveIt + stream_api_server
3. 本地 Cursor 或继续 SSH 调试
4. 测机械臂 / LeRobot 录数据
5. git push
```

---

## 八、仓库结构速查

```
dummy/
├── dummy_moveit_ws/          # ROS2 + MoveIt2 主工程
│   ├── dummy-ros2_description/
│   ├── dummy_moveit_config/
│   ├── dummy_controller/
│   ├── dummy_server/         # pymoveit2 + HTTP stream API
│   └── dummy_vision/         # 手眼标定（D435 + ArUco）
├── lerobot/                  # LeRobot + dummy_stream 机器人类型
├── docs/TODO.md              # 当前进度（跨 session 用）
└── AGENTS.md                 # 本文件
```

### 关键文档

- 流式控制：`dummy_moveit_ws/doc/moveit流式控制笔记.md`
- 手眼标定：`dummy_moveit_ws/doc/Dummy手眼标定笔记.pdf`
- Stream API：`dummy_moveit_ws/dummy_server/server/STREAM_API_DOC.md`

---

## 九、新 Cursor Session 恢复上下文

开新对话时可以说：

> 先读 `AGENTS.md` 和 `docs/TODO.md`，我们接着做 XXX。

---

## 十、常见问题

**Q: push 失败 / GitHub 连不上？**  
A: 试 SSH remote：`git remote set-url origin git@github.com:lixiaoxian38/dummyResearch.git`

**Q: 远程 SSH 连不上？**  
A: 检查 Tailscale 是否在线、家里机器是否开机、SSH 服务是否运行。

**Q: stream API 远程连不上？**  
A: 确认 `stream_api_server.py` 监听 `0.0.0.0:8001`，且防火墙放行。

**Q: 机械臂没反应？**  
A: 先确认 Servo 已启动：`ros2 service call /servo_node/start_servo std_srvs/srv/Trigger`
