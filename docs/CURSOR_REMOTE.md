# Cursor Remote SSH → 家里 Linux

> 单位 **Windows** 连家里 **Linux**（`lxx01` / 用户 `lxx`），在 Cursor 里远程写代码、跑 ROS。

---

## 架构

```text
单位 Windows (Cursor)  --Tailscale+SSH-->  家里 Linux lxx01
                                              ~/Projects/dummyResearch
```

---

## 一、家里 Linux（一次性，在家执行）

```bash
cd ~/Projects/dummyResearch
git pull
bash scripts/remote/setup_home_ssh.sh
```

脚本会：
- 安装并启用 `openssh-server`
- 把单位笔记本的公钥写入 `~/.ssh/authorized_keys`
- 打印 Tailscale IP / 主机名

**记下输出的 Tailscale 地址**（如 `100.x.x.x` 或 `lxx01`）。

---

## 二、单位 Windows（一次性）

### 1. 安装 Tailscale

https://tailscale.com/download/windows  
登录与家里**同一账号**。

### 2. 运行配置脚本

PowerShell：

```powershell
cd J:\dummy   # 或你的仓库路径
git pull
powershell -ExecutionPolicy Bypass -File scripts\remote\setup_work_cursor.ps1
```

会写入 `C:\Users\lxx\.ssh\config` 并测试 `ssh dummy-home`。

### 3. 安装 Cursor 扩展

Cursor → Extensions → 搜索并安装：**Remote - SSH**（Microsoft 出品）

### 4. 测试 SSH

```powershell
ssh dummy-home "hostname && ls ~/Projects/dummyResearch"
```

能输出 `lxx01` 和目录列表即成功。

---

## 三、每次远程开发（单位）

1. 确认 **Tailscale 已连接**（任务栏图标绿色）
2. 确认 **家里台式机开机**（Linux 已进桌面或服务器）
3. 打开 **Cursor**
4. `Ctrl+Shift+P` → **Remote-SSH: Connect to Host...** → 选 **`dummy-home`**
5. 新窗口打开后：**File → Open Folder** → `/home/lxx/Projects/dummyResearch`
6. Agent 输入框 **Run on** 选 **This Computer**（此时「本机」= 家里 Linux）

---

## 四、Cursor 推荐设置（已写入单位 settings.json）

| 设置 | 作用 |
|---|---|
| `remote.SSH.remotePlatform.dummy-home` | 识别为 Linux |
| `remote.SSH.connectTimeout` | 跨网连接超时加长 |
| `remote.SSH.showLoginTerminal` | 连不上时弹出登录终端便于排查 |

---

## 五、常见问题

### 连不上 `dummy-home`

```powershell
tailscale status          # 能否看到 lxx01？
ssh -o StrictHostKeyChecking=accept-new dummy-home hostname
```

| 报错 | 原因 | 处理 |
|---|---|---|
| `Host key verification failed` | Windows 未信任主机指纹 | 已在 `~/.ssh/config` 加 `StrictHostKeyChecking accept-new`；重跑 `setup_work_cursor.ps1` |
| `Permission denied (publickey)` | 家里未授权单位公钥 | **在家执行** `bash scripts/remote/setup_home_ssh.sh` |
| `Could not resolve hostname` | Tailscale 未连 | 安装/登录 Tailscale |

家里检查：

```bash
sudo systemctl status ssh
tailscale status
```

### Cursor 一直「Installing server」

家里网络慢时多等 2～3 分钟。仍失败：

```powershell
ssh dummy-home "rm -rf ~/.cursor-server ~/.vscode-server"
```

然后 Cursor 重连（会重新下载 remote server）。

### 扩展在远程不生效

`Ctrl+Shift+P` → **Remote-SSH: Install Local Extensions in SSH: dummy-home**

Python / ROS 相关扩展需装在 **SSH 侧**。

### Session 不能带回家

单位 SSH 的对话，回家本地 Cursor **看不到**。要续聊用 **Cloud Agent**，或更新 `docs/TODO.md`。

---

## 六、配置文件位置

| 文件 | 位置 |
|---|---|
| SSH config | `C:\Users\lxx\.ssh\config` |
| 单位公钥 | `C:\Users\lxx\.ssh\id_ed25519.pub` |
| 家里授权 | `/home/lxx/.ssh/authorized_keys` |
| 工作区配置模板 | `scripts/remote/work_config.env.example` |
