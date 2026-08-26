# 回家傻瓜式操作

> 家里 Linux 台式机专用。单位 Windows 远程开发见 `AGENTS.md`。

---

## 第一次（只做一次）

```bash
cd ~/projects/dummy          # git clone 后的目录
bash scripts/home/setup_once.sh
```

按提示装好 **Tailscale** 和 **SSH**（见 `AGENTS.md`）。

---

## 以后每次回家（三条命令）

```bash
cd ~/projects/dummy
git pull
bash scripts/home/go.sh
```

`go.sh` 会自动：**拉代码 → 按需编译 → 启动 MoveIt + Stream API**。

---

## 常用命令

| 命令 | 作用 |
|---|---|
| `bash scripts/home/go.sh` | pull + 编译 + 启动（推荐） |
| `bash scripts/home/start.sh` | 只启动服务 |
| `bash scripts/home/stop.sh` | 停止服务 |
| `bash scripts/home/status.sh` | 查看状态 |
| `bash scripts/home/lerobot.sh` | 启动 LeRobot 手机遥操作 |
| `tmux attach -t dummy` | 进入后台日志窗口 |

---

## 验证是否成功

```bash
bash scripts/home/status.sh
```

应看到：
- ✅ tmux 会话运行中
- ✅ Stream API 响应正常
- ✅ Tailscale IP（单位远程用）

本机测试：

```bash
curl http://127.0.0.1:8001/api/status
```

---

## LeRobot 遥操作

**先确保 `start.sh` / `go.sh` 已运行**，再：

```bash
bash scripts/home/lerobot.sh
```

---

## 退出但不关服务

```bash
# 在 tmux 里按：Ctrl+B 然后 D
```

机械臂服务继续在后台跑。

---

## 出问题？

1. `bash scripts/home/stop.sh` 然后重新 `bash scripts/home/start.sh`
2. 看日志：`tmux attach -t dummy`
3. 详细排查见 `AGENTS.md`
