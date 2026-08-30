# 交接：Windows DummyStudio 能连通的办法（给本机 Linux 上的 Cursor 读）

**不要在 Cursor 云端虚拟机里验证机械臂。** 云端没有 REF 的 USB。

这份记录来自：同一台双系统电脑，**Windows 上 DummyStudio 已能连接并拖动**。Linux 侧按同一条路径复现即可。

## Windows 实际在做什么

DummyStudio 打开的是 REF 板的 **USB CDC 虚拟串口**，不是 Fibre/WinUSB 调试口。

| 项 | Windows 上的值 |
| --- | --- |
| 设备名 | Robot Embedded Framework REF … CDC Interface |
| USB | VID `1209` PID `0d32` |
| 端口 | `COMx`（设备管理器里那个 CDC 口） |
| 波特率 | 115200 8N1 |
| 协议 | 文本 ASCII，行结束 `\n` / `\r`，应答 `\r\n` |

连接后上位机顺序：

1. 独占打开 COM
2. `!START` → 应答 `Started ok`（使能电机）
3. `#CMDMODE 2`（实时覆盖，滑条才能拖）
4. `#GETJPOS` 读六个关节角（度）
5. 拖动时发 `>j1,j2,j3,j4,j5,j6`（可带速度）

复位「7」字姿态：`0, -73, 180, 0, 0, 0`。

机器可读副本：`docs/windows-connection.json`。

## Linux 必须对齐的同一条路

把 CDC 映射成 `/dev/ttyACM*`，然后发**同一串 ASCII**。不要走 `reftool --path usb`（Fibre bulk）。那会和内核 `cdc_acm` 抢接口，Windows 能拖、Linux 拖不动就是这个差。

本机 Linux 验证（插着和 Windows 时同一根 USB）：

```bash
lsusb -d 1209:0d32
bash scripts/linux_setup.sh          # 只需一次：udev + dialout，然后重新登录
python3 -m dummy_arm diagnose
python3 -m dummy_arm ping --port auto
python3 -m dummy_arm serve --port auto
```

通过标准：

- `diagnose` 的 `machine` 是 `local_linux`（不是 `cloud_vm`）
- `can_control_hardware` 为 true
- 能看到 `1209:0d32` 和 `/dev/ttyACM*`
- `ping` 能打出六个关节角
- 网页里 `!START` 后拖滑条，臂会动

失败时对照：

- 只有 `10c4:ea60`：USB 插反，那是 CP2102 调试口，DummyStudio 不用
- `Permission denied`：没加入 `dialout` 或没重新登录
- 口存在但无应答：ModemManager 抢走 ttyACM，或脚本在用 Fibre 而不是 CDC ASCII

手眼标定等功能：等上面 `ping` + 拖动通过后再做。
