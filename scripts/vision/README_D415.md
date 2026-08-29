# D415 / Vision 起步说明

## 当前机器状态

- RealSense SDK（`realsense-viewer`）已可用
- **尚未安装 ROS2**，因此仓库内 `dummy_vision` 的 ROS 节点暂时无法直接 `ros2 run`
- 先用本目录独立脚本验证 D415 + ArUco

## 一键装轻量依赖（使用项目 venv，避开 PEP 668）

```bash
bash scripts/vision/install_deps.sh
```

依赖装到：`dummyResearch/.venv-vision/`（不要用系统 `python3 -m pip`）。

## 验证相机内参与检测

```bash
source ~/Projects/dummyResearch/.venv-vision/bin/activate
# 先关掉 realsense-viewer，避免占相机
python scripts/vision/print_d415_intrinsics.py
python scripts/vision/d415_aruco_demo.py --serial 033522060492 --marker-length 0.05
```

标定板字典默认 `ORIGINAL`（与 `dummy_vision` 原代码一致）。若你打印的是 4x4，加 `--dict 4X4_50`。

## 与仓库 ROS 节点的关系

`dummy_moveit_ws/dummy_vision` 已改为订阅 `/camera/camera/color/camera_info` 动态读取内参，换 D415 后不必再手写 D435 的 `camera_matrix`。

完整链路仍需：

1. 安装 ROS2（Ubuntu 24.04 建议 Jazzy）
2. 安装 `realsense2_camera`
3. `colcon build` 工作空间后启动相机 + `aruco_detector_node`

## 推荐 USB

请尽量使用 **USB3 直连**（`rs-enumerate-devices` 中 Usb Type 为 3.x）。
