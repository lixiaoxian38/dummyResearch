#!/usr/bin/env python3
"""打印已连接 RealSense（优先 D415）彩色相机内参，供 dummy_vision 使用。"""

from __future__ import annotations

import json
import sys

import pyrealsense2 as rs


def main() -> int:
    ctx = rs.context()
    devices = list(ctx.query_devices())
    if not devices:
        print("未检测到 RealSense 设备。请检查 USB 连接。", file=sys.stderr)
        return 1

    for dev in devices:
        name = dev.get_info(rs.camera_info.name)
        serial = dev.get_info(rs.camera_info.serial_number)
        print(f"设备: {name}  serial={serial}")

    # 打开彩色流以读取有效内参
    pipeline = rs.pipeline()
    config = rs.config()
    # 固定选用第一台；若有多台可按 serial 过滤
    serial = devices[0].get_info(rs.camera_info.serial_number)
    config.enable_device(serial)
    # USB2 友好分辨率
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)
    try:
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        data = {
            "serial": serial,
            "device": devices[0].get_info(rs.camera_info.name),
            "width": intr.width,
            "height": intr.height,
            "fx": intr.fx,
            "fy": intr.fy,
            "ppx": intr.ppx,
            "ppy": intr.ppy,
            "model": str(intr.model),
            "coeffs": list(intr.coeffs),
            "camera_matrix": [
                [intr.fx, 0.0, intr.ppx],
                [0.0, intr.fy, intr.ppy],
                [0.0, 0.0, 1.0],
            ],
            "dist_coeffs": list(intr.coeffs),
        }
        print(json.dumps(data, indent=2, ensure_ascii=False))
        out = "/tmp/d415_intrinsics.json"
        with open(out, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        print(f"\n已写入 {out}")
    finally:
        pipeline.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
