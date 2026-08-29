#!/usr/bin/env python3
"""
D415 独立 ArUco 检测 demo（不依赖 ROS2）。
用于验证：相机连接 + 本机内参 + 标定板检测。

用法:
  python3 scripts/vision/d415_aruco_demo.py
  python3 scripts/vision/d415_aruco_demo.py --serial 033522060492 --marker-length 0.05

按 q 退出。
"""

from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np
import pyrealsense2 as rs


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D415 ArUco demo")
    p.add_argument("--serial", default="", help="相机序列号，空则用第一台")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--marker-length", type=float, default=0.05, help="ArUco 边长（米）")
    p.add_argument(
        "--dict",
        default="ORIGINAL",
        choices=["ORIGINAL", "4X4_50", "5X5_50", "6X6_250"],
        help="ArUco 字典，需与打印的标定板一致",
    )
    return p.parse_args()


def aruco_dict_from_name(name: str):
    mapping = {
        "ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        "4X4_50": cv2.aruco.DICT_4X4_50,
        "5X5_50": cv2.aruco.DICT_5X5_50,
        "6X6_250": cv2.aruco.DICT_6X6_250,
    }
    return cv2.aruco.getPredefinedDictionary(mapping[name])


def main() -> int:
    args = parse_args()

    ctx = rs.context()
    devices = list(ctx.query_devices())
    if not devices:
        print("未检测到 RealSense。", file=sys.stderr)
        return 1

    serial = args.serial
    if not serial:
        # 优先 D415
        for dev in devices:
            name = dev.get_info(rs.camera_info.name)
            if "D415" in name:
                serial = dev.get_info(rs.camera_info.serial_number)
                break
        if not serial:
            serial = devices[0].get_info(rs.camera_info.serial_number)

    print(f"使用设备 serial={serial}")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
    )

    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        print(f"无法启动彩色流（可尝试降低分辨率或换 USB3）: {e}", file=sys.stderr)
        return 1

    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_profile.get_intrinsics()
    camera_matrix = np.array(
        [[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    dist_coeffs = np.array(intr.coeffs, dtype=np.float64)
    print("彩色内参 camera_matrix:\n", camera_matrix)
    print("dist_coeffs:", dist_coeffs)

    dictionary = aruco_dict_from_name(args.dict)
    # OpenCV 4.7+ 使用 Detector；旧版回退 detectMarkers
    use_detector = hasattr(cv2.aruco, "ArucoDetector")
    if use_detector:
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    else:
        detector = None

    print("对准 ArUco 标定板；按 q 退出。")
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color = frames.get_color_frame()
            if not color:
                continue
            frame = np.asanyarray(color.get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if use_detector:
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, args.marker_length, camera_matrix, dist_coeffs
                )
                for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
                    cv2.drawFrameAxes(
                        frame, camera_matrix, dist_coeffs, rvec, tvec, args.marker_length * 0.5
                    )
                    t = tvec.reshape(-1)
                    text = f"id={ids[i][0]} z={t[2]:.3f}m"
                    c = corners[i][0][0].astype(int)
                    cv2.putText(
                        frame,
                        text,
                        (int(c[0]), int(c[1]) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )
                cv2.putText(
                    frame,
                    f"detected: {len(ids)}",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )
            else:
                cv2.putText(
                    frame,
                    "no ArUco",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )

            cv2.imshow("D415 ArUco demo", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
