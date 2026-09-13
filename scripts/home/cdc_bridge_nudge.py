#!/usr/bin/env python3
"""Keyboard jog via CDC bridge ~/nudge (no second serial writer).

  1-5  J1-J5  (+3°)     R  reverse     Q  quit
  6    J6 — camera is on J5 housing; do not use for hand-eye samples
"""

from __future__ import annotations

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


def _getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main() -> int:
    rclpy.init()
    node = Node("cdc_bridge_nudge")
    pub = node.create_publisher(Int32, "/cdc_servo_bridge/nudge", 10)
    sign = 1
    print("点动经 CDC 桥（不占串口）。先点进本终端：1-5 轴，R 反向，6=J6勿用于标定，Q 退出", flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            ch = _getch()
            if ch in ("q", "Q", "\x03"):
                break
            if ch in ("r", "R"):
                sign *= -1
                print(f"direction={sign:+d}", flush=True)
                continue
            if ch in "123456":
                if ch == "6":
                    print("J6 不带动相机，标定请用 1-5", flush=True)
                msg = Int32()
                msg.data = sign * int(ch)
                pub.publish(msg)
                print(f"nudge J{ch} {sign:+d}", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
