#!/usr/bin/env python3
"""ArUco pose detector for eye-in-hand tracking.

Publishes marker pose every frame and TF: optical_frame -> camera_marker.
Intrinsics come from CameraInfo (D415 / D435).
"""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
import tf2_ros
import tf_transformations
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix, quaternion_multiply

ARUCO_DICTS = {
    "ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "4X4_50": cv2.aruco.DICT_4X4_50,
    "4X4_100": cv2.aruco.DICT_4X4_100,
    "5X5_50": cv2.aruco.DICT_5X5_50,
    "6X6_50": cv2.aruco.DICT_6X6_50,
}


def rvec_to_quaternion(rvec: np.ndarray) -> np.ndarray:
    """OpenCV rvec (Rodrigues) -> quaternion xyzw."""
    rot_mat, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    # tf_transformations expects 4x4
    mat44 = np.eye(4)
    mat44[:3, :3] = rot_mat
    return tf_transformations.quaternion_from_matrix(mat44)


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("marker_length", 0.05)
        self.declare_parameter("marker_id", -1)  # -1 = first detected
        self.declare_parameter("aruco_dict", "ORIGINAL")
        self.declare_parameter("optical_frame", "camera_color_optical_frame")
        self.declare_parameter("marker_frame", "camera_marker")
        self.declare_parameter("ee_frame", "link6_1_1")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_debug", False)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.marker_length = float(self.get_parameter("marker_length").value)
        self.marker_id = int(self.get_parameter("marker_id").value)
        dict_name = str(self.get_parameter("aruco_dict").value).upper()
        self.optical_frame = str(self.get_parameter("optical_frame").value)
        self.marker_frame = str(self.get_parameter("marker_frame").value)
        self.ee_frame = str(self.get_parameter("ee_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_debug = bool(self.get_parameter("publish_debug").value)

        if dict_name not in ARUCO_DICTS:
            self.get_logger().warn(f"Unknown aruco_dict={dict_name}, using ORIGINAL")
            dict_name = "ORIGINAL"
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
        # OpenCV 4.6: DetectorParameters() constructs but segfaults in detectMarkers.
        # Prefer DetectorParameters_create(); fall back for 4.7+.
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self._aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self._aruco_params = cv2.aruco.DetectorParameters()

        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.br = CvBridge()
        self.pub_tool = self.create_publisher(PoseStamped, "/aruco_target_pose", 10)
        self.pub_cam = self.create_publisher(PoseStamped, "/aruco_camera_pose", 10)
        self.pub_world = self.create_publisher(PoseStamped, "/aruco_world_pose", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_matrix = None
        self.dist_coeffs = None
        self._camera_info_ready = False
        self._last_tf_warn_ns = 0

        self.get_logger().info(
            f"ArUco detector ready: dict={dict_name} length={self.marker_length}m "
            f"marker_id={self.marker_id} frame={self.marker_frame}"
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        if not self._camera_info_ready:
            self._camera_info_ready = True
            self.get_logger().info(
                f"CameraInfo ready: fx={self.camera_matrix[0, 0]:.2f} "
                f"fy={self.camera_matrix[1, 1]:.2f} "
                f"cx={self.camera_matrix[0, 2]:.2f} cy={self.camera_matrix[1, 2]:.2f}"
            )

    def image_callback(self, msg: Image):
        if not self._camera_info_ready:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > 2e9:
                self._last_tf_warn_ns = now_ns
                self.get_logger().warn("Waiting for CameraInfo before ArUco pose estimation...")
            return

        try:
            # RealSense often publishes rgb8; accept either.
            enc = (msg.encoding or "").lower()
            if enc in ("rgb8", "rgb"):
                frame = self.br.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            else:
                frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._aruco_dict, parameters=self._aruco_params
            )
            if ids is None or len(ids) == 0:
                return

            ids_flat = ids.flatten()
            idx = 0
            if self.marker_id >= 0:
                matches = np.where(ids_flat == self.marker_id)[0]
                if len(matches) == 0:
                    return
                idx = int(matches[0])

            dist = self.dist_coeffs
            if dist is None or dist.size == 0:
                dist = np.zeros((5, 1), dtype=np.float64)
            else:
                dist = np.asarray(dist, dtype=np.float64).reshape(-1, 1)

            # OpenCV 4.7+ prefers ArucoDetector; keep estimatePoseSingleMarkers for 4.6.
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, dist
            )
            rvec = rvecs[idx][0]
            tvec = tvecs[idx][0]
            quat = rvec_to_quaternion(rvec)

            # Use node clock for TF stamp so easy_handeye2 lookups at "now" succeed.
            # (Camera stamps can lag / drop when the marker is briefly lost.)
            stamp = self.get_clock().now().to_msg()
            pose_cam = PoseStamped()
            pose_cam.header.stamp = stamp
            pose_cam.header.frame_id = self.optical_frame
            pose_cam.pose.position.x = float(tvec[0])
            pose_cam.pose.position.y = float(tvec[1])
            pose_cam.pose.position.z = float(tvec[2])
            pose_cam.pose.orientation.x = float(quat[0])
            pose_cam.pose.orientation.y = float(quat[1])
            pose_cam.pose.orientation.z = float(quat[2])
            pose_cam.pose.orientation.w = float(quat[3])

            self.pub_cam.publish(pose_cam)
            self._broadcast_marker_tf(stamp, tvec, quat)

            if self.publish_debug:
                self.get_logger().info(
                    f"marker t=[{tvec[0]:.3f},{tvec[1]:.3f},{tvec[2]:.3f}] "
                    f"q=[{quat[0]:.3f},{quat[1]:.3f},{quat[2]:.3f},{quat[3]:.3f}]"
                )

            try:
                tf_ee = self.tf_buffer.lookup_transform(
                    self.ee_frame, self.optical_frame, rclpy.time.Time()
                )
                tf_base = self.tf_buffer.lookup_transform(
                    self.base_frame, self.optical_frame, rclpy.time.Time()
                )
                pose_tool = self.transform_pose(pose_cam, tf_ee)
                pose_world = self.transform_pose(pose_cam, tf_base)
                self.pub_tool.publish(pose_tool)
                self.pub_world.publish(pose_world)
            except Exception:
                now_ns = self.get_clock().now().nanoseconds
                if now_ns - self._last_tf_warn_ns > 2e9:
                    self._last_tf_warn_ns = now_ns
                    self.get_logger().warn(
                        f"TF {self.optical_frame} -> {self.ee_frame}/{self.base_frame} "
                        "not available yet (need hand-eye / RealSense TF)"
                    )
        except Exception as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > 2e9:
                self._last_tf_warn_ns = now_ns
                self.get_logger().error(f"image_callback failed: {exc}")

    def _broadcast_marker_tf(self, stamp, tvec, quat):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.optical_frame
        tf_msg.child_frame_id = self.marker_frame
        tf_msg.transform.translation.x = float(tvec[0])
        tf_msg.transform.translation.y = float(tvec[1])
        tf_msg.transform.translation.z = float(tvec[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(tf_msg)

    def transform_pose(self, pose: PoseStamped, transform) -> PoseStamped:
        translation = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
        rotation = (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        point = np.array(
            [
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                1.0,
            ]
        )
        transform_mat = quaternion_matrix(rotation)
        transform_mat[0:3, 3] = translation
        transformed_point = transform_mat @ point
        input_quat = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
        output_quat = quaternion_multiply(rotation, input_quat)

        out = PoseStamped()
        out.header.stamp = pose.header.stamp
        out.header.frame_id = transform.header.frame_id
        out.pose.position.x = float(transformed_point[0])
        out.pose.position.y = float(transformed_point[1])
        out.pose.position.z = float(transformed_point[2])
        out.pose.orientation.x = float(output_quat[0])
        out.pose.orientation.y = float(output_quat[1])
        out.pose.orientation.z = float(output_quat[2])
        out.pose.orientation.w = float(output_quat[3])
        return out


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
