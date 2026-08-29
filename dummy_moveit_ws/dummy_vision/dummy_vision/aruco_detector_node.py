# simple Python demo using package cv2 + cv_bridge
# Intrinsics come from /camera/.../camera_info so D415/D435 both work.
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
import tf_transformations
import tf2_ros
import numpy as np

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from tf_transformations import quaternion_multiply, quaternion_matrix


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('marker_length', 0.05)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value

        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.sub_info = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.br = CvBridge()
        self.pub = self.create_publisher(PoseStamped, '/aruco_target_pose', 10)
        self.pub_cam = self.create_publisher(PoseStamped, '/aruco_camera_pose', 10)
        self.pub_world = self.create_publisher(PoseStamped, '/aruco_world_pose', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Filled from CameraInfo (works for D415 / D435). Optional fallback until first info arrives.
        self.camera_matrix = None
        self.dist_coeffs = None
        self._camera_info_ready = False

        # publish pose in every 1 seconds
        self.last_pose_tool = None
        self.last_pose_cam = None
        self.last_pose_world = None
        self.create_timer(1.0, self.publish_cached_poses)

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        if not self._camera_info_ready:
            self._camera_info_ready = True
            self.get_logger().info(
                f'CameraInfo ready: fx={self.camera_matrix[0, 0]:.2f} fy={self.camera_matrix[1, 1]:.2f} '
                f'cx={self.camera_matrix[0, 2]:.2f} cy={self.camera_matrix[1, 2]:.2f}'
            )

    def image_callback(self, msg):
        if not self._camera_info_ready:
            self.get_logger().warn('Waiting for CameraInfo before ArUco pose estimation...')
            return
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)  #DICT_ARUCO_ORIGINAL ,DICT_4X4_50
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
        if ids is not None:
            #self.get_logger().info('okok,detect aruco size:',len(ids))
            # use cv2 function to tran image pose to space pose
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            # use custom function to tran image pose to space pose
            #rvec_m, tvec_m, _ = self.my_estimate_pose_single_markers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            # just take the first marker
            pos_x = tvec[0][0][0]
            pos_y = tvec[0][0][1]
            pos_z = tvec[0][0][2]
            orien_x = rvec[0][0][0]
            orien_y = rvec[0][0][1]
            orien_z = rvec[0][0][2]
            # compare result between two function
            self.get_logger().info('check,estimatePoseSingleMarkers.pos_x:'+str(tvec[0][0][0]))
            self.get_logger().info('check,estimatePoseSingleMarkers.pos_y:'+str(tvec[0][0][1]))
            self.get_logger().info('check,estimatePoseSingleMarkers.pos_z:'+str(tvec[0][0][2]))
            #self.get_logger().info('check,my_estimate_pose_single_markers.pos_x:'+tvec[0][0][0]+'=='+tvec_m[0][0][0])
            #self.get_logger().info('check,my_estimate_pose_single_markers.pos_y:'+tvec[0][0][1]+'=='+tvec_m[0][0][1])
            #self.get_logger().info('check,my_estimate_pose_single_markers.pos_z:'+tvec[0][0][2]+'=='+tvec_m[0][0][2])
            #self.get_logger().info('check,my_estimate_pose_single_markers.orien_x:'+tvec[0][0][0]+'=='+tvec_m[0][0][0])
            #self.get_logger().info('check,my_estimate_pose_single_markers.orien_y:'+tvec[0][0][1]+'=='+tvec_m[0][0][1])
            #self.get_logger().info('check,my_estimate_pose_single_markers.orien_z:'+tvec[0][0][2]+'=='+tvec_m[0][0][2])
            
            # set camera_color_optical_frame pose
            pose_in_optical = PoseStamped()
            pose_in_optical.header.stamp = self.get_clock().now().to_msg()
            pose_in_optical.header.frame_id = 'camera_color_optical_frame'
            pose_in_optical.pose.position.x = pos_x
            pose_in_optical.pose.position.y = pos_y
            pose_in_optical.pose.position.z = pos_z
            quat = tf_transformations.quaternion_from_euler(orien_x,orien_y,orien_z)
            pose_in_optical.pose.orientation.x = quat[0]
            pose_in_optical.pose.orientation.y = quat[1]
            pose_in_optical.pose.orientation.z = quat[2]
            pose_in_optical.pose.orientation.w = quat[3]
            
            # make camera_color_optical_frame turn to camera_link, or straight to link6_1_1
            #self.get_logger().info('it frames in buffer:')
            #self.get_logger().info(self.tf_buffer.all_frames_as_string())
            self.last_pose_cam = pose_in_optical
            try:
                self.tf_buffer.can_transform("link6_1_1", "camera_color_optical_frame", rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0))
                transform = self.tf_buffer.lookup_transform("link6_1_1", "camera_color_optical_frame", rclpy.time.Time())
                self.tf_buffer.can_transform("base_link", "camera_color_optical_frame", rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0))
                transform_world = self.tf_buffer.lookup_transform("base_link", "camera_color_optical_frame", rclpy.time.Time())
                self.last_pose_tool = self.transform_pose(pose_in_optical, transform)
                self.last_pose_world = self.transform_pose(pose_in_optical, transform_world)
            except Exception:
                self.get_logger().warn("TF from camera_color_optical_frame to link6_1_1 or base_link not available yet")

            self.get_logger().info('---------------------------------------')
        # else: self.get_logger().info('oh,can not detect any aruco')
    
    def my_estimate_pose_single_markers(self,corners, marker_length, camera_matrix, dist_coeffs):
        """
        use solvePnP to replace cv2.aruco.estimatePoseSingleMarkers
        """
        # ArUco origin shape (use meter)
        half_len = marker_length / 2.0
        objp = np.array([
            [-half_len,  half_len, 0],
            [ half_len,  half_len, 0],
            [ half_len, -half_len, 0],
            [-half_len, -half_len, 0]
        ], dtype=np.float32)
        rvecs = []
        tvecs = []
        for corner in corners:
            # corner take (4, 2) from (1, 4, 2)
            imgp = corner[0].astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(objp, imgp, camera_matrix, dist_coeffs)
            if success:
                rvecs.append(rvec)
                tvecs.append(tvec)
            else:
                rvecs.append(None)
                tvecs.append(None)
        # tran to numpy, the same with estimatePoseSingleMarkers
        return np.array(rvecs), np.array(tvecs), None

    def publish_cached_poses(self):
        if self.last_pose_tool:
            self.pub.publish(self.last_pose_tool)
        if self.last_pose_cam:
            self.pub_cam.publish(self.last_pose_cam)
        if self.last_pose_world:
            self.pub_world.publish(self.last_pose_world)

    def transform_pose(self, pose, transform):
        # get translation and rotation from TransformStamped
        translation = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        )
        rotation = (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        )

        # origin point
        point = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            1.0
        ])

        # build change matrix
        transform_mat = tf_transformations.quaternion_matrix(rotation)
        transform_mat[0:3, 3] = translation

        # take the transform
        transformed_point = np.dot(transform_mat, point)

        # get quat
        input_quat = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ]
        output_quat = quaternion_multiply(rotation, input_quat)

        # build new PoseStamped
        transformed_pose = PoseStamped()
        transformed_pose.header.stamp = pose.header.stamp
        transformed_pose.header.frame_id = transform.header.frame_id
        transformed_pose.pose.position.x = transformed_point[0]
        transformed_pose.pose.position.y = transformed_point[1]
        transformed_pose.pose.position.z = transformed_point[2]
        transformed_pose.pose.orientation.x = output_quat[0]
        transformed_pose.pose.orientation.y = output_quat[1]
        transformed_pose.pose.orientation.z = output_quat[2]
        transformed_pose.pose.orientation.w = output_quat[3]

        return transformed_pose

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
