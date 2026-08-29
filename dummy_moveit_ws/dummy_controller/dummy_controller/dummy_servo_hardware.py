import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import numpy as np
import time
import dummy_controller.dummy_cli_tool.ref_tool

class ServoHardwareStreamer(Node):
    
    rad_volumn_diff = np.array([0,0,1.57079,0,0,0])
    rad_direct_diff = np.array([1,1,1,1,-1,-1])
    pai = 3.1415926
    ready_rad = np.array([0,0,0,0,0,0])
    home_rad = np.array([0,-1.3089,1.5707,0,0,0])
    
    joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

    def __init__(self):
        super().__init__('dummy_servo_hardware')
        self.get_logger().info('Ready to setup dummy arm for STREAMING mode')
        
        try:
            self.my_driver = dummy_controller.dummy_cli_tool.ref_tool.find_any()
            # Enable only — do NOT auto move_j / HOME on connect (DummyStudio may
            # chain !START then !HOME; we keep motion explicit and user-triggered).
            self.my_driver.robot.set_enable(True)
            self.my_driver.robot.set_rgb_mode(4)  # green light is ready
            self.get_logger().info(
                'Hardware connected: motors enabled, no auto motion. '
                'Waiting for /servo_node/command trajectories.'
            )
        except Exception as e:
            self.get_logger().error(f"Failed to connect to hardware: {e}")
            self.my_driver = None
            
        self.current_rad = self.ready_rad.copy()
        if self.my_driver:
            synced = self._read_joint_rad_from_hw()
            if synced is not None:
                self.current_rad = synced
                self.get_logger().info(
                    f'Synced joint_states from hardware (rad): {np.round(self.current_rad, 4)}'
                )
            else:
                self.get_logger().warn(
                    'Could not read joint angles; publishing ready_rad zeros until first command.'
                )
        
        # Subscribe to MoveIt Servo's output topic
        self.servo_sub = self.create_subscription(
            JointTrajectory,
            '/servo_node/command',
            self.servo_callback,
            10
        )
        
        # Publish joint_states for MoveIt (since we remove ros2_control in this mode)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish_joint_states) # 50Hz
        
        self.get_logger().info('Listening to /servo_node/command for joint trajectories...')

    def _read_joint_rad_from_hw(self):
        """Read joint angles (deg on fibre) and convert to MoveIt radians."""
        try:
            joints = [
                self.my_driver.robot.joint_1,
                self.my_driver.robot.joint_2,
                self.my_driver.robot.joint_3,
                self.my_driver.robot.joint_4,
                self.my_driver.robot.joint_5,
                self.my_driver.robot.joint_6,
            ]
            deg = np.array([float(j.angle) for j in joints], dtype=float)
            # Inverse of rad_fix + rad2degree used when commanding hardware.
            rad_cmd = deg / 180.0 * self.pai
            rad = (rad_cmd / self.rad_direct_diff) - self.rad_volumn_diff
            return rad
        except Exception as e:
            self.get_logger().warn(f'Failed to read HW joint angles: {e}')
            return None

    def rad_fix(self, arr_rad):
        return (arr_rad + self.rad_volumn_diff) * self.rad_direct_diff

    def rad2degree(self, arr_rad):
        return arr_rad / self.pai * 180

    def degree2rad(self, arr_degree):
        return arr_degree / 180 * self.pai

    def move_rad(self, arr_rad):
        arr_rad_fixed = self.rad_fix(arr_rad)
        arr_degree = self.rad2degree(arr_rad_fixed)
        if self.my_driver:
            self.my_driver.robot.move_j(arr_degree[0], arr_degree[1], arr_degree[2], arr_degree[3], arr_degree[4], arr_degree[5])
        return True

    def servo_callback(self, msg):
        if len(msg.points) > 0:
            # 1. Log that we have received a message from MoveIt Servo
            self.get_logger().info(f"==> [RECV] New trajectory point from Servo: {len(msg.points)} point(s)")
            
            # Extract joint positions (IK solution from MoveIt)
            target_positions = msg.points[0].positions
            self.current_rad = np.array(target_positions)
            
            # 2. Log raw joint angles (in radians)
            self.get_logger().info(f"    [RAW] Radians: {np.round(self.current_rad, 4)}")
            
            # 3. Log fixed joint angles (applying your rad_fix and conversion to degrees)
            arr_rad_fixed = self.rad_fix(self.current_rad)
            arr_degree = self.rad2degree(arr_rad_fixed)
            self.get_logger().info(f"    [SEND] Degrees to Hardware: {np.round(arr_degree, 2)}")
            
            # 4. Final attempt to send to physical hardware
            if self.my_driver:
                self.get_logger().debug("    [HW] Calling move_j via Fibre...")
                self.move_rad(self.current_rad)
            else:
                self.get_logger().warn("    [HW] Hardware driver not connected, skipping move_j")
        else:
            self.get_logger().debug("Received empty trajectory points from Servo")

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = self.joint_names
        # Convert numpy array to list of native python floats to satisfy ROS 2 type checking
        msg.position = [float(val) for val in self.current_rad]
        self.joint_state_pub.publish(msg)
        # Periodic log to confirm it is alive (once per 2 seconds)
        if int(time.time() * 50) % 100 == 0:
            self.get_logger().debug(f"Publishing joint states: {msg.position}")

    def cleanup(self):
        self.get_logger().info('Cleaning up dummy arm (disable only, no auto HOME)...')
        if self.my_driver:
            try:
                if hasattr(self.my_driver, 'robot'):
                    # Do not move_j on shutdown — avoid unexpected stretch/HOME.
                    self.my_driver.robot.set_enable(False)
                    self.my_driver.robot.set_rgb_mode(0)
                else:
                    self.get_logger().warn("Driver object has no 'robot' attribute")
            except Exception as e:
                self.get_logger().error(f"Error during cleanup: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ServoHardwareStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
