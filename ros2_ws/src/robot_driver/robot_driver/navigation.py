import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class WaypointNavigation(Node):
    def __init__(self):
        super().__init__('waypoint_nav_node')

        try:
            package_share_dir = get_package_share_directory('robot_driver')
            config_path = os.path.join(package_share_dir, 'config', 'navigation_config.yaml')
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Error cargando config: {e}')
            self.config = {}

        self.waypoints = self.config.get('waypoints', [])
        self.linear_speed = float(self.config.get('linear_speed', 0.1))
        self.angular_speed = float(self.config.get('angular_speed', 1.5))
        self.dist_tolerance = float(self.config.get('dist_tolerance', 0.1))

        self.current_idx = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.odom_received = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.create_subscription(Bool, '/vision/measuring', self._vision_measuring_cb, 10)
        self.vision_measuring = False
        self.create_timer(0.1, self.control_loop)

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def _vision_measuring_cb(self, msg):
        self.vision_measuring = msg.data

    def control_loop(self):
        if not self.odom_received or self.current_idx >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            return

        if self.vision_measuring:
            self.cmd_pub.publish(Twist())  # robot quieto mientras vision mide
            return

        target_x, target_y = self.waypoints[self.current_idx]
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.dist_tolerance:
            self.current_idx += 1
            return

        angle_to_target = math.atan2(dy, dx)
        alpha = math.atan2(math.sin(angle_to_target - self.robot_theta),
                           math.cos(angle_to_target - self.robot_theta))

        cmd = Twist()

        angle_threshold = 0.2
        kp_angular = 0.3  # ajusta si vibra o es lento

        if abs(alpha) > angle_threshold:
            cmd.linear.x = 0.0
            angular_vel = alpha * kp_angular
            min_w = 0.2
            # velocidad minima para vencer friccion estatica
            if abs(angular_vel) < min_w:
                angular_vel = min_w if alpha > 0 else -min_w
            cmd.angular.z = angular_vel
        else:
            alignment_factor = math.cos(alpha)
            cmd.linear.x = self.linear_speed * alignment_factor
            cmd.angular.z = alpha * 0.5  # correccion proporcional en linea recta

        cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd.angular.z))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
