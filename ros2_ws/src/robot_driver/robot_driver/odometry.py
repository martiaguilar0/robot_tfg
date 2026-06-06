import rclpy
from rclpy.node import Node
import math
from robot_msgs.msg import EncoderTicks
from nav_msgs.msg import Odometry

TICKS_PER_REV   = 1496
WHEEL_DIAMETER  = 0.08
WHEEL_BASE      = 0.2235
METERS_PER_TICK = (math.pi * WHEEL_DIAMETER) / TICKS_PER_REV

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_enc_l = None
        self.prev_enc_r = None
        self.last_time = None

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(EncoderTicks, '/encoders/ticks', self.encoders_cb, 10)

    def encoders_cb(self, msg):
        enc_l = msg.enc_l
        enc_r = msg.enc_r
        current_time = rclpy.time.Time.from_msg(msg.header.stamp)

        if self.prev_enc_l is None:
            self.prev_enc_l = enc_l
            self.prev_enc_r = enc_r
            self.last_time = current_time
            return

        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        if dt <= 0 or dt > 0.1:
            return

        delta_l = (enc_l - self.prev_enc_l) * METERS_PER_TICK
        delta_r = (enc_r - self.prev_enc_r) * METERS_PER_TICK

        delta_s = (delta_r + delta_l) / 2.0
        # (l - r) para que girar a la izquierda sea positivo, convencion ROS
        delta_theta = (delta_l - delta_r) / WHEEL_BASE

        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        v_lin = delta_s / dt
        v_ang = delta_theta / dt

        self.publish_odom(msg.header.stamp, v_lin, v_ang)

        self.prev_enc_l = enc_l
        self.prev_enc_r = enc_r
        self.last_time = current_time

    def publish_odom(self, stamp, v_lin, v_ang):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = v_lin
        odom.twist.twist.angular.z = v_ang

        # subir si los encoders patinan mucho
        odom.pose.covariance[0]  = 0.05  # x
        odom.pose.covariance[7]  = 0.05  # y
        odom.pose.covariance[35] = 0.1   # theta
        odom.twist.covariance[0]  = 0.05  # v_lin
        odom.twist.covariance[35] = 0.1   # v_ang

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
