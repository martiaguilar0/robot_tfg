import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

TICKS_PER_REV   = 1496
WHEEL_DIAMETER  = 0.08
WHEEL_BASE      = 0.2235
METERS_PER_TICK = (math.pi * WHEEL_DIAMETER) / TICKS_PER_REV


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inv_kinematics_node')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.motor_pub = self.create_publisher(Twist, '/motor_speed_target', 10)

    def cmd_vel_cb(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        v_r = v + (w * WHEEL_BASE / 2.0)
        v_l = v - (w * WHEEL_BASE / 2.0)

        ticks_s_r = v_r / METERS_PER_TICK
        ticks_s_l = v_l / METERS_PER_TICK

        motor_msg = Twist()
        motor_msg.linear.x = float(ticks_s_l)
        motor_msg.linear.y = float(ticks_s_r)
        self.motor_pub.publish(motor_msg)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
