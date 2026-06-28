import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf_transformations


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        self.state = np.zeros(5)
        self.P = np.eye(5) * 0.1

        self.Q      = np.diag([0.001, 0.001, 0.0005, 0.01, 0.01])
        self.R_odom = np.diag([0.01, 0.01])
        self.R_imu  = np.array([[9999]])

        self.latest_gyro_z = 0.0
        self.last_mcu_ns = None

        self.last_vision_time    = 0.0
        self.VISION_MIN_INTERVAL = 0.5

        self.pose_initialized = False

        self.filtered_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.create_subscription(Imu,                      '/imu/raw',     self.imu_cb,    10)
        self.create_subscription(Odometry,                 '/odom',         self.odom_cb,   10)
        self.create_subscription(PoseWithCovarianceStamped,'/vision/pose',  self.vision_cb, 10)
        self.create_timer(0.02, self.timer_predict)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def timer_predict(self):
        if self.last_mcu_ns is None:
            return
        now = self.get_clock().now()
        dt  = (self.last_mcu_ns - self._prev_mcu_ns) / 1e9
        self._prev_mcu_ns = self.last_mcu_ns
        if dt <= 0 or dt > 0.1:
            return

        x, y, th, v, w = self.state
        self.state[0] = x + v * math.cos(th) * dt
        self.state[1] = y + v * math.sin(th) * dt
        self.state[2] = self.normalize_angle(th + w * dt)

        F = np.eye(5)
        F[0, 2] = -v * math.sin(th) * dt
        F[1, 2] =  v * math.cos(th) * dt
        F[0, 3] =  math.cos(th) * dt
        F[1, 3] =  math.sin(th) * dt
        F[2, 4] =  dt

        self.P = F @ self.P @ F.T + self.Q
        self.publish_filtered(now.to_msg())

    def imu_cb(self, msg):
        mcu_ns = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
        if self.last_mcu_ns is None:
            self._prev_mcu_ns = mcu_ns
        self.last_mcu_ns = mcu_ns
        self.latest_gyro_z = msg.angular_velocity.z
        z = np.array([self.latest_gyro_z])
        H = np.array([[0, 0, 0, 0, 1]])
        self.apply_update(z, H, self.R_imu)

    def odom_cb(self, msg):
        z = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z
        ])
        H = np.zeros((2, 5))
        H[0, 3] = 1
        H[1, 4] = 1
        self.apply_update(z, H, self.R_odom)

    def vision_cb(self, msg):
        if not self.pose_initialized:
            self.state[0] = msg.pose.pose.position.x
            self.state[1] = msg.pose.pose.position.y
            self.pose_initialized = True
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_vision_time < self.VISION_MIN_INTERVAL:
            return

        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ])

        cov   = msg.pose.covariance
        var_x = max(cov[0], 1e-6)
        var_y = max(cov[7], 1e-6)
        R_dyn = np.diag([var_x, var_y])

        H = np.zeros((2, 5))
        H[0, 0] = 1
        H[1, 1] = 1
        innov = z - H @ self.state
        S_gate = H @ self.P @ H.T + R_dyn
        mahal_sq = float(innov @ np.linalg.inv(S_gate) @ innov)
        if mahal_sq > 9.21:
            return

        self.apply_update(z, H, R_dyn)
        self.last_vision_time = now

    def apply_update(self, z, H, R):
        y = z - (H @ self.state)

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.state    = self.state + K @ y
        self.state[2] = self.normalize_angle(self.state[2])
        self.P        = (np.eye(5) - K @ H) @ self.P

    def publish_filtered(self, stamp):
        msg = Odometry()
        msg.header.stamp     = stamp
        msg.header.frame_id  = 'odom'
        msg.child_frame_id   = 'base_link'
        msg.pose.pose.position.x = self.state[0]
        msg.pose.pose.position.y = self.state[1]
        q = tf_transformations.quaternion_from_euler(0, 0, self.state[2])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x  = self.state[3]
        msg.twist.twist.angular.z = self.state[4]
        self.filtered_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
