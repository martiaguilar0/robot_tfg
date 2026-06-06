import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
import threading
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory

MARKER_SIZE = 0.06  # metros

camera_matrix = np.array([
    [1092.0583, 0, 707.1624],
    [0, 1095.3598, 340.6184],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.array([[-0.124210, 0.732742, -0.000377, 0.000847, -1.459907]], dtype=np.float32)

pipeline_str = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
    "nvvidconv ! video/x-raw, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink name=sink emit-signals=True max-buffers=1 drop=True"
)

MAX_DETECTION_DIST = 1.0   # m
MAX_LATERAL_ANGLE  = 30.0  # grados

MARKER_STOP_DIST  = 0.50  # m — distancia a la que el robot para a medir
STOP_SETTLE_TIME  = 0.50  # s — espera para que el IMU se asiente tras el stop
N_MEASURE_FRAMES  = 10    # frames a promediar para la correccion
COOLDOWN_DURATION = 20.0  # s — tiempo minimo entre correcciones

MEASURE_VAR_BASE = 0.005  # m² — varianza minima de la medicion


class VisionLocalizationNode(Node):
    def __init__(self):
        super().__init__('vision_localization_node')

        self.pose_pub      = self.create_publisher(PoseWithCovarianceStamped, '/vision/pose',      10)
        self.measuring_pub = self.create_publisher(Bool,                      '/vision/measuring', 10)

        # estados: IDLE, STOPPING, MEASURING, COOLDOWN
        self.vision_state        = 'IDLE'
        self.measure_accumulator = []
        self.stop_start_time     = None
        self.last_measure_time   = 0.0

        self.create_timer(0.1, self._state_timer_cb)

        self.marker_positions = {}
        try:
            share_dir = get_package_share_directory('robot_driver')
            config_path = os.path.join(share_dir, 'config', 'navigation_config.yaml')
        except Exception:
            config_path = os.path.join(os.path.dirname(__file__), 'navigation_config.yaml')

        try:
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                raw_markers = config.get('markers', {})
                self.marker_positions = {int(k): v for k, v in raw_markers.items()}
            else:
                self.get_logger().error(f'No se encontro el archivo de configuracion en {config_path}')
        except Exception as e:
            self.get_logger().error(f'Error leyendo YAML: {e}')

        # offset camara respecto base_link [x_front, y_lat, z_up] en metros
        self.cam_offset = np.array([0.12, 0.03, 0.115])

        self.running = True
        try:
            Gst.init(None)
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsink = self.pipeline.get_by_name("sink")
            self.pipeline.set_state(Gst.State.PLAYING)

            self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
            self.parameters = aruco.DetectorParameters_create()

            self.parameters.adaptiveThreshWinSizeMin = 3
            self.parameters.adaptiveThreshWinSizeMax = 71
            self.parameters.adaptiveThreshWinSizeStep = 4
            self.parameters.adaptiveThreshConstant = 9.0
            self.parameters.minMarkerPerimeterRate = 0.02
            self.parameters.maxMarkerPerimeterRate = 4.0
            self.parameters.polygonalApproxAccuracyRate = 0.03
            self.parameters.perspectiveRemovePixelPerCell = 8
            self.parameters.perspectiveRemoveIgnoredMarginPerCell = 0.13

            self.camera_thread = threading.Thread(target=self.capture_loop, daemon=True)
            self.camera_thread.start()
        except Exception as e:
            self.get_logger().error(f'Error inicializando hardware: {e}')

    def capture_loop(self):
        while rclpy.ok() and self.running:
            try:
                sample = self.appsink.emit("pull-sample")
                if not sample:
                    continue
                buf  = sample.get_buffer()
                caps = sample.get_caps()
                data = buf.extract_dup(0, buf.get_size())
                frame = np.frombuffer(data, dtype=np.uint8).reshape(
                    caps.get_structure(0).get_value('height'),
                    caps.get_structure(0).get_value('width'), 3
                )
                self.process_frame(frame)
            except Exception:
                pass

    def _state_timer_cb(self):
        msg = Bool()
        msg.data = self.vision_state in ('STOPPING', 'MEASURING')
        self.measuring_pub.publish(msg)

        if self.vision_state == 'COOLDOWN':
            elapsed = self.get_clock().now().nanoseconds / 1e9 - self.last_measure_time
            if COOLDOWN_DURATION - elapsed <= 0:
                self.vision_state = 'IDLE'

    def process_frame(self, frame):
        if self.vision_state == 'COOLDOWN':
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )

        if ids is None:
            if self.vision_state == 'STOPPING':
                self.vision_state = 'IDLE'
                self.stop_start_time = None
            return

        res = aruco.estimatePoseSingleMarkers(
            corners, MARKER_SIZE, camera_matrix, dist_coeffs
        )
        rvecs, tvecs = res[0], res[1]

        best      = None
        best_dist = float('inf')

        for i in range(len(ids)):
            marker_id = ids[i][0]
            if marker_id not in self.marker_positions:
                continue

            x_m_cam = tvecs[i][0][0]
            z_m_cam = tvecs[i][0][2]
            dist    = math.sqrt(x_m_cam**2 + z_m_cam**2)

            if dist > MAX_DETECTION_DIST:
                continue

            lateral_angle_deg = math.degrees(math.atan2(abs(x_m_cam), z_m_cam))
            if lateral_angle_deg > MAX_LATERAL_ANGLE:
                continue

            if dist < best_dist:
                best_dist = dist
                best = (i, marker_id, x_m_cam, z_m_cam, dist, lateral_angle_deg)

        if best is None:
            if self.vision_state == 'STOPPING':
                self.vision_state = 'IDLE'
                self.stop_start_time = None
            return

        i, marker_id, x_m_cam, z_m_cam, dist, lateral_angle_deg = best
        m_abs_x, m_abs_y, m_abs_theta = self.marker_positions[marker_id]

        rvec = rvecs[i][0]
        rotation_matrix, _ = cv2.Rodrigues(rvec)

        cos_m = math.cos(m_abs_theta)
        sin_m = math.sin(m_abs_theta)
        cam_x_world = m_abs_x - (cos_m * z_m_cam + sin_m * x_m_cam)
        cam_y_world = m_abs_y - (sin_m * z_m_cam - cos_m * x_m_cam)

        cam_yaw       = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        cam_abs_theta = m_abs_theta + cam_yaw

        cos_t = math.cos(cam_abs_theta)
        sin_t = math.sin(cam_abs_theta)
        off_x_world = self.cam_offset[0] * cos_t - self.cam_offset[1] * sin_t
        off_y_world = self.cam_offset[0] * sin_t + self.cam_offset[1] * cos_t

        robot_x = cam_x_world - off_x_world
        robot_y = cam_y_world - off_y_world

        if self.vision_state == 'IDLE':
            if dist <= MARKER_STOP_DIST:
                self.vision_state    = 'STOPPING'
                self.stop_start_time = self.get_clock().now().nanoseconds / 1e9
                self.measure_accumulator = []
            return

        if self.vision_state == 'STOPPING':
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.stop_start_time >= STOP_SETTLE_TIME:
                self.vision_state = 'MEASURING'
            return

        if self.vision_state == 'MEASURING':
            self.measure_accumulator.append((robot_x, robot_y))
            if len(self.measure_accumulator) >= N_MEASURE_FRAMES:
                self._publish_averaged_pose(cam_abs_theta)

    def _publish_averaged_pose(self, theta):
        xs = [p[0] for p in self.measure_accumulator]
        ys = [p[1] for p in self.measure_accumulator]
        avg_x = float(np.mean(xs))
        avg_y = float(np.mean(ys))
        std_x = float(np.std(xs))
        std_y = float(np.std(ys))

        # max(base, dispersion real) para no sobreestimar la precision
        var_x = max(MEASURE_VAR_BASE, std_x ** 2)
        var_y = max(MEASURE_VAR_BASE, std_y ** 2)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp    = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x    = avg_x
        pose_msg.pose.pose.position.y    = avg_y
        pose_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        cov = [0.0] * 36
        cov[0]  = var_x
        cov[7]  = var_y
        cov[14] = 9999.0
        cov[21] = 9999.0
        cov[28] = 9999.0
        cov[35] = 9999.0
        pose_msg.pose.covariance = cov

        self.pose_pub.publish(pose_msg)

        self.last_measure_time = self.get_clock().now().nanoseconds / 1e9
        self.vision_state = 'COOLDOWN'
        self.measure_accumulator = []

    def destroy_node(self):
        self.running = False
        self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
