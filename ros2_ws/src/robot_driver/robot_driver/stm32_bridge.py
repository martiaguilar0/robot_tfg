import rclpy
from rclpy.node import Node
import serial
import struct
import threading
import numpy as np
from sensor_msgs.msg import Imu, MagneticField
from robot_msgs.msg import EncoderTicks
from std_msgs.msg import Int32MultiArray, Header
from geometry_msgs.msg import Twist

TELEM_FMT    = "<H I ii hhh hhh hhh HH B B"
TELEM_SIZE   = struct.calcsize(TELEM_FMT)
HEADER_TELEM = 0xABCD
HEADER_CMD   = 0xBBBB

ALPHA = 0.0001  # filtro paso bajo para el offset de tiempo

FLAG_MAGNET_UPDATED = 0x01
FLAG_BATT_UPDATED   = 0x02
FLAG_TRIGGER_FOTO   = 0x04

TIM3_MAX_US = 65536  # rango maximo TIM3 de 16 bits a 1MHz

class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge_node')

        self.ser = serial.Serial(port='/dev/ttyTHS1', baudrate=460800, timeout=0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.serial_lock = threading.Lock()

        self.time_offset_ns   = None
        self.last_stm32_us    = None
        self.reconstructed_us = 0

        self.imu_pub           = self.create_publisher(Imu,           '/imu/raw',        10)
        self.mag_pub           = self.create_publisher(MagneticField, '/mag/raw',        10)
        self.enc_pub           = self.create_publisher(EncoderTicks,  '/encoders/ticks', 10)
        self.photo_trigger_pub = self.create_publisher(Header,        '/photo_trigger',  10)

        self.create_subscription(Twist, '/motor_speed_target', self.motor_speed_target_cb, 10)

        self._buf = b''
        self.running = True
        self.read_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.read_thread.start()

    def update_time(self, stm32_us):
        ros_now_ns = self.get_clock().now().nanoseconds

        if self.last_stm32_us is None:
            self.reconstructed_us = 0
            self.time_offset_ns   = float(ros_now_ns)
            self.last_stm32_us    = stm32_us
            return

        dt_us = (stm32_us - self.last_stm32_us) % TIM3_MAX_US
        self.last_stm32_us = stm32_us

        if dt_us > 50_000:
            return

        self.reconstructed_us += dt_us
        raw_offset = ros_now_ns - self.reconstructed_us * 1000
        self.time_offset_ns = (1.0 - ALPHA) * self.time_offset_ns + ALPHA * raw_offset

    def get_ros_stamp(self):
        stamp_ns = int(self.time_offset_ns) + self.reconstructed_us * 1000
        t = rclpy.time.Time(nanoseconds=stamp_ns)
        return t.to_msg()

    def receive_loop(self):
        while rclpy.ok() and self.running:
            with self.serial_lock:
                if self.ser.in_waiting > 0:
                    self._buf += self.ser.read(self.ser.in_waiting)

            # ventana deslizante: busca header 0xABCD y valida checksum XOR
            while len(self._buf) >= TELEM_SIZE:
                if self._buf[0:2] != b'\xcd\xab':
                    self._buf = self._buf[1:]
                    continue
                pkt = self._buf[:TELEM_SIZE]
                received_cs = pkt[TELEM_SIZE - 1]
                computed_cs = 0
                for i in range(TELEM_SIZE - 1):
                    computed_cs ^= pkt[i]
                if received_cs == computed_cs:
                    self.parse_telemetry(pkt)
                    self._buf = self._buf[TELEM_SIZE:]
                else:
                    self._buf = self._buf[1:]

    def parse_telemetry(self, pkt):
        fields = struct.unpack(TELEM_FMT, pkt)
        if fields[0] != HEADER_TELEM:
            return

        stm32_us = fields[1] & 0xFFFF
        self.update_time(stm32_us)
        if self.time_offset_ns is None:
            return

        stamp = self.get_ros_stamp()

        # IMU
        imu_msg = Imu()
        imu_msg.header.stamp    = stamp
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.linear_acceleration.x = (fields[4] / 16384.0) * 9.806
        imu_msg.linear_acceleration.y = (fields[5] / 16384.0) * 9.806
        imu_msg.linear_acceleration.z = (fields[6] / 16384.0) * 9.806
        imu_msg.angular_velocity.x = np.radians(fields[7] / 131.0)
        imu_msg.angular_velocity.y = np.radians(fields[8] / 131.0)
        imu_msg.angular_velocity.z = np.radians(fields[9] / 131.0)
        self.imu_pub.publish(imu_msg)

        # magnetometro
        flags = fields[15]
        if flags & FLAG_MAGNET_UPDATED:
            mag_msg = MagneticField()
            mag_msg.header.stamp    = stamp
            mag_msg.header.frame_id = 'imu_link'
            mag_msg.magnetic_field.x = fields[10] * 1e-6
            mag_msg.magnetic_field.y = fields[11] * 1e-6
            mag_msg.magnetic_field.z = fields[12] * 1e-6
            self.mag_pub.publish(mag_msg)

        # encoders
        enc_msg = EncoderTicks()
        enc_msg.header.stamp    = stamp
        enc_msg.header.frame_id = 'base_link'
        enc_msg.enc_l = fields[3]
        enc_msg.enc_r = fields[2]
        self.enc_pub.publish(enc_msg)

        # trigger foto
        if flags & FLAG_TRIGGER_FOTO:
            trigger_msg = Header()
            trigger_msg.stamp    = stamp
            trigger_msg.frame_id = 'camera_link'
            self.photo_trigger_pub.publish(trigger_msg)

    def motor_speed_target_cb(self, msg: Twist):
        ticks_s_l = float(msg.linear.x)
        ticks_s_r = float(msg.linear.y)
        raw = struct.pack('<Hff', HEADER_CMD, ticks_s_l, ticks_s_r)
        chk = 0
        for b in raw:
            chk ^= b
        with self.serial_lock:
            self.ser.write(raw + bytes([chk]))


def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()
