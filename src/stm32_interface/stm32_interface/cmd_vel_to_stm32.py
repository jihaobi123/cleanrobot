#!/usr/bin/env python3
import time
import struct
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial


def crc16_ccitt_false(data: bytes, poly=0x1021, init=0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ poly
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


class CmdVelToSTM32(Node):
    """
    Subscribe: /cmd_vel (geometry_msgs/Twist)
    Send UART frames to STM32 with vx, wz
    Safety:
      - watchdog timeout => send 0,0
      - clamp max vx/wz
      - periodic send at send_rate_hz
    """

    def __init__(self):
        super().__init__('cmd_vel_to_stm32')

        # ===== parameters =====
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('send_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_s', 0.3)
        self.declare_parameter('max_vx_m_s', 0.6)
        self.declare_parameter('max_wz_rad_s', 2.0)
        self.declare_parameter('topic_cmd_vel', '/cmd_vel')

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.send_rate_hz = float(self.get_parameter('send_rate_hz').value)
        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)
        self.max_vx = float(self.get_parameter('max_vx_m_s').value)
        self.max_wz = float(self.get_parameter('max_wz_rad_s').value)
        self.topic_cmd_vel = str(self.get_parameter('topic_cmd_vel').value)

        # latest cmd
        self._vx = 0.0
        self._wz = 0.0
        self._last_cmd_time = 0.0
        self._lock = threading.Lock()

        # serial open
        self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
        self.get_logger().info(f'Opened serial {self.port} @ {self.baud}')

        # sub
        self.sub = self.create_subscription(Twist, self.topic_cmd_vel, self.cb_cmd, 10)

        # timer loop
        period = 1.0 / max(self.send_rate_hz, 1e-6)
        self.timer = self.create_timer(period, self.loop)

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

    def cb_cmd(self, msg: Twist):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)

        # clamp
        vx = max(-self.max_vx, min(self.max_vx, vx))
        wz = max(-self.max_wz, min(self.max_wz, wz))

        with self._lock:
            self._vx = vx
            self._wz = wz
            self._last_cmd_time = time.time()

    def build_frame(self, vx_m_s: float, wz_rad_s: float) -> bytes:
        # convert to int32 mm/s and mrad/s
        vx_mm_s = int(round(vx_m_s * 1000.0))
        wz_mrad_s = int(round(wz_rad_s * 1000.0))

        header = b'\xAA\x55'
        version = 0x01
        msg_id = 0x10
        payload = struct.pack('<ii', vx_mm_s, wz_mrad_s)  # little endian
        length = len(payload)  # 8

        body = struct.pack('<BBB', version, msg_id, length) + payload
        crc = crc16_ccitt_false(header + body)
        frame = header + body + struct.pack('<H', crc)
        return frame

    def loop(self):
        now = time.time()
        with self._lock:
            age = now - self._last_cmd_time
            vx = self._vx
            wz = self._wz

        if age > self.cmd_timeout_s:
            vx, wz = 0.0, 0.0

        frame = self.build_frame(vx, wz)

        try:
            self.ser.write(frame)
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')


def main():
    rclpy.init()
    node = CmdVelToSTM32()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

