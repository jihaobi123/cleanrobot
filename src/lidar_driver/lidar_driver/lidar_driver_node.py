#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LD14P激光雷达驱动节点
功能：通过串口读取LD14P雷达数据，解析并发布为LaserScan消息
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import serial
import struct
import math
from typing import Optional


class LidarDriverNode(Node):
    """LD14P激光雷达驱动节点"""

    # LD14P数据包结构
    PACKET_HEADER = b'\x54\x2C'
    PACKET_SIZE = 47  # 数据包大小
    ANGLE_MIN = 0.0
    ANGLE_MAX = 2 * math.pi
    RANGE_MIN = 0.12  # 最小检测距离(m)
    RANGE_MAX = 12.0  # 最大检测距离(m)

    def __init__(self):
        super().__init__('lidar_driver_node')
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 230400)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('topic_name', '/scan')
        self.declare_parameter('angle_min', self.ANGLE_MIN)
        self.declare_parameter('angle_max', self.ANGLE_MAX)
        self.declare_parameter('range_min', self.RANGE_MIN)
        self.declare_parameter('range_max', self.RANGE_MAX)
        self.declare_parameter('scan_time', 0.025)  # 40Hz
        self.declare_parameter('time_increment', 0.000625)  # 1/1600

        # 获取参数
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.angle_min = self.get_parameter('angle_min').get_parameter_value().double_value
        self.angle_max = self.get_parameter('angle_max').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        self.scan_time = self.get_parameter('scan_time').get_parameter_value().double_value
        self.time_increment = self.get_parameter('time_increment').get_parameter_value().double_value

        # 创建发布者
        self.scan_publisher = self.create_publisher(LaserScan, topic_name, 10)

        # 初始化串口
        self.serial_conn: Optional[serial.Serial] = None
        self.init_serial()

        # 数据缓冲区
        self.buffer = bytearray()
        self.ranges = [0.0] * 360  # 360度数据
        self.intensities = [0.0] * 360

        # 创建定时器，定期发布扫描数据
        self.timer = self.create_timer(0.025, self.publish_scan)  # 40Hz

        self.get_logger().info(f'LD14P雷达驱动节点已启动')
        self.get_logger().info(f'串口: {self.serial_port}, 波特率: {self.baud_rate}')
        self.get_logger().info(f'发布话题: {topic_name}, 坐标系: {self.frame_id}')

    def init_serial(self):
        """初始化串口连接"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.get_logger().info(f'串口 {self.serial_port} 打开成功')
        except Exception as e:
            self.get_logger().error(f'无法打开串口 {self.serial_port}: {str(e)}')
            self.serial_conn = None

    def parse_ld14p_packet(self, data: bytes) -> bool:
        """
        解析LD14P数据包
        数据包格式：Header(2字节) + Speed(2字节) + StartAngle(2字节) + Data(40字节) + EndAngle(2字节) + CRC(1字节)
        """
        if len(data) < self.PACKET_SIZE:
            return False

        # 检查包头
        if data[0:2] != self.PACKET_HEADER:
            return False

        try:
            # 解析速度 (低字节在前)
            speed = struct.unpack('<H', data[2:4])[0] / 100.0  # rpm

            # 解析起始角度 (低字节在前)
            start_angle = struct.unpack('<H', data[4:6])[0] / 100.0  # 度转弧度
            start_angle_rad = math.radians(start_angle)

            # 解析结束角度
            end_angle = struct.unpack('<H', data[44:46])[0] / 100.0
            end_angle_rad = math.radians(end_angle)

            # 解析12个点的数据 (每个点3字节: 距离低8位 + 距离高4位+强度高4位 + 强度低8位)
            points_data = data[6:42]
            angle_step = (end_angle_rad - start_angle_rad) / 12.0 if end_angle_rad > start_angle_rad else (end_angle_rad + 2*math.pi - start_angle_rad) / 12.0

            for i in range(12):
                idx = i * 3
                distance_low = points_data[idx]
                distance_high = (points_data[idx + 1] & 0x0F) << 8
                intensity_low = points_data[idx + 2]
                intensity_high = (points_data[idx + 1] & 0xF0) >> 4

                distance = (distance_high | distance_low) / 1000.0  # mm转m
                intensity = (intensity_high << 8) | intensity_low

                # 计算角度索引 (0-359)
                angle = start_angle_rad + i * angle_step
                angle_idx = int(math.degrees(angle)) % 360

                if 0 <= angle_idx < 360:
                    if self.range_min <= distance <= self.range_max:
                        self.ranges[angle_idx] = distance
                        self.intensities[angle_idx] = float(intensity)

            return True

        except Exception as e:
            self.get_logger().warn(f'解析数据包失败: {str(e)}')
            return False

    def read_serial_data(self):
        """读取串口数据并解析"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return

        try:
            # 读取可用数据
            if self.serial_conn.in_waiting > 0:
                data = self.serial_conn.read(self.serial_conn.in_waiting)
                self.buffer.extend(data)

                # 查找数据包
                while len(self.buffer) >= self.PACKET_SIZE:
                    # 查找包头
                    header_idx = self.buffer.find(self.PACKET_HEADER)
                    if header_idx == -1:
                        # 没有找到包头，清空缓冲区
                        self.buffer.clear()
                        break

                    # 移除包头之前的数据
                    if header_idx > 0:
                        self.buffer = self.buffer[header_idx:]

                    # 检查是否有完整数据包
                    if len(self.buffer) >= self.PACKET_SIZE:
                        packet = bytes(self.buffer[:self.PACKET_SIZE])
                        if self.parse_ld14p_packet(packet):
                            self.buffer = self.buffer[self.PACKET_SIZE:]
                        else:
                            # 解析失败，移除第一个字节继续查找
                            self.buffer = self.buffer[1:]
                    else:
                        break

        except Exception as e:
            self.get_logger().error(f'读取串口数据失败: {str(e)}')

    def publish_scan(self):
        """发布LaserScan消息"""
        # 读取并解析数据
        self.read_serial_data()

        # 创建LaserScan消息
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id

        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = (self.angle_max - self.angle_min) / len(self.ranges)
        scan_msg.time_increment = self.time_increment
        scan_msg.scan_time = self.scan_time
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max

        scan_msg.ranges = self.ranges.copy()
        scan_msg.intensities = self.intensities.copy()

        # 发布消息
        self.scan_publisher.publish(scan_msg)

    def destroy_node(self):
        """清理资源"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('串口已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

