#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
STM32接口节点
功能：与STM32下位机建立串口或UDP通信，发送清洁任务指令，接收反馈
"""

import rclpy
from rclpy.node import Node
from perception_fusion.msg import CleaningTaskList, CleanTask
from std_msgs.msg import String
import serial
import socket
import struct
import threading
from typing import Optional, List
import time


class SerialProtocol:
    """串口通信协议"""
    
    # 帧格式: Header(0xAA55) + CMD_ID(1字节) + Payload长度(2字节) + Payload + CRC16(2字节)
    # 其中Header用于快速定位帧起始，CMD_ID区分命令类型，Payload承载业务数据，
    # CRC16用于校验帧完整性。
    HEADER = b'\xAA\x55'
    HEADER_SIZE = 2
    CMD_ID_SIZE = 1
    LENGTH_SIZE = 2
    CRC_SIZE = 2
    
    # 命令ID定义
    CMD_SEND_TASK = 0x01
    CMD_REQUEST_STATUS = 0x02
    CMD_CANCEL_TASK = 0x03
    CMD_EMERGENCY_STOP = 0x04
    
    # 响应ID
    RESP_TASK_ACK = 0x81
    RESP_STATUS = 0x82
    RESP_ERROR = 0xFF
    
    @staticmethod
    def calculate_crc16(data: bytes) -> int:
        """计算CRC16校验和"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF
    
    @staticmethod
    def pack_frame(cmd_id: int, payload: bytes) -> bytes:
        """打包数据帧

        将命令ID和载荷组合成协议帧，同时在尾部附加CRC16校验。
        """
        length = len(payload)
        frame = struct.pack('>BBH', 0xAA, 0x55, cmd_id)  # Header + CMD_ID
        frame += struct.pack('<H', length)  # Payload长度（小端）
        frame += payload
        crc = SerialProtocol.calculate_crc16(frame)
        frame += struct.pack('<H', crc)  # CRC16（小端）
        return frame
    
    @staticmethod
    def unpack_frame(data: bytes) -> Optional[tuple]:
        """解包数据帧

        验证包头与CRC并返回(cmd_id, payload)，错误时返回None。
        """
        if len(data) < SerialProtocol.HEADER_SIZE + SerialProtocol.CMD_ID_SIZE + SerialProtocol.LENGTH_SIZE:
            return None
        
        # 检查包头
        if data[0:2] != SerialProtocol.HEADER:
            return None
        
        cmd_id = data[2]
        length = struct.unpack('<H', data[3:5])[0]
        
        if len(data) < 5 + length + SerialProtocol.CRC_SIZE:
            return None
        
        payload = data[5:5+length]
        crc_received = struct.unpack('<H', data[5+length:5+length+SerialProtocol.CRC_SIZE])[0]
        crc_calculated = SerialProtocol.calculate_crc16(data[0:5+length])
        
        if crc_received != crc_calculated:
            return None
        
        return (cmd_id, payload)


class SerialInterface:
    """串口接口"""
    
    def __init__(self, port: str, baud_rate: int = 115200):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn: Optional[serial.Serial] = None
        self.is_connected = False
        self.receive_thread: Optional[threading.Thread] = None
        self.receive_callback = None
        # 接收缓冲区，用于拼接可能被拆分的串口数据帧
        self.buffer = bytearray()
    
    def connect(self) -> bool:
        """连接串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.is_connected = True
            # 启动接收线程
            # 守护线程持续读取串口并解析帧
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            return True
        except Exception as e:
            print(f"串口连接失败: {str(e)}")
            return False
    
    def disconnect(self):
        """断开串口"""
        self.is_connected = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
    
    def send_frame(self, cmd_id: int, payload: bytes) -> bool:
        """发送数据帧"""
        if not self.is_connected or not self.serial_conn:
            return False
        
        try:
            frame = SerialProtocol.pack_frame(cmd_id, payload)
            self.serial_conn.write(frame)
            return True
        except Exception as e:
            print(f"发送数据失败: {str(e)}")
            return False
    
    def set_receive_callback(self, callback):
        """设置接收回调"""
        self.receive_callback = callback
    
    def _receive_loop(self):
        """接收数据循环

        持续从串口读取数据，拼接到缓冲区后按协议切分完整帧并回调处理。
        """
        while self.is_connected and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    self.buffer.extend(data)
                    
                    # 查找完整帧
                    while len(self.buffer) >= SerialProtocol.HEADER_SIZE:
                        # 查找包头
                        # 如果缓冲区前面有脏数据则丢弃到下一个正确包头
                        header_idx = self.buffer.find(SerialProtocol.HEADER)
                        if header_idx == -1:
                            self.buffer.clear()
                            break
                        
                        if header_idx > 0:
                            self.buffer = self.buffer[header_idx:]
                        
                        if len(self.buffer) < SerialProtocol.HEADER_SIZE + SerialProtocol.CMD_ID_SIZE + SerialProtocol.LENGTH_SIZE:
                            break
                        
                        # 根据长度字段计算整个帧的长度（含CRC）
                        length = struct.unpack('<H', self.buffer[3:5])[0]
                        frame_size = 5 + length + SerialProtocol.CRC_SIZE
                        
                        if len(self.buffer) >= frame_size:
                            frame_data = bytes(self.buffer[:frame_size])
                            result = SerialProtocol.unpack_frame(frame_data)
                            if result and self.receive_callback:
                                self.receive_callback(result[0], result[1])
                            self.buffer = self.buffer[frame_size:]
                        else:
                            break
            except Exception as e:
                print(f"接收数据错误: {str(e)}")
                time.sleep(0.1)


class UDPInterface:
    """UDP接口

    用于在网络上与STM32通信，逻辑与串口类似，但无需拆分帧。
    """
    
    def __init__(self, local_port: int, remote_addr: str, remote_port: int):
        self.local_port = local_port
        self.remote_addr = remote_addr
        self.remote_port = remote_port
        self.sock: Optional[socket.socket] = None
        self.is_connected = False
        self.receive_thread: Optional[threading.Thread] = None
        self.receive_callback = None
    
    def connect(self) -> bool:
        """创建UDP socket"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(('', self.local_port))
            # 设置短超时，使接收循环可以响应线程退出
            self.sock.settimeout(0.1)
            self.is_connected = True
            # 启动接收线程
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            return True
        except Exception as e:
            print(f"UDP连接失败: {str(e)}")
            return False
    
    def disconnect(self):
        """关闭UDP socket"""
        self.is_connected = False
        if self.sock:
            self.sock.close()
    
    def send_data(self, data: bytes) -> bool:
        """发送数据"""
        if not self.is_connected or not self.sock:
            return False
        
        try:
            self.sock.sendto(data, (self.remote_addr, self.remote_port))
            return True
        except Exception as e:
            print(f"发送数据失败: {str(e)}")
            return False
    
    def set_receive_callback(self, callback):
        """设置接收回调"""
        self.receive_callback = callback
    
    def _receive_loop(self):
        """接收数据循环"""
        while self.is_connected and self.sock:
            try:
                data, addr = self.sock.recvfrom(1024)
                if self.receive_callback:
                    self.receive_callback(data)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"接收数据错误: {str(e)}")
                time.sleep(0.1)


class STM32InterfaceNode(Node):
    """STM32接口节点"""
    
    def __init__(self):
        super().__init__('stm32_interface_node')
        
        # 声明参数
        self.declare_parameter('communication_type', 'serial')  # 'serial' or 'udp'
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('serial_baud_rate', 115200)
        self.declare_parameter('udp_local_port', 8888)
        self.declare_parameter('udp_remote_addr', '192.168.1.100')
        self.declare_parameter('udp_remote_port', 8889)
        self.declare_parameter('task_topic', '/cleaning_task')
        self.declare_parameter('status_topic', '/stm32_status')
        
        # 获取参数
        comm_type = self.get_parameter('communication_type').get_parameter_value().string_value
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        serial_baud = self.get_parameter('serial_baud_rate').get_parameter_value().integer_value
        udp_local_port = self.get_parameter('udp_local_port').get_parameter_value().integer_value
        udp_remote_addr = self.get_parameter('udp_remote_addr').get_parameter_value().string_value
        udp_remote_port = self.get_parameter('udp_remote_port').get_parameter_value().integer_value
        task_topic = self.get_parameter('task_topic').get_parameter_value().string_value
        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        
        # 初始化通信接口
        self.interface = None
        if comm_type == 'serial':
            self.interface = SerialInterface(serial_port, serial_baud)
            self.interface.set_receive_callback(self.on_receive_data)
            if self.interface.connect():
                self.get_logger().info(f'串口连接成功: {serial_port}')
            else:
                self.get_logger().error(f'串口连接失败: {serial_port}')
        elif comm_type == 'udp':
            self.interface = UDPInterface(udp_local_port, udp_remote_addr, udp_remote_port)
            self.interface.set_receive_callback(self.on_receive_udp_data)
            if self.interface.connect():
                self.get_logger().info(f'UDP连接成功: {udp_remote_addr}:{udp_remote_port}')
            else:
                self.get_logger().error('UDP连接失败')
        else:
            self.get_logger().error(f'不支持的通信类型: {comm_type}')
        
        # 创建订阅者
        self.task_sub = self.create_subscription(
            CleaningTaskList, task_topic, self.task_callback, 10
        )
        
        # 创建发布者
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        
        self.get_logger().info('STM32接口节点已启动')
        self.get_logger().info(f'通信类型: {comm_type}')
        self.get_logger().info(f'订阅话题: {task_topic}')
        self.get_logger().info(f'发布话题: {status_topic}')
    
    def task_callback(self, msg: CleaningTaskList):
        """任务回调，发送任务给STM32"""
        if self.interface is None:
            return
        
        self.get_logger().info(f'收到 {len(msg.tasks)} 个清洁任务')
        
        # 打包任务数据
        for task in msg.tasks:
            # 任务数据格式: task_id(4字节) + x(4字节) + y(4字节) + cleaning_type(1字节) + priority(1字节)
            payload = struct.pack('<IffBB', 
                task.task_id,
                task.target_position.x,
                task.target_position.y,
                task.cleaning_type,
                task.priority
            )
            
            if isinstance(self.interface, SerialInterface):
                # 串口通信
                self.interface.send_frame(SerialProtocol.CMD_SEND_TASK, payload)
            elif isinstance(self.interface, UDPInterface):
                # UDP通信（简化格式）
                udp_data = struct.pack('>HH', 0xAA55, SerialProtocol.CMD_SEND_TASK) + payload
                self.interface.send_data(udp_data)
            
            self.get_logger().info(
                f'发送任务: ID={task.task_id}, '
                f'位置=({task.target_position.x:.2f}, {task.target_position.y:.2f})'
            )
    
    def on_receive_data(self, cmd_id: int, payload: bytes):
        """接收数据回调（串口）"""
        if cmd_id == SerialProtocol.RESP_TASK_ACK:
            # 任务确认
            if len(payload) >= 4:
                task_id = struct.unpack('<I', payload[0:4])[0]
                self.get_logger().info(f'收到任务确认: ID={task_id}')
        elif cmd_id == SerialProtocol.RESP_STATUS:
            # 状态反馈
            status_str = payload.decode('utf-8', errors='ignore')
            self.publish_status(status_str)
        elif cmd_id == SerialProtocol.RESP_ERROR:
            # 错误反馈
            error_code = payload[0] if len(payload) > 0 else 0
            self.get_logger().warn(f'收到错误反馈: 错误码={error_code}')
    
    def on_receive_udp_data(self, data: bytes):
        """接收UDP数据回调"""
        if len(data) < 4:
            return
        
        header = struct.unpack('>HH', data[0:4])
        if header[0] == 0xAA55:
            cmd_id = header[1]
            payload = data[4:]
            if cmd_id == SerialProtocol.RESP_STATUS:
                status_str = payload.decode('utf-8', errors='ignore')
                self.publish_status(status_str)
    
    def publish_status(self, status_str: str):
        """发布状态消息"""
        status_msg = String()
        status_msg.data = status_str
        self.status_publisher.publish(status_msg)
        self.get_logger().debug(f'发布状态: {status_str}')
    
    def destroy_node(self):
        """清理资源"""
        if self.interface:
            self.interface.disconnect()
            self.get_logger().info('通信接口已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STM32InterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

