#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class IMUSerialReader(Node):
    def __init__(self):
        super().__init__('imu_serial_reader')
        self.publisher_ = self.create_publisher(String, 'imu_raw', 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: {line}')
            except Exception as e:
                self.get_logger().error(f"Error reading serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUSerialReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
