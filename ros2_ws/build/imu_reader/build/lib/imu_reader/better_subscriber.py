#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('imu_better_subscriber')
        topic_name = self.declare_parameter('topic_name', '/micro_ros_arduino_node_publisher').get_parameter_value().string_value
        self.subscription = self.create_subscription(
            Int32,
            topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused var warning
        self.get_logger().info(f"Subscribed to: {topic_name}")

    def listener_callback(self, msg):
        imu_value = msg.data
        # TODO: replace this with real processing logic!
        self.get_logger().info(f'Received IMU data: {imu_value}')

def main(args=None):
    rclpy.init(args=args)
    node = IMUSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
