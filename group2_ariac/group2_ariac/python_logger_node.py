#!/usr/bin/env python3

# Rey Roque-Perez - Used this to debug python nodes not building

import rclpy
from rclpy.node import Node

class PythonLoggerNode(Node):
    def __init__(self):
        super().__init__('python_logger_node')
        self.get_logger().info('I am a Python node')

def main(args=None):
    rclpy.init(args=args)
    node = PythonLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
