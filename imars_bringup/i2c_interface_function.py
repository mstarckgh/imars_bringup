import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import struct
from smbus2 import SMBus



class I2cInterfaceNode(Node):
    def __init__(self):
        super.__init__('i2c_interface_node')

        # Abonnieren des /cmd_vel-Themas
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )