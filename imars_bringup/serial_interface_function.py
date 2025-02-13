import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import math
import serial
import struct
from time import sleep
from threading import Thread


class SerialInterfaceNode(Node):
    def __init__(self):
        super().__init__('serial_interface_node')

        self.timer = self.create_timer(0.02, self.send_to_controller)
        self.cmd_velocity = 0.0
        self.cmd_steering_angle = 0.0

        serial_port = '/dev/ttyUSB0'
        baud_rate = 9600

        # Initialisierung der seriellen Verbindung
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f'Serial connection established on {serial_port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # Abonnieren des /imars_lite/ackermann_control - Themas
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/imars_lite/ackermann_control',
            self.control_callback,
            10
        )

    def control_callback(self, msg:Float32MultiArray):
        self.cmd_velocity = msg.data[0]
        self.cmd_steering_angle = msg.data[1]

    def send_to_controller(self):
        try:
            self.send_float(0x00, self.cmd_velocity)
            self.send_float(0x01, self.cmd_steering_angle)
            #self.get_logger().info(f'Send: {self.velocity}, {self.steering_angle}')
            #self.get_logger().info(f'Sent to controller')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send data to controller: {e}')
    
    def send_float(self, register_address:int, value:float):
        start_byte = 0xAA
        data_bytes = struct.pack('f', value)
        checksum = (register_address + sum(data_bytes)) % 256
        
        data = bytearray([start_byte, register_address]) + data_bytes + bytearray([checksum, ord('\n')])

        self.serial_conn.write(data)
        sleep(0.02)
    
        
    def destroy_node(self):
        # Schließen der seriellen Verbindung beim Beenden
        if self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SerialInterfaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()