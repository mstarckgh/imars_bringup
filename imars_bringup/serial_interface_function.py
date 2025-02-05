import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import math
import serial
import struct
from time import sleep


class SerialInterfaceNode(Node):
    def __init__(self):
        super().__init__('serial_interface_node')

        self.timer = self.create_timer(0.05, self.send_to_controller)

        # Parameter des Fahrzeugs
        self.wheelbase = 0.5
        self.throttle = 0.0
        self.steering_angle = 0.0

        serial_port = '/dev/ttyUSB0'
        baud_rate = 9600

        # Initialisierung der seriellen Verbindung
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f'Serial connection established on {serial_port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # Abonnieren des /cmd_vel-Themas
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        self.throttle = msg.linear.x
        angular_velocity = msg.angular.z

        # Ackermann-Berechnung
        if angular_velocity == 0.0 or self.throttle == 0.0:
            self.steering_angle = 0.0
        else:
            turning_radius = self.throttle / angular_velocity
            self.steering_angle = math.atan(self.wheelbase / turning_radius)

        # Loggen der berechneten Werte
        self.get_logger().info(f'Linear Velocity: {self.throttle:.2f} m/s, Steering Angle: {math.degrees(self.steering_angle):.2f} degrees')

    def send_to_controller(self):
        try:
            self.send_float(0x00, self.throttle)
            self.send_float(0x01, math.degrees(self.steering_angle))
            # self.get_logger().info(f'Sent to controller')
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