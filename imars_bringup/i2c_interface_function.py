import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import struct
import math
from smbus2 import SMBus



class I2cInterfaceNode(Node):
    def __init__(self):
        super.__init__('i2c_interface_node')

        # I2C Server Address
        self.i2c_server_address = 0x08
        self.bus = SMBus(1)

        # Parameter des Fahrzeugs
        self.wheelbase = 0.5

        # Abonnieren des /cmd_vel-Themas
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Ackermann-Berechnung
        if angular_velocity == 0.0 or linear_velocity == 0.0:
            steering_angle = 0.0
        else:
            turning_radius = linear_velocity / angular_velocity
            steering_angle = math.atan(self.wheelbase / turning_radius)

        # Loggen der berechneten Werte
        self.get_logger().info(f'Linear Velocity: {linear_velocity:.2f} m/s, Steering Angle: {math.degrees(steering_angle):.2f} degrees')

        # Daten an den Controller senden
        self.write_float_to_register(0x00, linear_velocity)
        self.write_float_to_register(0x01, steering_angle)

        # Debugg Abfrage, ob auch alles richtig gesendet wurde
        debugg_angle = self.read_float_from_register(0x01)
        debugg_velocity = self.read_float_from_register(0x00)
        self.get_logger().info(f'Updated Velocity: {linear_velocity:.2f} m/s, Updated Angle: {math.degrees(steering_angle):.2f} degrees')



    def write_float_to_register(self, register:int, value:float)->None:
        float_bytes = list(struct.pack('f', value))
        self.bus.write_i2c_block_data(self.i2c_server_address, register, float_bytes)
    
    def read_float_from_register(self, register:int)->float:
        float_bytes = self.bus.read_i2c_block_data(self.i2c_server_address, register, 4)

        return struct.unpack('f', bytes(float_bytes))[0]
    

def main(args=None):
    rclpy.init(args=args)

    node = I2cInterfaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
