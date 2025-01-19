import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import struct
import math
from smbus2 import SMBus
import lgpio
from time import sleep



class I2cInterfaceNode(Node):
    def __init__(self):
        super().__init__('i2c_interface_node')

        # I2C Server Address
        self.i2c_server_address = 0x08
        self.bus = SMBus(1)
        self.scl_pin = 3
        self.sda_pin = 2

        # GPIO-Chip initialisieren
        self.chip = lgpio.gpiochip_open(0)  # GPIO-Chip 0 auf Raspberry Pi
        
        # SCL als Ausgang, SDA als Eingang konfigurieren
        lgpio.gpio_claim_output(self.chip, self.scl_pin)
        lgpio.gpio_claim_input(self.chip, self.sda_pin)

        # Parameter des Fahrzeugs
        self.wheelbase = 0.5
        self.linear_velocity = 0.0
        self.steering_angle = 0.0

        # Timer zum Senden der Daten an Controller
        self.timer = self.create_timer(0.2, self.send_update)

        # Abonnieren des /cmd_vel-Themas
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Ackermann-Berechnung
        if angular_velocity == 0.0 or self.linear_velocity == 0.0:
            self.steering_angle = 0.0
        else:
            turning_radius = self.linear_velocity / angular_velocity
            self.steering_angle = math.atan(self.wheelbase / turning_radius)

    def send_update(self):
        # Loggen der berechneten Werte
        self.get_logger().info(f'Linear Velocity: {self.linear_velocity:.2f} m/s, Steering Angle: {math.degrees(self.steering_angle):.2f} degrees')

        # Daten an den Controller senden
        try:
            self.write_float_to_register(0x00, self.linear_velocity)
            self.write_float_to_register(0x01, self.steering_angle)
        except Exception as e:
            self.get_logger().error(f"I²C communication error: {e}")
            self.reset_i2c_bus()


    def write_float_to_register(self, register:int, value:float)->None:
        float_bytes = list(struct.pack('f', value))
        self.bus.write_i2c_block_data(self.i2c_server_address, register, float_bytes)
    
    def read_float_from_register(self, register:int)->float:
        float_bytes = self.bus.read_i2c_block_data(self.i2c_server_address, register, 4)
        return struct.unpack('f', bytes(float_bytes))[0]
    
    def reset_i2c_bus(self):
        self.get_logger().info("Resetting I²C bus")

        # Pulsiere SCL 9-mal
        for _ in range(9):
            lgpio.gpio_write(self.chip, self.scl_pin, 1)
            sleep(0.001)
            lgpio.gpio_write(self.chip, self.scl_pin, 0)
            sleep(0.001)
            
        # Überprüfe, ob SDA freigegeben ist
        sda_state = lgpio.gpio_read(self.chip, self.sda_pin)
        if sda_state == 0:
            self.get_logger().error("SDA is still held LOW. Manual intervention required.")
        else:
            self.get_logger().info("I²C bus successfully reset.")


    

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
