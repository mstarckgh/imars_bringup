import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import math
import serial

class ControllerInterfaceNode(Node):
    def __init__(self):
        super().__init__('controller_interface_node')

        # Parameter des Fahrzeugs
        self.wheelbase = 0.5
        serial_port = '/dev/ttyUSB1'
        baud_rate = 115200

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
        self.send_to_controller(linear_velocity, steering_angle)

    def send_to_controller(self, velocity, steering_angle):
        try:
            # Formatierung der Nachricht: "v:<velocity>,a:<steering_angle>\n"
            message = f'{velocity:.2f},{math.degrees(steering_angle):.2f}\n'
            self.serial_conn.write(message.encode('utf-8'))
            self.get_logger().info(f'Sent to controller: {message.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send data to controller: {e}')
        
    def destroy_node(self):
        # Schließen der seriellen Verbindung beim Beenden
        if self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ControllerInterfaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()