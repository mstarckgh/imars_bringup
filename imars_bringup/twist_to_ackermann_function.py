import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray  # Alternativ: ein Array für Geschwindigkeit und Lenkwinkel


from math import atan, atan2, pi, degrees


class TwistToAckermannNode(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann_node')

        # Parameter des Fahrzeugs
        self.wheelbase = 0.4
        self.reference_offset = 0.0
        self.velocity = 0.0
        self.steering_angle = 0.0 

        # Publisher für Geschwindigkeit und Lenkwinkel
        self.publisher = self.create_publisher(Float32MultiArray, '/imars_lite/ackermann_control', 10)


        # Abonnieren des /cmd_vel-Themas
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg:Twist):
        effective_velocity = msg.linear.x - abs(msg.angular.z)*self.reference_offset
        if msg.linear.x != 0:
            self.velocity = effective_velocity
            self.steering_angle = atan(msg.angular.z * self.wheelbase / effective_velocity)
        elif msg.linear.x == 0 and msg.angular.z == 1:
            self.steering_angle = pi/2 + 0.03
            self.velocity = 0.2
        else:
            self.velocity = 0.0
            self.steering_angle = 0.0

        # Erstellen der Nachricht
        control_msg = Float32MultiArray()
        control_msg.data = [self.velocity, degrees(self.steering_angle)]

        # Veröffentlichen der Nachricht
        self.publisher.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)

    node = TwistToAckermannNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()