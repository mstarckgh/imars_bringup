import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray  # Alternativ: ein Array für Geschwindigkeit und Lenkwinkel


from math import atan, atan2, pi, degrees, sqrt


class TwistToAckermannNode(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann_node')

        # Parameter des Fahrzeugs
        self.wheelbase = 0.41
        self.wheelbase_front = 0.124
        self.wheelbase_rear = 0.286
        self.weight = 25
        self.c_alpha = 300
        
        self.target_velocity = 0.0
        self.target_steering_angle = 0.0

        self.vx = 0
        self.vy = 0
        self.wz = 0

        # Publisher für Geschwindigkeit und Lenkwinkel
        self.publisher = self.create_publisher(Float32MultiArray, '/imars_lite/ackermann_control', 10)


        # Abonnieren des /cmd_vel-Themas
        self.cmd_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg:Twist):
        if msg.linear.x != 0 and msg.angular.z != 0:
            self.velocity = msg.linear.x
            rho = msg.linear.x / msg.angular.z
            self.steering_angle = self.wheelbase / rho
            #v =  sqrt(self.vx**2 + self.vy**2)
            #self.steering_angle = self.wheelbase/rho + self.weight/self.wheelbase * (self.wheelbase_rear/self.c_alpha - self.wheelbase_front/self.c_alpha) * (v**2)/rho
            if self.steering_angle > pi/4:
                self.steering_angle = pi/4
            elif self.steering_angle < -pi/4:
                self.steering_angle = -pi/4
        elif msg.linear.x == 0 and msg.angular.z == 1:
            self.steering_angle = pi/2 + 0.03
            self.velocity = 0.2
        else:
            self.velocity = msg.linear.x
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