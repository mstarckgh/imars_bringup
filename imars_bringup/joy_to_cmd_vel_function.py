import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToCmdVelNode(Node):
    def __init__(self):
        super().__init__('joy_to_cmd_vel')

        # Parameter deklarieren
        self.declare_parameter('axis_linear.x.forward', 5)
        self.declare_parameter('axis_linear.x.backward', 4)
        self.declare_parameter('scale_linear.x', 1.0)
        self.declare_parameter('offset_linear.x', 0.0)
        self.declare_parameter('axis_angular.yaw', 0)
        self.declare_parameter('scale_angular.yaw', 1.0)

        # Parameter auslesen
        self.axis_linear_x_fw = self.get_parameter('axis_linear.x.forward').value
        self.axis_linear_x_bw = self.get_parameter('axis_linear.x.backward').value
        self.scale_linear_x = self.get_parameter('scale_linear.x').value
        self.offset_linear_x = self.get_parameter('offset_linear.x').value
        self.axis_angular_yaw = self.get_parameter('axis_angular.yaw').value
        self.scale_angular_yaw = self.get_parameter('scale_angular.yaw').value

        # /joy Subscriber und /cmd_vel Publisher
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def joy_callback(self, msg):
        twist = Twist()

        # Achse für lineare Geschwindigkeit transformieren
        linear_input_x_fw = -0.5*msg.axes[self.axis_linear_x_fw] + 0.5    # Trigger für vorwärtsfahren auf [0.0; 1.0] trimmen
        linear_input_x_bw = 0.5*msg.axes[self.axis_linear_x_bw] - 0.5    # Trigger für rückwärtsfahren auf [-1.0; 0.0] trimmen
        linear_input_x = linear_input_x_fw + linear_input_x_bw

        twist.linear.x = (linear_input_x * self.scale_linear_x) + self.offset_linear_x

        # Achse für Drehgeschwindigkeit
        twist.angular.z = msg.axes[self.axis_angular_yaw] * self.scale_angular_yaw

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
