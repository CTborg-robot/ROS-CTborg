import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('serial_motor_driver_node')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        left = int((msg.linear.x - msg.angular.z) * 200)
        right = int((msg.linear.x + msg.angular.z) * 200)

        left = max(-255, min(255, left))
        right = max(-255, min(255, right))

        cmd = f"L{left:+03d}R{right:+03d}\n"
        self.ser.write(cmd.encode())
        self.get_logger().info(f'Sent: {cmd.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

