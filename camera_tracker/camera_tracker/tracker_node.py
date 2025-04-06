import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraTracker(Node):
    def __init__(self):
        super().__init__('camera_tracker')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape

        # Преобразуем в HSV для поиска объекта по цвету (например, красная футболка)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        mask = mask1

        # Поиск контуров
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        move = Twist()

        if contours:
            largest = max(contours, key=cv2.contourArea)
            (x, y, w_obj, h_obj) = cv2.boundingRect(largest)
            obj_x = x + w_obj / 2
            error = obj_x - w / 2

            move.linear.x = 0.1  # Вперед
            move.angular.z = -error / 200  # Поворот за объектом

            cv2.rectangle(frame, (x, y), (x + w_obj, y + h_obj), (0, 255, 0), 2)

        self.publisher.publish(move)

        cv2.imshow("Tracking", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
