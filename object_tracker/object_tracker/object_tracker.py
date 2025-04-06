import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.subscription = self.create_subscription(
            Image,
            '/v4l2_camera/image_raw',
            self.image_callback,
            10)
        self.subscription
        self.bridge = CvBridge()
        self.get_logger().info("Object Tracker is ready.")

    def image_callback(self, msg):
        try:
            # Преобразуем ROS Image в OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Простой пример отслеживания объекта: детекция цветового диапазона
            # Предположим, что мы ищем синий объект
            lower_blue = (100, 0, 0)
            upper_blue = (140, 255, 255)

            # Преобразуем изображение в HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Маска для синего цвета
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            # Нахождение контуров
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                if cv2.contourArea(contour) > 1000:  # Минимальная площадь объекта для отслеживания
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Показываем изображение
            cv2.imshow("Tracked Image", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

