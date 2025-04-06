import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

class LidarReceiver(Node):
    def __init__(self):
        super().__init__('lidar_receiver')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'lidar_data', 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_data)

    def read_data(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith('LIDAR:') and line.endswith(';'):
                data_str = line[6:-1]  # убираем LIDAR: и ;
                try:
                    distances = [int(x) for x in data_str.split(',')]
                    msg = Int32MultiArray()
                    msg.data = distances
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Data: {distances}')
                except:
                    self.get_logger().warn('Parsing error')

def main(args=None):
    rclpy.init(args=args)
    node = LidarReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

