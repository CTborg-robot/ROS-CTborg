import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import struct
import math

class DreameLidar(Node):
    def __init__(self):
        super().__init__('dreame_lidar')
        self.ser = serial.Serial('/dev/ttyUSB0', 230400, timeout=1)
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.05, self.read_data)

    def read_data(self):
        if self.ser.in_waiting:
            data = self.ser.read(self.ser.in_waiting)
            # Тут нужен парсер протокола Dreame D9/F9
            # Можно сдампить сырые данные и вместе разберём их
            print(data)

def main(args=None):
    rclpy.init(args=args)
    node = DreameLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
