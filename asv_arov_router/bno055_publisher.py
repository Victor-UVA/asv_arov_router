#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

class Imu_Publisher(Node):
    def __init__(self):
        super().__init__('bno055_gyro_publisher')
        self.publisher_ = self.create_publisher(Imu, '/bno055/gyro', 10)
        port = '/dev/ttyACM0'
        baud = 115200
        self.ser = serial.Serial(port, baud, timeout=1)
        self.get_logger().info("BNO055 Gyro Publisher started. Reading serial...")

    def parse_callback(self, line):
        try:
            parts = [x.strip() for x in line.split(',')]
            if len(parts) != 4:
                return None
            return [float(parts[1]), float(parts[2]), float(parts[3])]
        except:
            return None

    def gyro_callback(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        data = self.parse_ballback(line)
        if data:
            msg = Imu()
            msg.header = Header()
            msg.header.stamp = rclpy.time.Time()
            msg.header.frame_id = "bno055_link"
            msg.angular_velocity.x = data[0]
            msg.angular_velocity.y = data[1]
            msg.angular_velocity.z = data[2]
            self.publisher_.publish(msg)

def main():
    rclpy.init()
    imu_publisher = Imu_Publisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

