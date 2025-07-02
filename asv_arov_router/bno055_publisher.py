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
            if len(parts) != 11:
                return None
            # timestamp = int(parts[0])
            ax = float(parts[1])
            ay = float(parts[2])
            az = float(parts[3])
            gx = float(parts[4])
            gy = float(parts[5])
            gz = float(parts[6])
            qx = float(parts[7])
            qy = float(parts[8])
            qz = float(parts[9])
            qw = float(parts[10])
            return [ax, ay, az], [gx, gy, gz], [qx, qy, qz, qw]
        except:
            return None

    def gyro_callback(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        data = self.parse_ballback(line)
        if data:
            accel, gyro, quat = data
            msg = Imu()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "bno055_link"
            msg.orientation.x = quat[0]
            msg.orientation.y = quat[1]
            msg.orientation.z = quat[2]
            msg.orientation.w = quat[3]
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]
            self.publisher_.publish(msg)

def main():
    rclpy.init()
    imu_publisher = Imu_Publisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

