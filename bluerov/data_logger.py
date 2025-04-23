import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
import csv
import json
import os

from geometry_msgs.msg import AccelStamped, TwistStamped
from nav_msgs.msg import Odometry
from apriltag_msgs.msg import AprilTagDetectionArray

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class Data_Logger(Node):
    def __init__(self):
        super().__init__('data_logger')

        self.bluerov_accel = [0.0,0.0,0.0]
        self.bluerov_gyro_rates = [0.0,0.0,0.0]
        self.bluerov_pose_estimate = [0.0,0.0,0.0]
        self.bluerov_yaw_estimate = 0.0

        self.maddy_yaw_rate = 0.0
        self.maddy_pose_estimate = [0.0,0.0,0.0]
        self.maddy_yaw_estimate = 0.0

        self.apriltag_pose = [0.0, 0.0]
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.LOG_FILE = os.path.join('data_log', 'sensor_data', f'{self.get_clock().now().seconds_nanoseconds()[0]}_data.csv')
        data_logging_period = 0.02

        # apriltag x and y are in the BlueROV's coordinate frame
        self.fields = ['timestep','uuv_x','uuv_y','uuv_z','uuv_psi','usv_x','usv_y','usv_psi','gps_x','gps_y','uuv_compass','usv_compass','uuv_gyro','usv_gyro', 'uuv_acc_x', 'uuv_acc_y', 'uuv_acc_z', 'apriltag_x', 'apriltag_y']
        with open(self.LOG_FILE, 'w') as csv_file:
            csv_writer =csv.DictWriter(csv_file, fieldnames=self.fields)
            csv_writer.writeheader()

        # BlueROV subscriptions
        self.bluerov_accel_sub = self.create_subscription(
            AccelStamped,
            'bluerov/accel',
            self.bluerov_accel_callback,
            10)
        self.bluerov_accel_sub  # prevent unused variable warning

        self.bluerov_gyro_rates_sub = self.create_subscription(
            TwistStamped,
            'bluerov/gyro_rates',
            self.bluerov_gyro_rates_callback,
            10)
        self.bluerov_gyro_rates_sub  # prevent unused variable warning

        self.bluerov_pose_sub = self.create_subscription(
            Odometry,
            'bluerov/pose_from_dvl',
            self.bluerov_pose_callback,
            10)
        self.bluerov_pose_sub  # prevent unused variable warning

        self.bluerov_apriltag_detect_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.bluerov_apriltag_detect_callback,
            10)
        self.bluerov_apriltag_detect_sub  # prevent unused variable warning


        # USV subscriptions
        self.maddy_yaw_rate_sub = self.create_subscription(
            TwistStamped,
            'maddy/yaw_rate',
            self.maddy_yaw_rate_callback,
            10)
        self.maddy_yaw_rate_sub  # prevent unused variable warning

        self.maddy_pose_sub = self.create_subscription(
            Odometry,
            'maddy/pose',
            self.maddy_pose_callback,
            10)
        self.maddy_pose_sub  # prevent unused variable warning

        self.create_timer(data_logging_period, self.log_data)

    def bluerov_accel_callback(self, msg):
        self.bluerov_accel[0] = msg.accel.linear.x
        self.bluerov_accel[1] = msg.accel.linear.y
        self.bluerov_accel[2] = msg.accel.linear.z

    def bluerov_gyro_rates_callback(self, msg):
        self.bluerov_gyro_rates[0] = msg.twist.angular.x
        self.bluerov_gyro_rates[1] = msg.twist.angular.y
        self.bluerov_gyro_rates[2] = msg.twist.angular.z

    def bluerov_pose_callback(self, msg):
        self.bluerov_pose_estimate[0] = msg.pose.pose.position.x
        self.bluerov_pose_estimate[1] = msg.pose.pose.position.y
        self.bluerov_pose_estimate[2] = msg.pose.pose.position.z

        rot = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.bluerov_yaw_estimate = Rotation.from_quat(rot).as_euler('xyz', degrees=True)[2]

    def bluerov_apriltag_detect_callback(self, msg):
        if msg.detections:
            from_frame = f'{msg.detections[0].family}:{msg.detections[0].id}'
            to_frame = 'bluerov'
            try:
                t = self.tf_buffer.lookup_transform(
                    to_frame,
                    from_frame,
                    rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame} to {from_frame}: {ex}')
                return
            
            self.apriltag_pose = [t.transform.translation.x, t.transform.translation.y]


    def maddy_yaw_rate_callback(self, msg):
        self.maddy_yaw_rate = msg.twist.angular.z

    def maddy_pose_callback(self, msg):
        self.maddy_pose_estimate[0] = msg.pose.pose.position.x
        self.maddy_pose_estimate[1] = msg.pose.pose.position.y
        self.maddy_pose_estimate[2] = msg.pose.pose.position.z

        rot = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.maddy_yaw_estimate = Rotation.from_quat(rot).as_euler('xyz', degrees=True)[2]

    def log_data(self):
        with open(self.LOG_FILE, 'a') as csv_file:
            csv_writer =csv.DictWriter(csv_file, fieldnames=self.fields)
            info = {
                'timestep': str(self.get_clock().now().seconds_nanoseconds()[0]) + '.' + str(self.get_clock().now().seconds_nanoseconds()[1]),
                'uuv_x': self.bluerov_pose_estimate[0],
                'uuv_y': self.bluerov_pose_estimate[1],
                'uuv_z': self.bluerov_pose_estimate[2],
                'uuv_psi': self.bluerov_yaw_estimate,
                'usv_x': self.maddy_pose_estimate[0],
                'usv_y': self.maddy_pose_estimate[1],
                'usv_psi': self.maddy_yaw_estimate,
                'gps_x': 0,
                'gps_y': 0,
                'uuv_compass': self.bluerov_yaw_estimate,
                'usv_compass': self.maddy_yaw_estimate,
                'uuv_gyro': self.bluerov_gyro_rates[2],
                'usv_gyro': self.maddy_yaw_rate,
                'uuv_acc_x': self.bluerov_accel[0],
                'uuv_acc_y': self.bluerov_accel[1],
                'uuv_acc_z': self.bluerov_accel[2],
                'apriltag_x': self.apriltag_pose[0],
                'apriltag_y': self.apriltag_pose[1]
            }
            
            csv_writer.writerow(info)
            self.apriltag_pose = [0.0, 0.0]

        json_info = {
            'time': 10,
            'apriltags': [{'id1': 1}, {'id2': 2}]
        }

        with open(os.path.join('data_log', 'sensor_data', f'{self.get_clock().now().seconds_nanoseconds()[0]}_test_output.json'), 'a') as json_file:
            pass


def main(args=None):
    rclpy.init(args=args)

    data_logger = Data_Logger()

    rclpy.spin(data_logger)
    data_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
