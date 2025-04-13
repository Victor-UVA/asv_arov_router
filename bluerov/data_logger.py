import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
import csv

from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped

class Data_Logger(Node):
    def __init__(self):
        super().__init__('data_logger')

        self.bluerov_accel = [0.0,0.0,0.0]
        self.bluerov_yaw_rate = 0.0
        self.bluerov_pose_estimate = [0.0,0.0,0.0]
        self.bluerov_yaw_estimate = 0.0

        self.maddy_yaw_rate = 0.0
        self.maddy_pose_estimate = [0.0,0.0,0.0]
        self.maddy_yaw_estimate = 0.0
        
        self.LOG_FILE = f'/home/malori/Desktop/{self.get_clock().now().seconds_nanoseconds()[0]}_data.csv' # TODO Change file path to be appropriate for USV computer
        data_logging_period = 0.1

        self.fields = ['timestep','uuv_x','uuv_y','uuv_z','uuv_psi','usv_x','usv_y','usv_psi','gps_x','gps_y','uuv_compass','usv_compass','uuv_gyro','usv_gyro', 'uuv_acc_x', 'uuv_acc_y', 'uuv_acc_z']
        with open(self.LOG_FILE, 'w') as csv_file:
            csv_writer =csv.DictWriter(csv_file, fieldnames=self.fields)
            csv_writer.writeheader()

        self.bluerov_accel_sub = self.create_subscription(
            AccelStamped,
            'bluerov/accel',
            self.bluerov_accel_callback,
            10)
        self.bluerov_accel_sub  # prevent unused variable warning

        self.bluerov_yaw_rate_sub = self.create_subscription(
            TwistStamped,
            'bluerov/yaw_rate',
            self.bluerov_yaw_rate_callback,
            10)
        self.bluerov_yaw_rate_sub  # prevent unused variable warning

        self.bluerov_pose_sub = self.create_subscription(
            PoseStamped,
            'bluerov/pose_from_dvl',
            self.bluerov_pose_callback,
            10)
        self.bluerov_pose_sub  # prevent unused variable warning

        self.maddy_yaw_rate_sub = self.create_subscription(
            TwistStamped,
            'maddy/yaw_rate',
            self.maddy_yaw_rate_callback,
            10)
        self.maddy_yaw_rate_sub  # prevent unused variable warning

        self.maddy_pose_sub = self.create_subscription(
            PoseStamped,
            'maddy/pose',
            self.maddy_pose_callback,
            10)
        self.maddy_pose_sub  # prevent unused variable warning

        self.create_timer(data_logging_period, self.log_data)

    def bluerov_accel_callback(self, msg):
        self.bluerov_accel[0] = msg.accel.linear.x
        self.bluerov_accel[1] = msg.accel.linear.y
        self.bluerov_accel[2] = msg.accel.linear.z

    def bluerov_yaw_rate_callback(self, msg):
        self.bluerov_yaw_rate = msg.twist.angular.z

    def bluerov_pose_callback(self, msg):
        self.bluerov_pose_estimate[0] = msg.pose.position.x
        self.bluerov_pose_estimate[1] = msg.pose.position.y
        self.bluerov_pose_estimate[2] = msg.pose.position.z

        rot = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.bluerov_yaw_estimate = Rotation.from_quat(rot).as_euler('xyz', degrees=True)[2]

    def maddy_yaw_rate_callback(self, msg):
        self.maddy_yaw_rate = msg.twist.angular.z

    def maddy_pose_callback(self, msg):
        self.maddy_pose_estimate[0] = msg.pose.position.x
        self.maddy_pose_estimate[1] = msg.pose.position.y
        self.maddy_pose_estimate[2] = msg.pose.position.z

        rot = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
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
                'uuv_gyro': self.bluerov_yaw_rate,
                'usv_gyro': self.maddy_yaw_rate,
                'uuv_acc_x': self.bluerov_accel[0],
                'uuv_acc_y': self.bluerov_accel[1],
                'uuv_acc_z': self.bluerov_accel[2]
            }
            
            csv_writer.writerow(info)


def main(args=None):
    rclpy.init(args=args)

    data_logger = Data_Logger()

    rclpy.spin(data_logger)
    data_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
