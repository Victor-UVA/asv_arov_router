import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
import csv
import json
import os

from geometry_msgs.msg import AccelStamped, TwistStamped, Twist
from nav_msgs.msg import Odometry
from apriltag_msgs.msg import AprilTagDetectionArray

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class Data_Logger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.declare_parameters(namespace='', parameters=[
            ('arov_name', 'arov'),
            ('asv_name', 'asv')
        ])

        self.asv = self.get_parameter('asv_name').value
        self.arov = self.get_parameter('arov_name').value

        self.arov_accel = [0.0,0.0,0.0]
        self.arov_gyro_rates = [0.0,0.0,0.0]
        self.arov_pose_estimate = [0.0,0.0,0.0]
        self.arov_yaw_estimate = 0.0
        self.arov_control = Twist()

        self.asv_yaw_rate = 0.0
        self.asv_pose_estimate = [0.0,0.0,0.0]
        self.asv_yaw_estimate = 0.0

        self.apriltags = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.JSON_LOG_FILE = os.path.join('data_log', 'sensor_data', f'{self.get_clock().now().seconds_nanoseconds()[0]}_data.json')
        data_logging_period = 0.02

        # AROV subscriptions
        self.arov_accel_sub = self.create_subscription(
            AccelStamped,
            f'{self.arov}/accel',
            self.arov_accel_callback,
            10)
        self.arov_accel_sub  # prevent unused variable warning

        self.arov_gyro_rates_sub = self.create_subscription(
            TwistStamped,
            f'{self.arov}/gyro_rates',
            self.arov_gyro_rates_callback,
            10)
        self.arov_gyro_rates_sub  # prevent unused variable warning

        self.arov_pose_sub = self.create_subscription(
            Odometry,
            f'{self.arov}/odom',
            self.arov_pose_callback,
            10)
        self.arov_pose_sub  # prevent unused variable warning

        self.arov_apriltag_detect_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.arov_apriltag_detect_callback,
            10)
        self.arov_apriltag_detect_sub  # prevent unused variable warning

        self.arov_cmd_vel_sub = self.create_subscription(
            Twist,
            f'{self.arov}/cmd_vel',
            self.arov_cmd_vel_callback,
            10)
        self.arov_apriltag_detect_sub  # prevent unused variable warning

        # ASV subscriptions
        self.asv_yaw_rate_sub = self.create_subscription(
            TwistStamped,
            f'{self.arov}/gyro_rates',
            self.asv_yaw_rate_callback,
            10)
        self.asv_yaw_rate_sub  # prevent unused variable warning

        self.asv_pose_sub = self.create_subscription(
            Odometry,
            f'{self.arov}/pose',
            self.asv_pose_callback,
            10)
        self.asv_pose_sub  # prevent unused variable warning

        self.create_timer(data_logging_period, self.log_data)

    def arov_accel_callback(self, msg):
        self.arov_accel[0] = msg.accel.linear.x
        self.arov_accel[1] = msg.accel.linear.y
        self.arov_accel[2] = msg.accel.linear.z

    def arov_gyro_rates_callback(self, msg):
        self.arov_gyro_rates[0] = msg.twist.angular.x
        self.arov_gyro_rates[1] = msg.twist.angular.y
        self.arov_gyro_rates[2] = msg.twist.angular.z

    def arov_pose_callback(self, msg: Odometry):
        self.arov_pose_estimate[0] = msg.pose.pose.position.x
        self.arov_pose_estimate[1] = msg.pose.pose.position.y
        self.arov_pose_estimate[2] = msg.pose.pose.position.z

        rot = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.arov_yaw_estimate = Rotation.from_quat(rot).as_euler('xyz', degrees=False)[2]

    def arov_apriltag_detect_callback(self, msg: AprilTagDetectionArray):
        if msg.detections:
            from_frame = f'{msg.detections[0].family}:{msg.detections[0].id}'
            to_frame = 'arov'
            for tag in msg.detections:
                from_frame = f'{tag.family}:{tag.id}'
                to_frame = 'arov'
                try:
                    t = self.tf_buffer.lookup_transform(
                        to_frame,
                        from_frame,
                        rclpy.time.Time())

                    tag_already_detected = False
                    for tags in self.apriltags:
                        if tags['id'] == from_frame:
                            tags['x'] = t.transform.translation.x
                            tags['y'] = t.transform.translation.y
                            tag_already_detected = True

                    if not tag_already_detected:
                        self.apriltags.append({'id': from_frame, 'x': t.transform.translation.x, 'y': t.transform.translation.y})

                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {to_frame} to {from_frame}: {ex}')
                    return

    def arov_cmd_vel_callback(self, msg):
        self.arov_control = msg

    def asv_yaw_rate_callback(self, msg):
        self.asv_yaw_rate = msg.twist.angular.z

    def asv_pose_callback(self, msg):
        self.asv_pose_estimate[0] = msg.pose.pose.position.x
        self.asv_pose_estimate[1] = msg.pose.pose.position.y
        self.asv_pose_estimate[2] = msg.pose.pose.position.z

        rot = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.asv_yaw_estimate = Rotation.from_quat(rot).as_euler('xyz', degrees=True)[2]

    def log_data(self):
        json_info = {
            'timestep': str(self.get_clock().now().seconds_nanoseconds()[0]) + '.' + str(self.get_clock().now().seconds_nanoseconds()[1]),
            'uuv_x': self.arov_pose_estimate[0],
            'uuv_y': self.arov_pose_estimate[1],
            'uuv_z': self.arov_pose_estimate[2],
            'uuv_psi': self.arov_yaw_estimate,
            'usv_x': self.asv_pose_estimate[0],
            'usv_y': self.asv_pose_estimate[1],
            'usv_psi': self.asv_yaw_estimate,
            'uuv_compass': self.arov_yaw_estimate,
            'usv_compass': self.asv_yaw_estimate,
            'uuv_gyro': self.arov_gyro_rates[2],
            'usv_gyro': self.asv_yaw_rate,
            'uuv_acc_x': self.arov_accel[0],
            'uuv_acc_y': self.arov_accel[1],
            'uuv_acc_z': self.arov_accel[2],
            'uuv_u_x': self.arov_control.linear.x,
            'uuv_u_y': self.arov_control.linear.y,
            'uuv_u_z': self.arov_control.linear.z,
            'uuv_u_yaw': self.arov_control.angular.z,
            'apriltags': self.apriltags
        }

        self.apriltags = []

        with open(self.JSON_LOG_FILE, 'a') as json_file:
            json.dump(json_info, json_file)
            json_file.write('\n')


def main(args=None):
    rclpy.init(args=args)

    data_logger = Data_Logger()

    rclpy.spin(data_logger)
    data_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
