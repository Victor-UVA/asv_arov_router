from pymavlink import mavutil
import rclpy
from rclpy.node import Node
import time
from scipy.spatial.transform import Rotation
import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import TransformBroadcaster

class Maddy_Listener(Node):
    '''
    Node to get data from Maddy over serial connection via MavLink and publish the data to ROS topics
    
    Data collected:
    - Yaw rate: psi dot (rad/s)
    - Yaw estimate: psi (rad)
    - Local position estimate from GPS: x, y, z (m)
    '''
    def __init__(self):
        super().__init__('maddy_listener')
        self.USING_HARDWARE = True # Toggle for testing when connected to hardware or not
    
        self.get_logger().info('Hi from Maddy node!')
        
        if self.USING_HARDWARE:
            self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14552')
            self.master.wait_heartbeat()

            self.get_logger().info("Heartbeat from system (system %u component %u)" % (self.master.target_system, self.master.target_component))
            self.get_logger().info('Maddy connected!')

        # Define variables to store the data from the BlueROV between publishing it
        self.yaw_rate = 0.0
        self.pose_estimate = [0.0,0.0,0.0]
        self.yaw_estimate = 0.0
        self.imu = Imu()
        self.ned_to_enu = Rotation.from_euler('xyz', [180, 0, 90], degrees=True).as_matrix()
        self.gps = NavSatFix()

        # Define publishers for data from Maddy to ROS
        self.yaw_rate_publisher_ = self.create_publisher(TwistStamped, 'maddy/yaw_rate', 10)
        self.imu_publisher_ = self.create_publisher(Imu, 'maddy/imu', 10)
        self.gps_publisher_ = self.create_publisher(NavSatFix, 'maddy/gps', 10)
        self.pose_publisher_ = self.create_publisher(Odometry, 'maddy/pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)


        # Create timers to aquire and publish data
        publish_timer_period = 0.1 # seconds
        data_timer_period = 0.01

        self.yaw_rate_timer = self.create_timer(publish_timer_period, self.yaw_rate_publish)
        self.imu_timer = self.create_timer(data_timer_period, self.imu_publish)
        self.gps_timer = self.create_timer(data_timer_period, self.gps_publish)
        self.pose_timer = self.create_timer(publish_timer_period, self.pose_publish)
        self.data_timer = self.create_timer(data_timer_period, self.get_maddy_data)

    def yaw_rate_publish(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = '/maddy'

        msg.twist.angular.z = self.yaw_rate

        self.yaw_rate_publisher_.publish(msg)

    def imu_publish(self):
        self.imu_publisher_.publish(self.imu)
        
    def gps_publish(self):
        self.gps_publisher_.publish(self.gps)

    def pose_publish(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = '/world'
        msg.child_frame_id = '/maddy_odom'

        msg.pose.pose.position.x = self.pose_estimate[0]
        msg.pose.pose.position.y = self.pose_estimate[1]
        msg.pose.pose.position.z = self.pose_estimate[2]

        yaw = Rotation.from_euler('xyz', [0, 0, self.yaw_estimate], degrees=True).as_quat()

        # self.get_logger().info(f"{yaw}, {np.sum(np.power(yaw,2))}")
        
        msg.pose.pose.orientation.w = yaw[3]
        msg.pose.pose.orientation.x = yaw[0]
        msg.pose.pose.orientation.y = yaw[1]
        msg.pose.pose.orientation.z = yaw[2]

        self.pose_publisher_.publish(msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = '/world'
        t.child_frame_id = '/maddy_odom'

        t.transform.translation.x = self.pose_estimate[0]
        t.transform.translation.y = self.pose_estimate[1]
        t.transform.translation.z = self.pose_estimate[2]

        t.transform.rotation.x = yaw[0]
        t.transform.rotation.y = yaw[1]
        t.transform.rotation.z = yaw[2]
        t.transform.rotation.w = yaw[3]

        self.tf_broadcaster.sendTransform(t)

    def get_maddy_data(self):
        if self.USING_HARDWARE:
            try:
                msg = self.master.recv_match(blocking=True)
                if msg == None or msg.get_type() == 'BAD_DATA':
                    pass

                elif msg.get_type() == 'SCALED_IMU2':
                    self.yaw_rate = float(msg.to_dict()['zgyro']) / 1000.0

                    self.imu.header.stamp = self.get_clock().now().to_msg()
                    self.imu.header.frame_id = '/maddy'

                    self.imu_orientation = Rotation.from_euler('xyz', [0, 0, self.yaw_estimate -90], degrees=True).as_quat()

                    self.imu.orientation.x = self.imu_orientation[0]
                    self.imu.orientation.y = self.imu_orientation[1]
                    self.imu.orientation.z = self.imu_orientation[2]
                    self.imu.orientation.w = self.imu_orientation[3]

                    angular_vel_ned = np.array([msg.to_dict()['xgyro'], msg.to_dict()['ygyro'], msg.to_dict()['zgyro']], dtype=float) / 1000
                    angular_vel_enu = self.ned_to_enu @ angular_vel_ned

                    self.imu.angular_velocity.x = angular_vel_enu[0]
                    self.imu.angular_velocity.y = angular_vel_enu[1]
                    self.imu.angular_velocity.z = angular_vel_enu[2]

                    # self.get_logger().info(f"{angular_vel_enu}")

                    linear_accel_ned = -9.81 * np.array([msg.to_dict()['xacc'], msg.to_dict()['yacc'], msg.to_dict()['zacc']], dtype=float) / 1000
                    linear_accel_enu = self.ned_to_enu @ linear_accel_ned

                    self.imu.linear_acceleration.x = linear_accel_enu[0]
                    self.imu.linear_acceleration.y = linear_accel_enu[1]
                    self.imu.linear_acceleration.z = linear_accel_enu[2]

                    # self.get_logger().info(f"{linear_accel_enu}")

                elif msg.get_type() == 'GPS_RAW_INT':
                    self.gps.header.stamp = self.get_clock().now().to_msg()
                    self.gps.header.frame_id = '/maddy'


                    if msg.to_dict()['fix_type'] == 1:
                        status = -1
                    elif msg.to_dict()['fix_type'] == 2 or msg.to_dict()['fix_type'] == 3:
                        status = 0
                    elif msg.to_dict()['fix_type'] == 4:
                        status = 1
                    elif msg.to_dict()['fix_type'] == 5 or msg.to_dict()['fix_type'] == 6:
                        status = 2
                    else:
                        status = -1

                    self.gps.status.status = status
                    self.gps.status.service = 1

                    self.gps.latitude = msg.to_dict()['lat'] / (10**7)
                    self.gps.longitude = msg.to_dict()['lon'] / (10**7)
                    self.gps.altitude = msg.to_dict()['alt'] / 1000.0

                elif msg.get_type() == 'LOCAL_POSITION_NED':
                    self.pose_estimate[0] = msg.to_dict()['x']
                    self.pose_estimate[1] = msg.to_dict()['y']
                    self.pose_estimate[2] = msg.to_dict()['z']

                    # self.get_logger().info(f"{msg.to_dict()['x']}, {msg.to_dict()['y']}, {msg.to_dict()['z']}, {self.yaw_estimate}")

                elif msg.get_type() == 'ATTITUDE':
                    self.yaw_estimate = msg.to_dict()['yaw']

                else:
                    pass

            except:
                pass
        else:
            pass


def main():
    rclpy.init()

    maddy_listener = Maddy_Listener()

    rclpy.spin(maddy_listener)

    maddy_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()