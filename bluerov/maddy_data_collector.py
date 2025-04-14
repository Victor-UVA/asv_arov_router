from pymavlink import mavutil
import rclpy
from rclpy.node import Node
import time
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
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

        # Define publishers for data from Maddy to ROS
        self.yaw_rate_publisher_ = self.create_publisher(TwistStamped, 'maddy/yaw_rate', 10)
        self.pose_publisher_ = self.create_publisher(Odometry, 'maddy/pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)


        # Create timers to aquire and publish data
        publish_timer_period = 0.1 # seconds
        data_timer_period = 0.01

        self.yaw_rate_timer = self.create_timer(publish_timer_period, self.yaw_rate_publish)
        self.pose_timer = self.create_timer(publish_timer_period, self.pose_publish)
        self.data_timer = self.create_timer(data_timer_period, self.get_maddy_data)

    def yaw_rate_publish(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = '/maddy'

        msg.twist.angular.z = self.yaw_rate

        self.yaw_rate_publisher_.publish(msg)

    def pose_publish(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = '/world'
        msg.child_frame_id = '/maddy_odom'

        msg.pose.pose.position.x = self.pose_estimate[0]
        msg.pose.pose.position.y = self.pose_estimate[1]
        msg.pose.pose.position.z = self.pose_estimate[2]

        yaw = Rotation.from_euler('xyz', [0, 0, self.yaw_estimate], degrees=True).as_quat()
        
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

                elif msg.get_type() == 'LOCAL_POSITION_NED':
                    self.pose_estimate[0] = msg.to_dict()['x']
                    self.pose_estimate[1] = msg.to_dict()['y']
                    self.pose_estimate[2] = msg.to_dict()['z']

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