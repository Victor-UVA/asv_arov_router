from pymavlink import mavutil
import rclpy
from rclpy.node import Node
import time
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped


class BlueROV_Listener(Node):
    '''
    Node to get data from BlueROV2 over udp connection via MavLink and publish the data to ROS topics

    Data collected:
    - Acceleration: x, y, z (m/s^2)
    - Yaw rate: psi dot (rad/s)
    - Yaw estimate: psi (rad)
    - Local position estimate from DVL: x, y, z (m)
    '''
    def __init__(self):
        super().__init__('bluerov_listener')
        self.USING_HARDWARE = True # Toggle for testing when connected to hardware or not
    
        self.get_logger().info('Hi from bluerov node!')
        
        if self.USING_HARDWARE:
            self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
            self.master.wait_heartbeat()

            self.get_logger().info("Heartbeat from system (system %u component %u)" % (self.master.target_system, self.master.target_component))
            self.get_logger().info('BlueROV connected!')

        # Define variables to store the data from the BlueROV between publishing it
        self.accel = [0.0,0.0,0.0]
        self.yaw_rate = 0.0
        self.pose_estimate = [0.0,0.0,0.0]
        self.yaw_estimate = 0.0

        # Define publishers for data from BlueROV to ROS
        self.acc_publisher_ = self.create_publisher(AccelStamped, 'bluerov/accel', 10)
        self.yaw_rate_publisher_ = self.create_publisher(TwistStamped, 'bluerov/yaw_rate', 10)
        self.pose_publisher_ = self.create_publisher(PoseStamped, 'bluerov/pose_from_dvl', 10)


        # Create timers to aquire and publish data
        publish_timer_period = 0.1 # seconds
        data_timer_period = 0.01

        self.accel_timer = self.create_timer(publish_timer_period, self.accel_publish)
        self.yaw_rate_timer = self.create_timer(publish_timer_period, self.yaw_rate_publish)
        self.pose_timer = self.create_timer(publish_timer_period, self.pose_publish)
        self.data_timer = self.create_timer(data_timer_period, self.get_bluerov_data)


    def accel_publish(self):
        msg = AccelStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'BlueROV'

        msg.accel.linear.x = self.accel[0]
        msg.accel.linear.y = self.accel[1]
        msg.accel.linear.z = self.accel[2]

        self.acc_publisher_.publish(msg)

    def yaw_rate_publish(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'BlueROV'

        msg.twist.angular.z = self.yaw_rate

        self.yaw_rate_publisher_.publish(msg)

    def pose_publish(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'BlueROV'

        msg.pose.position.x = self.pose_estimate[0]
        msg.pose.position.y = self.pose_estimate[1]
        msg.pose.position.z = self.pose_estimate[2]

        yaw = Rotation.from_euler('xyz', [0, 0, self.yaw_estimate], degrees=True).as_quat()
        
        msg.pose.orientation.w = yaw[3]
        msg.pose.orientation.x = yaw[0]
        msg.pose.orientation.y = yaw[1]
        msg.pose.orientation.z = yaw[2]

        self.pose_publisher_.publish(msg)

    def get_bluerov_data(self):
        if self.USING_HARDWARE:
            try:
                msg = self.master.recv_match(blocking=True)
                if msg == None or msg.get_type() == 'BAD_DATA':
                    pass

                elif msg.get_type() == 'SCALED_IMU2':
                    self.accel[0] = 9.81 * float(msg.to_dict()['xacc']) / 1000.0
                    self.accel[1] = -9.81 * float(msg.to_dict()['yacc']) / 1000.0
                    self.accel[2] = -9.81 * float(msg.to_dict()['zacc']) / 1000.0

                    self.yaw_rate = -float(msg.to_dict()['zgyro']) / 1000.0

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

    bluerov_listener = BlueROV_Listener()

    rclpy.spin(bluerov_listener)

    bluerov_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()