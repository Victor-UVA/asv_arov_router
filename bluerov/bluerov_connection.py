from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import AccelStamped, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class BlueROV_Connection(Node):
    '''
    Node to connect to the BlueROV2 over mavlink to translate data from it into ROS topics and allow for autonomous control.
    '''
    def __init__(self):
        super().__init__('bluerov_connection')
        self.get_logger().info('Hi from BlueROV node!')
        
        # self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
        self.master = mavutil.mavlink_connection('udpin:localhost:14551')
        self.master.wait_heartbeat()

        self.get_logger().info("Heartbeat from system (system %u component %u)" % (self.master.target_system, self.master.target_component))
        self.get_logger().info('BlueROV connected!')

        # Define variables to store the data from the BlueROV between publishing it
        self.accel = [0.0,0.0,0.0]
        self.gyro_rates = [0.0, 0.0, 0.0]
        self.pose_estimate = [0.0,0.0,0.0]
        self.orientation_estimate = [0.0,0.0,0.0]
        self.vel_estimate = [0.0,0.0,0.0]

        # Define publishers for data from BlueROV to ROS
        self.acc_publisher_ = self.create_publisher(AccelStamped, 'bluerov/accel', 10)
        self.gyro_rates_publisher_ = self.create_publisher(TwistStamped, 'bluerov/gyro_rates', 10)
        self.pose_publisher_ = self.create_publisher(Odometry, 'bluerov/pose_from_dvl', 10)
        self.tf_broadcaster = TransformBroadcaster(self)


        # Create timers to aquire and publish data
        publish_timer_period = 0.02 # seconds
        data_timer_period = 0.005

        self.accel_timer = self.create_timer(publish_timer_period, self.accel_publish)
        self.gyro_rates_timer = self.create_timer(publish_timer_period, self.gyro_rates_publish)
        self.pose_timer = self.create_timer(publish_timer_period, self.pose_publish)
        self.data_timer = self.create_timer(data_timer_period, self.parse_bluerov_data)












    # Publishers
    def accel_publish(self):
        msg = AccelStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = '/bluerov'

        msg.accel.linear.x = self.accel[0]
        msg.accel.linear.y = self.accel[1]
        msg.accel.linear.z = self.accel[2]

        self.acc_publisher_.publish(msg)

    def gyro_rates_publish(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = '/bluerov'

        msg.twist.angular.x = self.gyro_rates[0]
        msg.twist.angular.y = self.gyro_rates[1]
        msg.twist.angular.z = self.gyro_rates[2]

        self.gyro_rates_publisher_.publish(msg)

    def pose_publish(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = '/world'
        msg.child_frame_id = '/bluerov_odom'

        msg.pose.pose.position.x = self.pose_estimate[0]
        msg.pose.pose.position.y = self.pose_estimate[1]
        msg.pose.pose.position.z = self.pose_estimate[2]

        orientation = Rotation.from_euler('xyz', self.orientation_estimate, degrees=True).as_quat()
        
        msg.pose.pose.orientation.w = orientation[3]
        msg.pose.pose.orientation.x = orientation[0]
        msg.pose.pose.orientation.y = orientation[1]
        msg.pose.pose.orientation.z = orientation[2]

        self.pose_publisher_.publish(msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = '/world'
        t.child_frame_id = '/bluerov_odom'

        t.transform.translation.x = self.pose_estimate[0]
        t.transform.translation.y = self.pose_estimate[1]
        t.transform.translation.z = self.pose_estimate[2]

        t.transform.rotation.x = orientation[0]
        t.transform.rotation.y = orientation[1]
        t.transform.rotation.z = orientation[2]
        t.transform.rotation.w = orientation[3]

        self.tf_broadcaster.sendTransform(t)










    # Data collection from BlueROV
    def get_bluerov_data(self):
        msgs = []

        while True:
            try:
                msg = self.master.recv_match()
                if msg != None:
                    msgs.append(msg)
                else:
                    break
            except:
                break

        return msgs
    
    def parse_bluerov_data(self):
        
        msgs = self.get_bluerov_data()

        for msg in msgs:
            if msg == None or msg.get_type() == 'BAD_DATA':
                pass

            elif msg.get_type() == 'SCALED_IMU2':
                self.accel[0] = 9.81 * float(msg.to_dict()['xacc']) / 1000.0
                self.accel[1] = -9.81 * float(msg.to_dict()['yacc']) / 1000.0
                self.accel[2] = -9.81 * float(msg.to_dict()['zacc']) / 1000.0

                # TODO Verify orientation of gyro axes (originally had minus on z)
                self.gyro_rates[0] = float(msg.to_dict()['xgyro']) / 1000.0
                self.gyro_rates[1] = -float(msg.to_dict()['ygyro']) / 1000.0
                self.gyro_rates[2] = -float(msg.to_dict()['zgyro']) / 1000.0

            elif msg.get_type() == 'LOCAL_POSITION_NED':
                self.pose_estimate[0] = msg.to_dict()['x']
                self.pose_estimate[1] = msg.to_dict()['y']
                self.pose_estimate[2] = msg.to_dict()['z']

                self.vel_estimate[0] = msg.to_dict()['vx']
                self.vel_estimate[1] = msg.to_dict()['vy']
                self.vel_estimate[2] = msg.to_dict()['vz']

            elif msg.get_type() == 'ATTITUDE':
                self.orientation_estimate[0] = msg.to_dict()['roll']
                self.orientation_estimate[1] = msg.to_dict()['pitch']
                self.orientation_estimate[2] = msg.to_dict()['yaw']

            else:
                pass


def main():
    rclpy.init()

    bluerov_connection = BlueROV_Connection()

    rclpy.spin(bluerov_connection)

    bluerov_connection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
pymavlink control:
Possible commands:
- SET_ATTITUDE_TARGET
- SET_POSITION_TARGET_LOCAL_NED



'''