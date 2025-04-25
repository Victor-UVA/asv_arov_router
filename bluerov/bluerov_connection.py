from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import AccelStamped, TwistStamped, TransformStamped, Twist
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


        # Define control constants and variables
        TRANSLATION_LIMIT = 100
        ROTATION_LIMIT = 80
        self.MAX_VEL = 0.2
        self.MAX_OMEGA = 0.15
        self.VEL_TO_CMD = TRANSLATION_LIMIT / self.MAX_VEL
        self.OMEGA_TO_CMD = ROTATION_LIMIT / self.MAX_OMEGA

        self.cmd_vel_enabled = True


        # Define subscriptions to other ROS topics
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback)


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





    # Commands
    def arm_disarm(self, set_value = None):
        '''
        Change the armed state of the vehicle.

        Args:
            set_value (bool, optional): 
            If set_value is not provided, then if the vehicle is armed, disarm it, if it is disarmed, arm it.
        '''
        if set_value == None:
            if not self.armed:
                self.master.arducopter_arm()
            else:
                self.master.arducopter_disarm()
        elif set_value:
            self.master.arducopter_arm()
        else:
            self.master.arducopter_disarm()

    def set_mode(self, mode):
        """
        Set ROV mode
        http://ardupilot.org/copter/docs/flight-modes.html

        Args:
            mode (str): MAVLink mode
        """
        mode = mode.upper()

        mode_id = self.master.mode_mapping()[mode]
        self.master.set_mode(mode_id)

    def set_position_target_local_ned(self, param=[]):
        '''
        Implement MavLink command: SET_POSITION_TARGET_LOCAL_NED
        
        Args:
            param (list, optional): param1, param2, ..., param11\\
                1, 2, 3 are position\\
                4, 5, 6 are velocity\\
                7, 8, 9 are acceleration\\
                10, 11 are yaw and yaw rate
        '''

        if len(param) != 11:
            print('SET_POSITION_TARGET_LOCAL_NED need 11 params')

        # Set mask
        mask = 0b0000000111111111
        for i, value in enumerate(param):
            if value is not None:
                mask -= 1<<i
            else:
                param[i] = 0.0
        
        self.master.mav.set_position_target_local_ned_send(
            0,                                              # system time in milliseconds
            self.master.target_system,                      # target system
            self.master.target_component,                   # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,            # frame
            mask,                                           # mask
            param[0], param[1], param[2],                   # position x,y,z
            param[3], param[4], param[5],                   # velocity x,y,z
            param[6], param[7], param[8],                   # accel x,y,z
            param[9], param[10])                            # yaw, yaw rate
        
    def set_attitude_target(self, param=[]):
        """
        Implement MavLink command: SET_ATTITUDE_TARGET

        Args:
            param (list, optional): param1, param2, ..., param8\\
            1, 2, 3, 4 are a quaternion represention the desired rotation\\
            5, 6, 7 are roll, pitch, and yaw rates respectively\\
            8 is thrust
        """
        if len(param) != 8:
            print('SET_ATTITUDE_TARGET need 8 params')

        # Set mask
        mask = 0b11111111
        for i, value in enumerate(param[4:-1]):
            if value is not None:
                mask -= 1<<i
            else:
                param[i+3] = 0.0

        if param[7] is not None:
            mask += 1<<6
        else:
            param[7] = 0.0

        q = param[:4]

        if q != [None, None, None, None]:
            mask += 1<<7
        else:
            q = [1.0, 0.0, 0.0, 0.0]

        self.master.mav.set_attitude_target_send(0, # system time in milliseconds
            self.master.target_system,              # target system
            self.master.target_component,           # target component
            mask,                                   # mask
            q,                                      # quaternion attitude
            param[4],                               # body roll rate
            param[5],                               # body pitch rate
            param[6],                               # body yaw rate
            param[7])                               # thrust

    def set_cmd_pwm(self, pwm_linear_x=1500, pwm_linear_y=1500, pwm_linear_z=1500, pwm_angular_z=1500):
        '''
        Send commands mimicing joystick inputs.

        Args:
            pwm_linear_x (int, optional): Forward / backward
            pwm_linear_y (int, optional): Rigth / left
            pwm_linear_z (int, optional): Up / down
            pwm_angular_yaw (int, optional): Clockwise / counter-clockwise
        '''
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[2] = pwm_linear_z
        rc_channel_values[3] = pwm_angular_z
        rc_channel_values[4] = pwm_linear_x
        rc_channel_values[5] = pwm_linear_y
        self.master.mav.rc_channels_override_send(
            self.master.target_system,              # target_system
            self.master.target_component,           # target_component
            *rc_channel_values)                     # RC channel list, in microseconds.






    # Subscribers and Support
    def cmd_vel_to_pwm(self, msg):
        '''
        Convert a ROS cmd_vel message into PWM values

        Args:
            msg (Twist): Velocity command to convert to PWM
        '''
        vel_x, vel_y = msg.linear.x, msg.linear.y
        omega_z = msg.angular.z

        vel_x = max(-self.MAX_VEL, min(self.MAX_VEL, vel_x))
        vel_y = max(-self.MAX_VEL, min(self.MAX_VEL, vel_y))
        omega_z = max(-self.MAX_OMEGA, min(self.MAX_OMEGA, omega_z))

        x = 1500 + int(self.VEL_TO_CMD * vel_x)
        y = 1500 + int(self.VEL_TO_CMD * vel_y)
        z = 65535
        yaw = 1500 + int(self.OMEGA_TO_CMD * omega_z)

        return x, y, z, yaw
    
    def cmd_vel_callback(self, msg):
        '''
        Callback for ROS cmd_vel messages to convert them to PWM values and send those to the vehicle.

        Args:
            msg (Twist): Velocity command to convert to PWM
        '''
        if self.cmd_vel_enabled:
            self.set_cmd_pwm(self.cmd_vel_to_pwm(msg))
        else:
            return




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
        
        for msg_raw in msgs:
            # self.get_logger().info(f'{msg.to_dict()}')
            msg = msg_raw.to_dict()
            msg_type = msg_raw.get_type()

            if msg == None or msg_type == 'BAD_DATA':
                pass

            elif msg_type == 'HEARTBEAT':
                base_mode = msg['base_mode']
                is_autopilot = True if msg['autopilot'] != 8 else False
                if is_autopilot:
                    self.armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            elif msg_type == 'SCALED_IMU2':
                self.accel[0] = 9.81 * float(msg['xacc']) / 1000.0
                self.accel[1] = -9.81 * float(msg['yacc']) / 1000.0
                self.accel[2] = -9.81 * float(msg['zacc']) / 1000.0

                # TODO Verify orientation of gyro axes (originally had minus on z)
                self.gyro_rates[0] = float(msg['xgyro']) / 1000.0
                self.gyro_rates[1] = -float(msg['ygyro']) / 1000.0
                self.gyro_rates[2] = -float(msg['zgyro']) / 1000.0

            elif msg_type == 'LOCAL_POSITION_NED':
                self.pose_estimate[0] = msg['x']
                self.pose_estimate[1] = msg['y']
                self.pose_estimate[2] = msg['z']

                self.vel_estimate[0] = msg['vx']
                self.vel_estimate[1] = msg['vy']
                self.vel_estimate[2] = msg['vz']

            elif msg_type == 'ATTITUDE':
                self.orientation_estimate[0] = msg['roll']
                self.orientation_estimate[1] = msg['pitch']
                self.orientation_estimate[2] = msg['yaw']

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