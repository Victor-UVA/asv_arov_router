from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
import numpy as np

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import TransformBroadcaster

from asv_arov_interfaces.srv import PositionTargetLocalNED, SetMAVMode, SetArmDisarm

class MAVLink_Router(Node):
    '''
    Node to connect to a MAVLink vehicle to translate data from it into ROS topics and allow for autonomous control.
    '''
    def __init__(self):
        super().__init__('mavlink_router')
        self.declare_parameters(namespace='',parameters=[
            ('device', 'udpin:localhost:14551'),
            ('rc_override_mapping', [4, 5, 2, 6, 7, 3]),                    # Which RC channels (out of 0-7) to map x, y, z, roll, pitch, yaw commands to (assumes that mixing servo outputs occurs on the vehicle)
                                                                            #   Axes are in the vehicle frame (NED)
                                                                            #   Roll and pitch are not currently used
            ('max_cmd_vel_linear', 1.0),                                    # Value for cmd_vel messages sent to this vehicle that map to the maximum PWM output
            ('max_cmd_vel_angular', 1.0),                                   # Value for cmd_vel messages sent to this vehicle that map to the maximum PWM output
            ('translation_limit', 500),                                     # Max PWM difference to send to the vehicle, typical range 1000 - 2000 with 1500 as centered
            ('rotation_limit', 500),                                        # Max PWM difference to send to the vehicle, typical range 1000 - 2000 with 1500 as centered
            ('has_camera', True),                                           # Define whether or not the vehicle has a camera to deal with
            ('camera_transform', [0.15, 0.0, 0.0, -1.571, 0.0, -1.571]),    # (m, rad) x, y, z, roll, pitch, yaw initial transform for the camera position
            ('gimbal_tilt_mapping', [1.571, -0.436, -0.5675, 0.5675])])     # (rad) Mapping from the values the gimbal system outputs at its limits (first two numbers) to the actual gimbal angles (last two)

        self.vehicle = self.get_namespace().strip('/')
        self.master = mavutil.mavlink_connection(self.get_parameter('device').value)
        self.master.wait_heartbeat()

        self.get_logger().info(f'Heartbeat from system (system {self.master.target_system} component {self.master.target_component})')
        self.get_logger().info(f'{self.vehicle} connected!')


        # Define control constants and variables
        self.MAX_VEL = self.get_parameter('max_cmd_vel_linear').value
        self.MAX_OMEGA = self.get_parameter('max_cmd_vel_angular').value
        self.VEL_TO_CMD = self.get_parameter('translation_limit').value / self.MAX_VEL
        self.OMEGA_TO_CMD = self.get_parameter('rotation_limit').value / self.MAX_OMEGA

        self.cmd_vel_enabled = True
        self.rc_override_mapping = {'x': self.get_parameter('rc_override_mapping').value[0], 'y': self.get_parameter('rc_override_mapping').value[1], 'z': self.get_parameter('rc_override_mapping').value[2],
                                    'roll': self.get_parameter('rc_override_mapping').value[3], 'pitch': self.get_parameter('rc_override_mapping').value[4], 'yaw': self.get_parameter('rc_override_mapping').value[5]}


        # Define subscriptions to other ROS topics
        self.cmd_vel_sub = self.create_subscription(Twist, f'{self.get_namespace()}/cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_vel_sub


        # Define service servers
        self.set_position_target_local_ned_srv = self.create_service(PositionTargetLocalNED, '/set_position_target_local_ned',
                                                                     self.position_target_local_ned_callback)
        self.set_mode_srv = self.create_service(SetMAVMode, '/set_mav_mode', self.set_mode_callback)
        self.set_arm_disarm_srv = self.create_service(SetArmDisarm, '/set_arm_disarm', self.set_arm_disarm_callback)


        # Define variables to store the data from the vehicle between publishing it
        self.odom = Odometry()
        self.odom.header.frame_id = f'{self.vehicle}/odom'
        self.odom.child_frame_id = f'{self.vehicle}/base_link'

        self.t = TransformStamped()
        self.t.header.frame_id = f'{self.vehicle}/odom'
        self.t.child_frame_id = f'{self.vehicle}/base_link'

        if self.get_parameter('has_camera').value:
            self.camera_gimbal_mapping = self.get_parameter('gimbal_tilt_mapping').value

            self.camera_transform = TransformStamped()
            self.camera_transform.header.frame_id = f'{self.vehicle}/base_link'
            self.camera_transform.child_frame_id = f'{self.vehicle}_camera'

            self.camera_transform.transform.translation.x = self.get_parameter('camera_transform').value[0]
            self.camera_transform.transform.translation.y = self.get_parameter('camera_transform').value[1]
            self.camera_transform.transform.translation.z = self.get_parameter('camera_transform').value[2]

            self.camera_orientation = Rotation.from_euler('xyz', [self.get_parameter('camera_transform').value[3],
                                                                  self.get_parameter('camera_transform').value[4],
                                                                  self.get_parameter('camera_transform').value[5]])

            self.camera_transform.transform.rotation.x = self.camera_orientation.as_quat()[0]
            self.camera_transform.transform.rotation.y = self.camera_orientation.as_quat()[1]
            self.camera_transform.transform.rotation.z = self.camera_orientation.as_quat()[2]
            self.camera_transform.transform.rotation.w = self.camera_orientation.as_quat()[3]

        self.imu = Imu()
        self.imu.header.frame_id = f'/{self.vehicle}/base_link'
        
        self.gps = NavSatFix()
        self.gps.header.frame_id = f'/{self.vehicle}/world'

        self.ned_to_enu = Rotation.from_euler('xyz', [180, 0, -90], degrees=True)


        # Define publishers for data from the vehicle to ROS
        self.pose_publisher_ = self.create_publisher(Odometry, f'{self.get_namespace()}/odom', 10)
        self.imu_publisher_ = self.create_publisher(Imu, f'{self.get_namespace()}/imu', 10)
        self.gps_publisher_ = self.create_publisher(NavSatFix, f'{self.get_namespace()}/gps', 10)
        self.tf_broadcaster = TransformBroadcaster(self)


        # Create timers to aquire and publish data
        publish_timer_period = 0.02 # seconds
        data_timer_period = 0.005

        self.pub_timer = self.create_timer(publish_timer_period, self.data_pub)
        self.data_timer = self.create_timer(data_timer_period, self.parse_vehicle_data)

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

    def set_mode(self, mode: str):
        """
        Set ROV mode
        http://ardupilot.org/copter/docs/flight-modes.html

        Args:
            mode (str): MAVLink mode
        """
        mode = mode.upper()

        mode_id = self.master.mode_mapping()[mode]
        self.master.set_mode(mode_id)

    def set_position_target_local_ned(self, param=[], mask='1111110111111111'):
        '''
        Implement MavLink command: SET_POSITION_TARGET_LOCAL_NED
        https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
        
        Args:
            param (list, optional): param1, param2, ..., param11\\
                1, 2, 3 are position\\
                4, 5, 6 are velocity\\
                7, 8, 9 are acceleration\\
                10, 11 are yaw and yaw rate
            mask (str, optional): Bitmask setting which values the autopilot will ignore, 1 is ignore, 0 is not.  If bit 9 is 1, it will treat accelerations as forces
        '''

        if len(param) != 11:
            self.get_logger().info('SET_POSITION_TARGET_LOCAL_NED need 11 params')

        # Set mask
        mask = int(mask, base=2)

        # self.get_logger().info(f'Sending position target: X: {param[0]}, Y: {param[1]}, Z: {param[2]}, Yaw: {param[9]}')
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
        https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET

        Args:
            param (list, optional): param1, param2, ..., param8\\
                1, 2, 3, 4 are a quaternion represention the desired rotation\\
                5, 6, 7 are roll, pitch, and yaw rates respectively\\
                8 is thrust
        """
        if len(param) != 8:
            self.get_logger().info('SET_ATTITUDE_TARGET need 8 params')

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
        rc_channel_values[self.rc_override_mapping['x']] = pwm_linear_x
        rc_channel_values[self.rc_override_mapping['y']] = pwm_linear_y
        rc_channel_values[self.rc_override_mapping['z']] = pwm_linear_z
        rc_channel_values[self.rc_override_mapping['yaw']] = pwm_angular_z
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
        vel_x, vel_y, vel_z = msg.linear.x, msg.linear.y, msg.linear.z
        omega_z = msg.angular.z

        vel_x = max(-self.MAX_VEL, min(self.MAX_VEL, vel_x))
        vel_y = max(-self.MAX_VEL, min(self.MAX_VEL, vel_y))
        vel_z = max(-self.MAX_VEL, min(self.MAX_VEL, vel_z))
        omega_z = max(-self.MAX_OMEGA, min(self.MAX_OMEGA, omega_z))

        x = 1500 + int(self.VEL_TO_CMD * vel_x)
        y = 1500 + int(self.VEL_TO_CMD * vel_y)
        z = 1500 + int(self.VEL_TO_CMD * vel_z)
        yaw = 1500 + int(self.OMEGA_TO_CMD * omega_z)

        return x, y, z, yaw
    
    def cmd_vel_callback(self, msg):
        '''
        Callback for ROS cmd_vel messages to convert them to PWM values and send those to the vehicle.

        Args:
            msg (Twist): Velocity command to convert to PWM
        '''
        if self.cmd_vel_enabled:
            self.set_cmd_pwm(*self.cmd_vel_to_pwm(msg))
        else:
            return
        
    # Services
    def position_target_local_ned_callback(self, request: PositionTargetLocalNED.Request, response: PositionTargetLocalNED.Response):
        '''
        Service callback to set a position target
        '''
        target = [request.x, request.y, request.z, request.vx, request.vy, request.vz,
                  request.ax, request.ay, request.az, request.yaw, request.yaw_rate]
        
        mask = request.bit_mask

        try:
            self.set_position_target_local_ned(target, mask)
            
            response.position_target_set = True
        except:
            response.position_target_set = False
        return response
    
    def set_mode_callback(self, request: SetMAVMode.Request, response: SetMAVMode.Response):
        '''
        Service callback to set the mode of the vehicle
        '''
        try:
            self.set_mode(request.mode)
            response.mode_set = True
        except:
            response.mode_set = False

        return response
    
    def set_arm_disarm_callback(self, request: SetArmDisarm.Request, response: SetArmDisarm.Response):
        '''
        Service callback to arm or disarm the vehicle
        '''
        try:
            self.arm_disarm(request.set_arm)
            response.arm_state_set = True
        except:
            response.arm_state_set = False
            
        return response

    # Publishers
    def data_pub(self):
        '''
        Publishes all data on a timer set in init
        '''
        self.pose_publisher_.publish(self.odom)
        self.tf_broadcaster.sendTransform(self.t)
        if self.get_parameter('has_camera').value:
            self.tf_broadcaster.sendTransform(self.camera_transform)

        self.imu_publisher_.publish(self.imu)
        self.gps_publisher_.publish(self.gps)    

    # Data collection from connected vehicle
    def get_vehicle_data(self):
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
    
    def parse_vehicle_data(self):
        
        msgs = self.get_vehicle_data()
        
        for msg_raw in msgs:
            # self.get_logger().info(f'{msg.to_dict()}')
            msg = msg_raw.to_dict()
            msg_type = msg_raw.get_type()
            now = self.get_clock().now().to_msg()

            if msg == None or msg_type == 'BAD_DATA':
                pass

            elif msg_type == 'HEARTBEAT':
                base_mode = msg['base_mode']
                is_autopilot = True if msg['autopilot'] != 8 else False
                if is_autopilot:
                    self.armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            elif msg_type == 'SCALED_IMU2':
                angular_vel_ned = np.array([msg['xgyro'], msg['ygyro'], msg['zgyro']], dtype=float) / 1000
                angular_vel_enu = self.ned_to_enu.apply(angular_vel_ned)

                linear_accel_ned = 9.81 * np.array([msg['xacc'], msg['yacc'], msg['zacc']], dtype=float) / 1000
                linear_accel_enu = self.ned_to_enu.apply(linear_accel_ned)

                # Odom message
                self.odom.twist.twist.angular.x =  angular_vel_enu[0]
                self.odom.twist.twist.angular.y =  angular_vel_enu[1]
                self.odom.twist.twist.angular.z =  angular_vel_enu[2]

                # IMU sensor message
                self.imu.header.stamp = now

                self.imu.orientation = self.odom.pose.pose.orientation

                self.imu.angular_velocity.x = angular_vel_enu[0]
                self.imu.angular_velocity.y = angular_vel_enu[1]
                self.imu.angular_velocity.z = angular_vel_enu[2]
                self.imu.linear_acceleration.x = linear_accel_enu[0]
                self.imu.linear_acceleration.y = linear_accel_enu[1]
                self.imu.linear_acceleration.z = linear_accel_enu[2]

            elif msg_type == 'LOCAL_POSITION_NED':
                linear_vel_ned = np.array([msg['vx'], msg['vy'], msg['vz']])
                position_ned = np.array([msg['x'], msg['y'], msg['z']])

                linear_vel_enu = self.ned_to_enu.apply(linear_vel_ned)
                position_enu = self.ned_to_enu.apply(position_ned)

                # Odom message
                self.odom.pose.pose.position.x = position_enu[0]
                self.odom.pose.pose.position.y = position_enu[1]
                self.odom.pose.pose.position.z = position_enu[2]

                self.odom.twist.twist.linear.x =  linear_vel_enu[0]
                self.odom.twist.twist.linear.y =  linear_vel_enu[1]
                self.odom.twist.twist.linear.z =  linear_vel_enu[2]

                # TF message
                self.t.transform.translation.x = position_enu[0]
                self.t.transform.translation.y = position_enu[1]
                self.t.transform.translation.z = position_enu[2]

            elif msg_type == 'ATTITUDE':
                # Rotated into front, left, up frame
                orientation_ned = Rotation.from_euler('xyz', [msg['roll'], msg['pitch'], msg['yaw']], degrees=False)
                ned_to_flu = Rotation.from_euler('xyz', [180, 0, 0], degrees=True)
                orientation_flu = (ned_to_flu.inv() * orientation_ned * ned_to_flu).as_quat()
                
                # Odom message
                self.odom.header.stamp = now

                self.odom.pose.pose.orientation.w = orientation_flu[3]
                self.odom.pose.pose.orientation.x = orientation_flu[0]
                self.odom.pose.pose.orientation.y = orientation_flu[1]
                self.odom.pose.pose.orientation.z = orientation_flu[2]
                
                # TF message
                self.t.header.stamp = now

                self.t.transform.rotation.x = orientation_flu[0]
                self.t.transform.rotation.y = orientation_flu[1]
                self.t.transform.rotation.z = orientation_flu[2]
                self.t.transform.rotation.w = orientation_flu[3]

            elif msg_type == 'GPS_RAW_INT':
                self.gps.header.stamp = now

                if msg['fix_type'] == 1:
                    status = -1
                elif msg['fix_type'] == 2 or msg['fix_type'] == 3:
                    status = 0
                elif msg['fix_type'] == 4:
                    status = 1
                elif msg['fix_type'] == 5 or msg['fix_type'] == 6:
                    status = 2
                else:
                    status = -1

                self.gps.status.status = status
                self.gps.status.service = 1

                self.gps.latitude = msg['lat'] / (10**7)
                self.gps.longitude = msg['lon'] / (10**7)
                self.gps.altitude = msg['alt'] / 1000.0

            elif msg_type == 'GIMBAL_DEVICE_ATTITUDE_STATUS':
                if self.get_parameter('has_camera').value:
                    # TODO Confirm order of rotations
                    camera_rot = (self.camera_orientation *\
                                Rotation.from_euler('xyz', [0.0,
                                                            np.interp(Rotation.from_quat([*msg['q'][1:], msg['q'][0]]).as_euler('xyz')[1],
                                                                        self.camera_gimbal_mapping[:1],
                                                                        self.camera_gimbal_mapping[2:]),
                                                            0.0])).as_quat()

                    self.camera_transform.transform.rotation.x = camera_rot[0]
                    self.camera_transform.transform.rotation.y = camera_rot[1]
                    self.camera_transform.transform.rotation.z = camera_rot[2]
                    self.camera_transform.transform.rotation.w = camera_rot[3]
            
            # else:
            #     self.get_logger().info(f'{msg_type}, {msg}')


def main():
    rclpy.init()

    mavlink_connection = MAVLink_Router()

    rclpy.spin(mavlink_connection)

    mavlink_connection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()