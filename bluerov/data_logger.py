import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped

class Data_Logger(Node):
    def __init__(self):
        super().__init__('data_logger')

        self.bluerov_accel = self.create_subscription(
            AccelStamped,
            'bluerov/accel',
            self.bluerov_accel_callback,
            10)
        self.bluerov_accel  # prevent unused variable warning

        self.bluerov_yaw_rate = self.create_subscription(
            TwistStamped,
            'bluerov/yaw_rate',
            self.bluerov_yaw_rate_callback,
            10)
        self.bluerov_yaw_rate  # prevent unused variable warning

        self.bluerov_pose = self.create_subscription(
            PoseStamped,
            'bluerov/pose_from_dvl',
            self.bluerov_pose_callback,
            10)
        self.bluerov_pose  # prevent unused variable warning

    def bluerov_accel_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.accel)

    def bluerov_yaw_rate_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.twist)

    def bluerov_pose_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.pose)


def main(args=None):
    rclpy.init(args=args)

    data_logger = Data_Logger()

    rclpy.spin(data_logger)
    data_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
