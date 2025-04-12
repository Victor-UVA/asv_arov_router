import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time

class Data_Heartbeat(Node):
    def __init__(self):
        super().__init__('data_log_heartbeat')
        self.publisher_ = self.create_publisher(Time, 'data_log_heartbeat', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # msg = Time()
        msg = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    heartbeat = Data_Heartbeat()

    rclpy.spin(heartbeat)
    heartbeat.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
