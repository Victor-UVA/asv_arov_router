from mavros import mavlink
import rclpy
from rclpy.node import Node

class BlueROV_Listener(Node):
    def __init__(self):
        super().__init__('bluerov_listener')
    
        print('Hi from bluerov.')
        
        self.master = mavlink.mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.master.wait_heartbeat()

        print('BlueROV connected!.')
        self.get_msg()

        self.create_timer(0.01, self.get_msg)

    def get_msg(self):
        msg = self.master.recv_match()
        print(type(msg).__name__)


def main():
    rclpy.init()

    bluerov_listener = BlueROV_Listener()

    rclpy.spin(bluerov_listener)

    bluerov_listener.destroy_node()
    rclpy.shutdown()

# ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://0.0.0.0:14550@

if __name__ == '__main__':
    main()
