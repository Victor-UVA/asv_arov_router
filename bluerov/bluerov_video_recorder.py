import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class BlueROV_Video_Recorder(Node):
    def __init__(self):
        super().__init__('bluerov_video_recorder')
        self.subscription = self.create_subscription(
            Image,
            '/image_rect',
            self.listener_callback,
            10)
        self.br = CvBridge()

        # self.cap = cv2.VideoCapture()

        self.size = (1920, 1080) # Webcam video input
        # self.size = (1920, 1080) # BlueROV video input
        self.frame_rate = 30

        self.output_length = 10 # Length of each output file in seconds

        self.start_file()
        self.start_timer = self.create_timer(self.output_length, self.start_file)
        self.write_video_timer = self.create_timer(1/self.frame_rate, self.write_video)
    
    def listener_callback(self, data):
        self.current_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)
        # self.current_frame = self.br.imgmsg_to_cv2(data)
        # self.get_logger().info(f'Receiving video frame, {self.size}')

    def write_video(self):
        try:
            self.video_writer.write(self.current_frame)
        except AttributeError:
            self.get_logger().warn('No images yet')

    def start_file(self):
        self.video_writer = cv2.VideoWriter(f"data_log/videos/bluerov_output_{self.get_clock().now().seconds_nanoseconds()[0]}.mp4",
                                            cv2.VideoWriter_fourcc(*'mp4v'), self.frame_rate, self.size)

def main(args=None):
    rclpy.init(args=args)
    bluerov_video_recorder = BlueROV_Video_Recorder()
    rclpy.spin(bluerov_video_recorder)
    bluerov_video_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()