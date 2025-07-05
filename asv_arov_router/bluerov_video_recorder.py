import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from asv_arov_interfaces.srv import SetRecording

class BlueROV_Video_Recorder(Node):
    def __init__(self):
        super().__init__('bluerov_video_recorder')
        self.subscription = self.create_subscription(
            Image,
            f'{self.get_namespace()}/image_rect',
            self.listener_callback,
            10)
        self.br = CvBridge()

        self.srv = self.create_service(SetRecording, f'{self.get_namespace()}/set_recording', self.set_recording_callback)
        self.recording = False

        self.size = (2592, 1944) # Barlus video input
        # self.size = (1920, 1080) # BlueROV video input
        self.frame_rate = 20

        self.output_length = 120 # Length of each output file in seconds

        self.start_file()
        self.start_timer = self.create_timer(self.output_length, self.start_file)
        self.write_video_timer = self.create_timer(1/self.frame_rate, self.write_video)
    
    def listener_callback(self, data):
        self.current_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)
        # self.current_frame = self.br.imgmsg_to_cv2(data)
        # self.get_logger().info(f'Receiving video frame, {self.current_frame.shape}')

    def write_video(self):
        if self.recording:
            try:
                self.video_writer.write(self.current_frame)
            except AttributeError:
                self.get_logger().warn('No images being received')

    def start_file(self):
        if self.recording:
            video_name = f"data_log/videos/cam1_output_{self.get_clock().now().seconds_nanoseconds()[0]}.mp4"
            self.video_writer = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'mp4v'), self.frame_rate, self.size)
            self.get_logger().info(f"Starting new recording: {video_name}")

    def set_recording_callback(self, request, response):
        self.recording = request.set_recording
        response.recording_state = self.recording

        if self.recording:
            self.start_file()
        else:
            self.video_writer.release()
            self.get_logger().info("Stoping recording")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    bluerov_video_recorder = BlueROV_Video_Recorder()
    rclpy.spin(bluerov_video_recorder)
    bluerov_video_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()