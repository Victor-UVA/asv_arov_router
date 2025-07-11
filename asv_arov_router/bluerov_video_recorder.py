import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from asv_arov_interfaces.srv import SetRecording

class BlueROV_Video_Recorder(Node):
    def __init__(self):
        super().__init__('bluerov_video_recorder')
        self.cam1_sub = self.create_subscription(Image, '/cam1/image_rect', self.listener_callback, 10)
        self.cam2_sub = self.create_subscription(Image, '/cam2/image_rect', self.listener_callback, 10)
        self.cam3_sub = self.create_subscription(Image, '/cam3/image_rect', self.listener_callback, 10)
        self.cam4_sub = self.create_subscription(Image, '/cam4/image_rect', self.listener_callback, 10)
        # self.arovcam_sub = self.create_subscription(Image, '/arov/image_rect', self.listener_callback, 10)
        self.br = CvBridge()

        self.srv = self.create_service(SetRecording, f'{self.get_namespace()}/set_recording', self.set_recording_callback)
        self.recording = False

        self.size = (2592, 1944) # Barlus cameras covering mort trap video input
        self.resized = (1920/2592)*self.size
        self.arov_size = (1920, 1080) # BlueROV video input
        self.frame_rate = 20

        self.output_length = 120 # Length of each output file in seconds

        # self.start_file()
        # self.start_timer = self.create_timer(self.output_length, self.start_file)
        # self.write_video_timer = self.create_timer(1/self.frame_rate, self.write_video)
    
    def listener_callback(self, data):
        namespace = data.header.frame_id[:-7]
        if namespace == '/cam1':
            self.cam1_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)
            # self.current_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)
            cam1_resized = cv2.resize(self.cam1_frame, self.resized)
            cv2.imshow('/cam1', cam1_resized)
            cv2.waitKey(1)
        elif namespace == '/cam2':
            self.cam2_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)
            cam2_resized = cv2.resize(self.cam1_frame, self.resized)
            cv2.imshow('/cam2', cam2_resized)
            cv2.waitKey(1)
        elif namespace == '/cam3':
            self.cam3_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)
            cv2.imshow('/cam3', self.cam3_frame)
            cv2.waitKey(1)
        elif namespace == '/cam4':
            self.cam4_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)
            cv2.imshow('/cam4', self.cam4_frame)
            cv2.waitKey(1)
        elif namespace == '/arov':
            self.arov_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)
            cv2.imshow('/arov', self.arov_frame)
            cv2.waitKey(1)
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
            self.get_logger().info("Stopping recording")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    bluerov_video_recorder = BlueROV_Video_Recorder()
    rclpy.spin(bluerov_video_recorder)
    bluerov_video_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
