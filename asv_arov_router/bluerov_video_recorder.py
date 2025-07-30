import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from asv_arov_interfaces.srv import SetRecording

class BlueROV_Video_Recorder(Node):
    def __init__(self):
        super().__init__('bluerov_video_recorder')
        # self.cam1_sub = self.create_subscription(Image, '/cam1/image_rect', self.listener_callback, 10)
        # self.cam2_sub = self.create_subscription(Image, '/cam2/image_rect', self.listener_callback, 10)
        # self.cam3_sub = self.create_subscription(Image, '/cam3/image_rect', self.listener_callback, 10)
        # self.cam4_sub = self.create_subscription(Image, '/cam4/image_rect', self.listener_callback, 10)
        self.arovcam_sub = self.create_subscription(Image, '/arov/image_rect', self.listener_callback, 10)
        self.br = CvBridge()

        self.srv = self.create_service(SetRecording, f'{self.get_namespace()}/set_recording', self.set_recording_callback)
        self.recording = False

        self.size = (2592, 1944) # Barlus cameras covering mort trap video input
        barlus_resize_factor = (1080/2592)
        self.resized = (int(barlus_resize_factor*self.size[0]), int(barlus_resize_factor*self.size[1]))
        self.arov_size = (3072, 2048) # BlueROV video input
        self.frame_rate = 20

        self.output_length = 1200 # Length of each output file in seconds

        self.start_file()
        self.start_timer = self.create_timer(self.output_length, self.start_file)
        self.write_video_timer = self.create_timer(1/self.frame_rate, self.write_video)
    
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
            cam2_resized = cv2.resize(self.cam2_frame, self.resized)
            cv2.imshow('/cam2', cam2_resized)
            cv2.waitKey(1)
        elif namespace == '/cam3':
            self.cam3_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)
            cam3_resized = cv2.resize(self.cam3_frame, self.resized)
            cv2.imshow('/cam3', cam3_resized)
            cv2.waitKey(1)
        elif namespace == '/cam4':
            self.cam4_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)
            cam4_resized = cv2.resize(self.cam4_frame, self.resized)
            cv2.imshow('/cam4', cam4_resized)
            cv2.waitKey(1)
        elif namespace == '/arov':
            self.arov_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)
            # cv2.imshow('/arov', self.arov_frame)
            cv2.waitKey(1)
        # self.current_frame = self.br.imgmsg_to_cv2(data)
        # self.get_logger().info(f'Receiving video frame, {self.current_frame.shape}')

    def write_video(self):
        if self.recording:
            try:
                # self.video_writer_cam1.write(self.cam1_frame)
                # self.video_writer_cam2.write(self.cam2_frame)
                # self.video_writer_cam3.write(self.cam3_frame)
                # self.video_writer_cam4.write(self.cam4_frame)
                self.video_writer_arov.write(self.arov_frame)
            except AttributeError:
                self.get_logger().warn('No images being received')

    def start_file(self):
        if self.recording:
            video_name = f"data_log/videos/cam_output_{self.get_clock().now().seconds_nanoseconds()[0]}"
            # self.video_writer_cam1 = cv2.VideoWriter(video_name + 'cam1.mp4', cv2.VideoWriter_fourcc(*'mp4v'), self.frame_rate, self.size)
            # self.video_writer_cam2 = cv2.VideoWriter(video_name + 'cam2.mp4', cv2.VideoWriter_fourcc(*'mp4v'), self.frame_rate, self.size)
            # self.video_writer_cam3 = cv2.VideoWriter(video_name + 'cam3.mp4', cv2.VideoWriter_fourcc(*'mp4v'), self.frame_rate, self.size)
            # self.video_writer_cam4 = cv2.VideoWriter(video_name + 'cam4.mp4', cv2.VideoWriter_fourcc(*'mp4v'), self.frame_rate, self.size)
            self.video_writer_arov = cv2.VideoWriter(video_name + 'arov.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, self.arov_size)
            self.get_logger().info(f"Starting new recording: {video_name}")

    def set_recording_callback(self, request, response):
        self.recording = request.set_recording
        response.recording_state = self.recording

        if self.recording:
            self.start_file()
        else:
            # self.video_writer_cam1.release()
            # self.video_writer_cam2.release()
            # self.video_writer_cam3.release()
            # self.video_writer_cam4.release()
            self.video_writer_arov.release()
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
