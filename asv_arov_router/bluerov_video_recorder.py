import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from asv_arov_interfaces.srv import SetRecording
import numpy as np

class BlueROV_Video_Recorder(Node):
    def __init__(self):
        super().__init__('bluerov_video_recorder')
        self.declare_parameters(namespace='',parameters=[
            ('camera_list', ['']),
            ('frame_rates', [25]),
            ('display_list', [False]),
            ('display_size', [1920, 1080]),
            ('display_layout', [1, 1]) # Images per row, rows
            ])

        self.camera_info_subs = []
        self.camera_image_subs = []
        self.sizes = []
        self.frames = []
        self.frame_rates = self.get_parameter('frame_rates').value
        self.display_list = self.get_parameter('display_list').value
        self.video_writers = []
        self.writing_cameras = []
        self.write_times = []

        for camera in self.get_parameter('camera_list').value :
            self.camera_info_subs.append(self.create_subscription(CameraInfo, f'{camera}/camera_info', self.info_callback, 10))
            self.camera_image_subs.append(self.create_subscription(Image, f'{camera}/image_rect', self.image_callback, 10))
            self.sizes.append((0,0))
            self.frames.append(None)
        
        self.br = CvBridge()

        self.srv = self.create_service(SetRecording, f'/set_recording', self.set_recording_callback)
        self.recording = False

        self.create_timer(0.05,self.display_images)
    
    def info_callback(self, data: CameraInfo) :
        namespace = data.header.frame_id[:-7]
        idx = self.get_parameter('camera_list').value.index(namespace)
        self.sizes[idx] = (data.width, data.height)

    def image_callback(self, data) :
        namespace = data.header.frame_id[:-7]
        idx = self.get_parameter('camera_list').value.index(namespace)

        self.frames[idx] = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)

    def display_images(self) :
        columns = self.get_parameter('display_layout').value[0]
        rows = self.get_parameter('display_layout').value[1]

        for idx, frame in enumerate(self.frames) :
            if frame is not None and self.display_list[idx] :
                cv2.imshow(self.get_parameter('camera_list').value[idx], frame)
        cv2.waitKey(1)

    def write_video(self) :
        if self.recording :
            now = self.get_clock().now().seconds_nanoseconds()
            now = now[0] + now[1] * 1e-9
            for wrt, camera in enumerate(self.writing_cameras) :
                idx = self.get_parameter('camera_list').value.index(camera)
                if self.frames[idx] is not None and (now - self.write_times[wrt]) >= 1/self.frame_rates[idx] :

                    self.write_times[wrt] = now
                    try :
                        self.video_writers[wrt].write(self.frames[idx])
                    except AttributeError :
                        self.get_logger().warn(f'Error writing {camera}')

    def start_file(self):
        if self.recording:
            now = self.get_clock().now().seconds_nanoseconds()
            for idx, camera in enumerate(self.get_parameter('camera_list').value) :
                if self.frames[idx] is not None :
                    video_name = f"data_log/videos/{camera}_{now[0]}.mp4"
                    self.video_writers.append(cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'mp4v'), self.frame_rates[idx], self.sizes[idx]))
                    self.writing_cameras.append(camera)
                    self.write_times.append(now[0] + now[1] * 1e-9)
                    self.get_logger().info(f"Starting new recording: {video_name}")

            self.video_timer = self.create_timer(0.01, self.write_video)

    def set_recording_callback(self, request, response):
        self.recording = request.set_recording
        response.recording_state = self.recording

        if self.recording:
            self.start_file()
        else:
            self.video_timer.destroy()
            for writer in self.video_writers :
                writer.release()
            
            self.video_writers = []
            self.writing_cameras = []
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
