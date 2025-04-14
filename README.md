# BlueROV
ROS 2 nodes for collecting data from the BlueROV and surface vehicle.

# Installing Nodes and Dependencies
## Python Dependencies
- pymavlink
- scipy

## gscam2
Needed to get the video from the BlueROV as a ROS topic.

1. Install dependencies ```apt-get install -y libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
 gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools \
 gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
 libgstreamer-plugins-base1.0-dev```
2. In the ROS workspace, go to the src folder.
3. ```git clone https://github.com/ptrmu/ros2_shared.git -b master```
4. ```git clone https://github.com/clydemcqueen/gscam2.git```
5. ```cd ..```
6. ```rosdep install -y --from-paths . --ignore-src```
7. ```source /opt/ros/jazzy/setup.bash && colcon build```

Between 6 and 7, may also need to run:
- ```sudo apt install ros-jazzy-camera-info-manager```
- ```sudo apt install ros-jazzy-camera-calibration-parsers```

For testing with computer webcam:
- ```export GSCAM_CONFIG="v4l2src name=cam_src ! decodebin ! videoconvert ! videoscale ! video/x-raw,format=RGB ! queue ! videoconvert"```
- ```ros2 run gscam2 gscam_main```
- In RViz, create an image visualization and subscribe to /image_raw

## apriltag_ros
- ```sudo apt install ros-jazzy-apriltag-ros```


# Running Things
## gscam2 Node Connected to BlueROV Camera
### To Get Video on USV
- ```export GSCAM_CONFIG="udpsrc port=5601 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=RGB ! queue ! videoconvert"```
- ```ros2 run gscam2 gscam_main```
- Note: The port that the video stream is exposed at on the BlueROV needs to be an odd number so that it's treated as a video stream and not audio.

### To Stream Video to Ground Station
- On the USV computer:
    - If only getting video on ROS, run ```gst-launch-1.0 udpsrc port=5601 ! queue ! udpsink host=172.16.0.16 port=5500```
    - If only getting video on QGroundControl, run ```gst-launch-1.0 udpsrc port=5601 ! queue ! udpsink host=172.16.0.16 port=5502```
    - If getting video in ROS and on QGroundControl, run ```gst-launch-1.0 udpsrc port=5601 ! queue ! multiudpsink clients=172.16.0.16:5500,172.16.0.16:5502```
    - Note: Change host/clients as needed to the computer receiving the stream
    - This forwards the video stream from the BlueROV to USV to the ground station computer
- To get the video stream into ROS, on the ground station:
    - ```export GSCAM_CONFIG="udpsrc port=5500 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=RGB ! queue ! videoconvert"```
    - ```ros2 run gscam2 gscam_main```
- To get the video stream in QGroundControl, on the ground station:
    - Change the port that QGroundControl receives video on to ```5502``` (or change the port that the stream is forwarded on to ```5600``` in the command on the USV)

## Apriltag Node
- ```ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image_raw -r camera_info:=/camera_info --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml```

## Streaming MavLink Telemetry to Ground Station
### BlueROV
- On the USV, run ```mavproxy.py --master=udpin:192.168.2.1:14550 --out=udpbcast:172.16.0.255:14550 --out=udpbcast:172.16.0.255:14551```

### USV
- On the USV, run ```mavproxy.py --master=/dev/ttyACM0 --out=udpbcast:172.16.0.255:14552```