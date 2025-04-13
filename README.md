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
3. ```git clone https://github.com/ptrmu/ros2_shared.git -b $master```
4. ```git clone https://github.com/clydemcqueen/gscam2.git```
5. ```cd ..```
6. ```rosdep install -y --from-paths . --ignore-src```
7. ```source /opt/ros/$TARGET_ROS_DISTRO/setup.bash && colcon build```

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
- ```export GSCAM_CONFIG="v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! avdec_h264 ! videoconvert"```
- ```ros2 run gscam2 gscam_main```

## Apriltag Node
- ```ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image_raw -r camera_info:=/camera_info --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml```