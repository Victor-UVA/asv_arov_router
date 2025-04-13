# BlueROV
ROS 2 nodes for collecting data from the BlueROV and surface vehicle.

# Installing gscam2 Node
Needed to get the video from the BlueROV as a ROS topic.

1. Install dependencies ''apt-get install -y libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
 gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools \
 gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
 libgstreamer-plugins-base1.0-dev''
2. In the ROS workspace, go to the src folder.
3. ''git clone https://github.com/ptrmu/ros2_shared.git -b $master''
4. ''git clone https://github.com/clydemcqueen/gscam2.git''
5. ''cd ..''
6. ''rosdep install -y --from-paths . --ignore-src''
7. ''source /opt/ros/$TARGET_ROS_DISTRO/setup.bash && colcon build''

Between 6 and 7, may also need to run:
- ''sudo apt install ros-jazzy-camera-info-manager''
- ''sudo apt install ros-jazzy-camera-calibration-parsers''