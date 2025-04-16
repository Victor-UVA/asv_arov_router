# BlueROV
ROS 2 nodes for collecting data from the BlueROV and surface vehicle.

# Installing Nodes and Dependencies
## Python Dependencies
- pymavlink
- scipy

## [gscam2](https://github.com/clydemcqueen/gscam2)
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

## [apriltag_ros](https://index.ros.org/p/apriltag_ros/#jazzy-overview)
- ```sudo apt install ros-jazzy-apriltag-ros```

## [robot_localization](http://docs.ros.org/en/jade/api/robot_localization/html/index.html)
Used to transform the GPS info from the USV into the robots' global frame.
No longer needed, but might be useful in the future.

- ```sudo apt install ros-jazzy-robot-localization```
- Go [here](http://www.ngdc.noaa.gov/geomag-web) to get magnetic declination for input (at Chris Greene Lake: 9° 49' W  ± 0° 22' -> 0.17133316072 rad) and add it to the ```bluerov_launch.xml``` file

# Running Things
## Hardware Setup
- Power on USV and turn on its computer
- Power on and seal up BlueROV
- Connect BlueROV tether to the USV
- Set up router with NETGEAR21 and connect ground station to it (if USV doesn't connected automatically, get a screen)
    - Password: breezygadfly716
    - Set ground station IP to 192.168.1.35
- Run ```ssh maddy@192.168.1.4``` to connect to USV from ground station to run the neccessary commands below

## gscam2 Node Connected to BlueROV Camera
### To Get Video on USV
- ```export GSCAM_CONFIG="udpsrc port=5601 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=RGB ! queue ! videoconvert"```
- ```ros2 run gscam2 gscam_main```
- Note: The port that the video stream is exposed at on the BlueROV needs to be an odd number so that it's treated as a video stream and not audio.

### To Stream Video to Ground Station
- On the USV computer:
    - If only getting video on ROS, run ```gst-launch-1.0 udpsrc port=5601 ! queue ! udpsink host=192.168.1.35 port=5500```
    - If only getting video on QGroundControl, run ```gst-launch-1.0 udpsrc port=5601 ! queue ! udpsink host=192.168.1.35 port=5502```
    - If getting video in ROS and on QGroundControl, run ```gst-launch-1.0 udpsrc port=5601 ! queue ! multiudpsink clients=192.168.1.35:5500,192.168.1.35:5502```
    - Note: Change host/clients as needed to the computer receiving the stream
    - This forwards the video stream from the BlueROV to USV to the ground station computer
- To get the video stream into ROS, on the ground station:
    - ```export GSCAM_CONFIG="udpsrc port=5500 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=RGB ! queue ! videoconvert"```
    - ```ros2 run gscam2 gscam_main --ros-args -p camera_info_url:="file:///home/malori/ros2_ws/bluerov/ost.yaml" -p camera_name:="narrow_stereo"```
- To get the video stream in QGroundControl, on the ground station:
    - Change the port that QGroundControl receives video on to ```5502``` (or change the port that the stream is forwarded on to ```5600``` in the command on the USV)


## Apriltag Node
- ```ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image_raw -r camera_info:=/camera_info --params-file /home/malori/ros2_ws/bluerov/apriltag_node_config.yaml```

## Launch File
- Once the package has been built and sourced, run ```ros2 launch bluerov bluerov_launch.xml```
- Run gstreamer commands with instructions above
- Run apriltag node with instructions above
- Run telemetry streaming with instructions below

## Streaming MavLink Telemetry to Ground Station
### BlueROV
- On the USV, run ```mavproxy.py --master=udpin:192.168.2.1:14550 --out=udpbcast:192.168.1.255:14550 --out=udpbcast:192.168.1.255:14551```

### USV
- On the USV, run ```mavproxy.py --master=/dev/ttyACM0 --out=udpbcast:192.168.1.255:14552 --out=udpbcast:192.168.1.255:14553```
    - Note: Master may be ```/dev/ttyACM0``` or ```/dev/ttyACM1```, it seems to change sometimes.  If needed, run ```ls /dev/tty*``` to find out (run that with the Pixhawk on the USV plugged in with the USB cord and with it unplugged, and find the difference in the output)


# Camera Calibration
The gscam2 node needs the camera calibration data from the BlueROV's camera so that it can pass it into the /camera_info topic that the apriltag node then uses to calibrate its detections.  To do the calibration:
- Install ROS2 camera calibration node by running ```sudo apt install ros-jazzy-camera-calibration```
- Print a calibration pattern (details in linked post below)
- Stream the camera feed from the BlueROV and run gscam2 as above
- Launch the calibration node ```ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.015 --ros-args -r image:=/image_raw```
- [Link](https://medium.com/starschema-blog/offline-camera-calibration-in-ros-2-45e81df12555) to post with directions and additional info
- [Other link](https://docs.nav2.org/tutorials/docs/camera_calibration.html) with more details