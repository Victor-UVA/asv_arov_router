#!/usr/bin/bash

# Run from home/maddy on the ASV computer
echo "Today is " `date`

gst-launch-1.0 udpsrc port=5600 ! queue ! udpsink host=172.16.0.16 port=5502 &

mavproxy.py --master=/dev/ttyACM0 --out=udpbcast:172.16.0.255:14552 --out=udpbcast:localhost:14553 --daemon &
mavproxy.py --master=udpin:192.168.2.1:14550 --out=udpbcast:172.16.0.255:14550 --out=udpbcast:localhost:14551 --daemon $

cd ~/ros2_ws
source install/setup.bash
ros2 launch bluerov asv.launch.py && fg


wait