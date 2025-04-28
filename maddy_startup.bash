#!/usr/bin/bash

# Run from home/maddy on the ASV computer
# bash ros2_ws/src/bluerov/maddy_startup.bash
echo "Today is " `date`
cd ~/ros2_ws
source install/setup.bash

NETWORK=PORTABLE

case $NETWORK in

  VICTOR)
    GATE=172.16.0
    GCS=172.16.0.16
    ;;

  PORTABLE)
    GATE=192.168.1
    GCS=192.168.1.35
    ;;
esac


gst-launch-1.0 udpsrc port=5600 ! queue ! udpsink host=$GCS port=5502 &

mavproxy.py --master=/dev/ttyACM0 --out=udpbcast:$GATE.255:14552 --out=udpbcast:localhost:14553 --daemon &
mavproxy.py --master=udpin:192.168.2.1:14550 --out=udpbcast:$GATE.255:14550 --out=udpbcast:localhost:14551 --daemon &

ros2 launch bluerov asv.launch.py && fg


wait