#!/usr/bin/bash

# Run from ~ on the GCS computer
# bash ros2_ws/src/asv_arov_router/gcs_startup.bash
echo "Today is " `date`
cd ~/ros2_ws
source install/setup.bash

NETWORK=VICTOR

case $NETWORK in

  VICTOR)
    GATE=172.16.0
    ASV=$GATE.18
    ;;

  PORTABLE)
    GATE=192.168.1
    ASV=$GATE.4
    ;;
esac

gst-launch-1.0 udpsrc port=5500 ! queue ! multiudpsink clients=localhost:5501,localhost:5502 &

mavproxy.py --master=udpin:$ASV:14552 --out=udpbcast:localhost:14552 --out=udpbcast:localhost:14553 --daemon & # ASV 
mavproxy.py --master=udpin:$ASV:14550 --out=udpbcast:localhost:14550 --out=udpbcast:localhost:14551 --daemon & # AROV 

ros2 launch asv_arov_router asv.launch.py && fg


wait
