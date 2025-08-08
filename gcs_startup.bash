#!/usr/bin/bash

# Run from ~ on the GCS computer
# bash ros2_ws/src/asv_arov_router/gcs_startup.bash VICTOR DIRECT
cd ~/ros2_ws
source install/setup.bash

NETWORK=$1
AROV_CONNECTION=$2

case $NETWORK in

  VICTOR)
    GATE=172.16.0
    ASV=$GATE.18
    OPTI=$GATE.22
    ;;

  PORTABLE)
    GATE=192.168.1
    ASV=$GATE.4
    OPTI=$GATE.2
    ;;
esac

case $AROV_CONNECTION in

  DIRECT)
    # For connecting to the AROV directly through the Blue Box
    gst-launch-1.0 udpsrc port=5600 ! queue ! multiudpsink clients=127.0.0.1:5501,127.0.0.1:5502 &
    mavproxy.py --master=udpin:192.168.2.1:14550 --out=udpbcast:localhost:14551 --out=udpbcast:localhost:14552 --daemon &
    ros2 launch asv_arov_router asv.launch.py && fg
    wait
    ;;

  ASV)
    # For connecting to the AROV through the ASV
    gst-launch-1.0 udpsrc port=5500 ! queue ! multiudpsink clients=127.0.0.1:5501,127.0.0.1:5502 &
    mavproxy.py --master=udp:0.0.0.0:14553 --out=udpbcast:localhost:14554 --out=udpbcast:localhost:14555 --out=udpbcast:$OPTI:14556 --daemon & # ASV 
    mavproxy.py --master=udp:0.0.0.0:14550 --out=udpbcast:localhost:14551 --out=udpbcast:localhost:14552 --daemon & # AROV 
    ros2 launch asv_arov_router asv.launch.py && fg
    wait
    ;;
esac