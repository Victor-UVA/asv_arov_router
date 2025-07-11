#!/usr/bin/bash

# Run from home/maddy on the ASV computer
# bash ros2_ws/src/asv_arov_router/maddy_startup.bash
echo "Today is " `date`
# cd ~/ros2_ws
# source install/setup.bash

NETWORK=VICTOR

case $NETWORK in

  VICTOR)
    GATE=172.16.0
    GCS=$GATE.30
    ;;

  PORTABLE)
    GATE=192.168.1
    GCS=$GATE.35
    ;;
esac


gst-launch-1.0 udpsrc port=5600 ! queue ! udpsink host=$GCS port=5500 &

# mavproxy.py --master=/dev/ttyACM1 --out=udpbcast:$GATE.255:14552 --out=udpbcast:localhost:14553 --daemon --cmd="set moddebug 2; module load optitrack; optitrack set server $SERVER; optitrack set client $CLIENT; optitrack set obj_id $OBJ_ID; optitrack start;" &
mavproxy.py --master=/dev/ttyACM0 --out=udpbcast:$GATE.255:14553 --daemon & # ASV
mavproxy.py --master=udpin:192.168.2.1:14550 --out=udpbcast:$GATE.255:14550 --daemon && fg # AROV

# ros2 launch asv_arov_router asv.launch.py && fg


wait
