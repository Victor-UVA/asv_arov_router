from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    ld = LaunchDescription()

    bluerov_node = Node(
        package="asv_arov_router",
        executable="bluerov_connection",
        name="bluerov_connection"
    )

    maddy_node = Node(
        package="asv_arov_router",
        executable="maddy_connection",
        name="maddy_connection"
    )

    data_logger_node = Node(
        package="asv_arov_router",
        executable="data_logger",
        name="data_logger"
    )

    gscam2_node = Node(
        package="gscam2",
        executable="gscam_main",
        name="gscam",
        parameters=[
            {"camera_name": "narrow_stereo"},
            {"camera_info_url": f"file://{os.path.join(get_package_share_directory('bluerov'), 'config', 'ost.yaml')}"},
            {"gscam_config": "udpsrc port=5601 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=RGB ! queue ! videoconvert"}, # Use for video from BlueROV camera
            # {"gscam_config": "v4l2src name=cam_src ! decodebin ! videoconvert ! videoscale ! video/x-raw,format=RGB ! queue ! videoconvert"}, # Use for testing with laptop webcam
            {"frame_id": "bluerov_camera"}
        ],
        remappings=[
            ('/image_raw', '/image_rect')
        ]
    )

    config = os.path.join(
        get_package_share_directory('bluerov'),
        'config',
        'apriltag_node_config.yaml'
    )

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        parameters=[
            config,
            {"image_rect": "/image_rect"},
            {"camera_info": "/camera_info"}
        ]
        # remappings=[
        #     ('/image_rect', ' /image_raw')
        # ]
    )

    video_recorder_node = Node(
        package="asv_arov_router",
        executable="bluerov_video_recorder",
        name="video_recorder"
    )

    # tf2 static transforms
    bluerov_camera_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bluerov_camera_transform",
        arguments=[
            "0.15",
            "0",
            "0",
            "-1.571",
            "-3.141",
            "-1.571",
            "/bluerov",
            "/bluerov_camera"
        ]
    )

    bluerov_odom_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bluerov_odom_transform",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "/bluerov_odom",
            "/bluerov"
        ]
    )

    # End tf2 static transforms


    ld.add_action(bluerov_node)
    ld.add_action(maddy_node)
    ld.add_action(data_logger_node)
    ld.add_action(gscam2_node)
    ld.add_action(apriltag_node)
    ld.add_action(video_recorder_node)

    # tf2 static transforms
    ld.add_action(bluerov_camera_transform_node)
    ld.add_action(bluerov_odom_transform_node)

    return ld