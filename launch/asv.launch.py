from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import yaml
from scipy.spatial.transform import Rotation

def generate_launch_description():
    ld = LaunchDescription()

    AROV_NAME = 'arov'
    ASV_NAME = 'asv'

    bluerov_node = Node(
        package="asv_arov_router",
        executable="mavlink_router",
        name="arov_connection",
        namespace=f'{AROV_NAME}',
        parameters=[
            {'device': 'udpin:localhost:14552'},
            {'vehicle_name': AROV_NAME},
            {'rc_override_mapping': [4, 5, 2, 6, 7, 3]},
            {'max_cmd_vel_linear': 0.33},
            {'max_cmd_vel_angular': 3.1415/2},
            {'translation_limit': 500},
            {'rotation_limit': 500}
        ]
    )

    maddy_node = Node(
        package="asv_arov_router",
        executable="mavlink_router",
        name="asv_connection",
        namespace=f'{ASV_NAME}',
        parameters=[
            {'device': 'udpin:localhost:14555'},
            {'vehicle_name': ASV_NAME},
            {'rc_override_mapping': [0, 2, 3, 4, 5, 1]},
            {'has_camera': False},
            {'max_cmd_vel_linear': 1.0},
            {'max_cmd_vel_angular': 1.2},
            {'translation_limit': 500},
            {'rotation_limit': 500}
        ]
    )

    data_logger_node = Node(
        package="asv_arov_router",
        executable="data_logger",
        name="data_logger",
        parameters=[
            {'arov_name': AROV_NAME},
            {'asv_name': ASV_NAME}
        ]
    )

    arov_ekf_global_node = Node(
        package="asv_arov_localization",
        executable="arov_ekf_global",
        name="arov_ekf_global",
        namespace=f'{AROV_NAME}',
        parameters=[
            {'~vehicle_name': AROV_NAME},
            {'~ros_bag': False}
        ]
    )

    # AROV Camera
    external_cam_apriltag_config = os.path.join(
        get_package_share_directory('asv_arov_router'),
        'config',
        'apriltag_node_config.yaml'
    )

    arov_cam_apriltag_config = os.path.join(
        get_package_share_directory('asv_arov_router'),
        'config',
        'apriltag_node_config.yaml'
    )

    arov_gscam2_node = Node(
        package="gscam2",
        executable="gscam_main",
        name="gscam",
        namespace=f'{AROV_NAME}',
        parameters=[
            {"camera_name": "narrow_stereo"},
            {"camera_info_url": f"file://{os.path.join(get_package_share_directory('asv_arov_router'), 'config', 'barlus_stevens_water_calibration.yaml')}"},
            {"gscam_config": "rtspsrc location=\"rtsp://admin:@169.254.34.12/h264_stream\" latency=0 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=RGB ! queue ! videoconvert"}, # Use for video from Barlus camera
            # {"gscam_config": "v4l2src name=cam_src ! decodebin ! videoconvert ! videoscale ! video/x-raw,format=RGB ! queue ! videoconvert"}, # Use for testing with laptop webcam
            {"frame_id": f"/{AROV_NAME}/camera"}
        ],
        remappings=[
            (f'/{AROV_NAME}/image_raw', f'/{AROV_NAME}/image_rect')
        ]
    )

    arov_apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        namespace=f'{AROV_NAME}',
        parameters=[
            external_cam_apriltag_config
        ],
        remappings=[
            ('/apriltag/image_rect',f'/{AROV_NAME}/image_rect'),
            ('/camera/camera_info',f'/{AROV_NAME}/camera_info')
        ]
    )

    # External Cameras
    cam1_gscam2_node = Node(
        package="gscam2",
        executable="gscam_main",
        name="gscam",
        namespace='cam1',
        parameters=[
            {"camera_name": "narrow_stereo"},
            {"camera_info_url": f"file://{os.path.join(get_package_share_directory('asv_arov_router'), 'config', 'barlus_stevens_water_calibration.yaml')}"},
            {"gscam_config": "rtspsrc location=\"rtsp://admin:@169.254.34.13/h264_stream\" latency=0 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=RGB ! queue ! videoconvert"}, # Use for video from Barlus camera
            # For testing: gst-launch-1.0 rtspsrc location="rtsp://admin:@169.254.209.11/h264_stream" latency=0 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! autovideosink
            # {"gscam_config": "v4l2src name=cam_src ! decodebin ! videoconvert ! videoscale ! video/x-raw,format=RGB ! queue ! videoconvert"}, # Use for testing with laptop webcam # ! application/x-rtp, payload=96
            {"frame_id": '/cam1/camera'}
        ],
        remappings=[
            ('/cam1/image_raw', '/cam1/image_rect')
        ]
    )

    cam1_apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        namespace=f'cam1',
        parameters=[
            external_cam_apriltag_config
        ],
        remappings=[
            ('/apriltag/image_rect','/cam1/image_rect'),
            ('/camera/camera_info','/cam1/camera_info')
        ]
    )

    cam2_gscam2_node = Node(
        package="gscam2",
        executable="gscam_main",
        name="gscam",
        namespace='cam2',
        parameters=[
            {"camera_name": "narrow_stereo"},
            {"camera_info_url": f"file://{os.path.join(get_package_share_directory('asv_arov_router'), 'config', 'barlus_stevens_water_calibration.yaml')}"},
            {"gscam_config": "rtspsrc location=\"rtsp://admin:@169.254.34.11/h264_stream\" latency=0 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=RGB ! queue ! videoconvert"}, # Use for video from Barlus camera
            # For testing: gst-launch-1.0 rtspsrc location="rtsp://admin:@169.254.209.11/h264_stream" latency=0 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! autovideosink
            # {"gscam_config": "v4l2src name=cam_src ! decodebin ! videoconvert ! videoscale ! video/x-raw,format=RGB ! queue ! videoconvert"}, # Use for testing with laptop webcam # ! application/x-rtp, payload=96
            {"frame_id": '/cam2/camera'}
        ],
        remappings=[
            ('/cam2/image_raw', '/cam2/image_rect')
        ]
    )

    cam2_apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        namespace=f'cam2',
        parameters=[
            external_cam_apriltag_config
        ],
        remappings=[
            ('/apriltag/image_rect','/cam2/image_rect'),
            ('/camera/camera_info','/cam2/camera_info')
        ]
    )

    cam3_gscam2_node = Node(
        package="gscam2",
        executable="gscam_main",
        name="gscam",
        namespace='cam3',
        parameters=[
            {"camera_name": "narrow_stereo"},
            {"camera_info_url": f"file://{os.path.join(get_package_share_directory('asv_arov_router'), 'config', 'barlus_stevens_water_calibration.yaml')}"},
            {"gscam_config": "rtspsrc location=\"rtsp://admin:@169.254.34.14/h264_stream\" latency=0 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=RGB ! queue ! videoconvert"}, # Use for video from Barlus camera
            # For testing: gst-launch-1.0 rtspsrc location="rtsp://admin:@169.254.209.11/h264_stream" latency=0 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! autovideosink
            # {"gscam_config": "v4l2src name=cam_src ! decodebin ! videoconvert ! videoscale ! video/x-raw,format=RGB ! queue ! videoconvert"}, # Use for testing with laptop webcam # ! application/x-rtp, payload=96
            {"frame_id": '/cam3/camera'}
        ],
        remappings=[
            ('/cam3/image_raw', '/cam3/image_rect')
        ]
    )

    cam3_apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        namespace=f'cam3',
        parameters=[
            external_cam_apriltag_config
        ],
        remappings=[
            ('/apriltag/image_rect','/cam3/image_rect'),
            ('/camera/camera_info','/cam3/camera_info')
        ]
    )

    cam4_gscam2_node = Node(
        package="gscam2",
        executable="gscam_main",
        name="gscam",
        namespace='cam4',
        parameters=[
            {"camera_name": "narrow_stereo"},
            {"camera_info_url": f"file://{os.path.join(get_package_share_directory('asv_arov_router'), 'config', 'barlus_stevens_water_calibration.yaml')}"},
            {"gscam_config": "rtspsrc location=\"rtsp://admin:@169.254.34.12/h264_stream\" latency=0 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=RGB ! queue ! videoconvert"}, # Use for video from Barlus camera
            # For testing: gst-launch-1.0 rtspsrc location="rtsp://admin:@169.254.209.11/h264_stream" latency=0 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! autovideosink
            # {"gscam_config": "v4l2src name=cam_src ! decodebin ! videoconvert ! videoscale ! video/x-raw,format=RGB ! queue ! videoconvert"}, # Use for testing with laptop webcam # ! application/x-rtp, payload=96
            {"frame_id": '/cam4/camera'}
        ],
        remappings=[
            ('/cam4/image_raw', '/cam4/image_rect')
        ]
    )

    cam4_apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        namespace=f'cam4',
        parameters=[
            external_cam_apriltag_config
        ],
        remappings=[
            ('/apriltag/image_rect','/cam4/image_rect'),
            ('/camera/camera_info','/cam4/camera_info')
        ]
    )

    # End Cameras

    arov_ekf_external_node = Node(
        package="asv_arov_localization",
        executable="arov_ekf_external",
        name="arov_ekf_external",
        namespace=f'{AROV_NAME}',
        parameters=[
            {'~vehicle_name': AROV_NAME},
            {'~ros_bag': False},
            {'~use_gyro': False}
        ]
    )

    video_recorder_node = Node(
        package="asv_arov_router",
        executable="bluerov_video_recorder",
        name="video_recorder",
        namespace='cam1'
    )

    bno055_publisher_node = Node(
        package="asv_arov_router",
        executable="bno055_publisher",
        name="bno055_publisher",
        namespace=f'{AROV_NAME}',
    )

    # tf2 static transforms
    # bluerov_camera_transform_node = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="bluerov_camera_transform",
    #     arguments=[
    #         '--x', '0.15',
    #         '--y', '0.0',
    #         '--z', '0.0',
    #         '--yaw', '-1.571',
    #         '--pitch', '0.0',
    #         '--roll', '-1.571',
    #         '--frame-id', f"/{AROV_NAME}/base_link",
    #         '--child-frame-id', f"/{AROV_NAME}_camera"
    #     ]
    # )

    maddy_odom_map_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="asv_odom_map_transform",
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', "/map",
            '--child-frame-id', f"/{ASV_NAME}/odom"
        ]
    )

    arov_odom_map_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="arov_odom_map_transform",
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', "/map",
            '--child-frame-id', f"/{AROV_NAME}/odom"
        ]
    )

    arov_base_link_odom_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="arov_base_link_odom_transform",
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', f"/{AROV_NAME}/odom",
            '--child-frame-id', f"/{AROV_NAME}/base_link"
        ]
    )

    apriltags = os.path.join(
        get_package_share_directory('asv_arov_router'),
        'config',
        'apriltag_layout_config.yaml'
    )

    with open(apriltags) as stream :
        try :
            apriltags_layout = yaml.safe_load(stream)
            static_rot = Rotation.from_euler('xyz', [apriltags_layout['static_offset']['roll'],
                                                     apriltags_layout['static_offset']['pitch'],
                                                     apriltags_layout['static_offset']['yaw']])
            static_translation = [apriltags_layout['static_offset']['x'],
                                  apriltags_layout['static_offset']['y'],
                                  apriltags_layout['static_offset']['z']]

            for family in apriltags_layout['apriltags'] :
                for tag in family['tags'] :
                    tag_rot = (Rotation.from_euler('xyz', [tag['roll'], tag['pitch'], tag['yaw']]) * static_rot).as_euler('xyz')
                    
                    if tag['frame_id'] == '/map' :
                        tag_x = f'{tag['x'] + static_translation[0]}'
                        tag_y = f'{tag['y'] + static_translation[1]}'
                        tag_z = f'{tag['z'] + static_translation[2]}'
                    else :
                        tag_x = f'{tag['x']}'
                        tag_y = f'{tag['y']}'
                        tag_z = f'{tag['z']}'

                    ld.add_action(Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name=f"tag_{family['family']}_{tag['id']}_transform",
                        arguments=[
                            '--x', f'{tag_x}',
                            '--y', f'{tag_y}',
                            '--z', f'{tag_z}',
                            '--yaw', f'{tag_rot[2]}',
                            '--pitch', f'{tag_rot[1]}',
                            '--roll', f'{tag_rot[0]}',
                            '--frame-id', tag['frame_id'],
                            '--child-frame-id', f"/tag{family['family']}:{tag['id']}_true"
                        ]
                    ))

        except yaml.YAMLError as exc:
            print(exc)

    cameras = os.path.join(
        get_package_share_directory('asv_arov_router'),
        'config',
        'external_camera_layout.yaml'
    )

    with open(cameras) as stream :
        try :
            camera_layout = yaml.safe_load(stream)
            static_rot = Rotation.from_euler('xyz', [camera_layout['static_offset']['roll'],
                                                     camera_layout['static_offset']['pitch'],
                                                     camera_layout['static_offset']['yaw']])
        
            static_translation = [camera_layout['static_offset']['x'],
                                  camera_layout['static_offset']['y'],
                                  camera_layout['static_offset']['z']]
            
            optitrack_rot = Rotation.from_euler('xyz', [camera_layout['optitrack_rot']['roll'],
                                                        camera_layout['optitrack_rot']['pitch'],
                                                        camera_layout['optitrack_rot']['yaw']])

            for camera in camera_layout['cameras'] :
                cam_rot = (Rotation.from_euler('xyz', [camera['roll'], camera['pitch'], camera['yaw']]) * static_rot).as_euler('xyz')
                
                cam_offset = optitrack_rot.apply([camera['x'], camera['y'], camera['z']])

                ld.add_action(Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"tag_{family['family']}_{tag['id']}_transform",
                    arguments=[
                        '--x', str(cam_offset[0] + static_translation[0]),
                        '--y', str(cam_offset[1] + static_translation[1]),
                        '--z', str(cam_offset[2] + static_translation[2]),
                        '--yaw', f'{cam_rot[2]}',
                        '--pitch', f'{cam_rot[1]}',
                        '--roll', f'{cam_rot[0]}',
                        '--frame-id', camera['frame_id'],
                        '--child-frame-id', f'{camera['namespace']}/camera'
                    ]
                ))

        except yaml.YAMLError as exc:
            print(exc)

    # End tf2 static transforms

    ld.add_action(bluerov_node)
    ld.add_action(maddy_node)

    ld.add_action(arov_gscam2_node)
    ld.add_action(arov_apriltag_node)

    # ld.add_action(cam1_gscam2_node)
    # ld.add_action(cam1_apriltag_node)

    # ld.add_action(cam2_gscam2_node)
    # ld.add_action(cam2_apriltag_node)

    # ld.add_action(cam3_gscam2_node)
    # ld.add_action(cam3_apriltag_node)

    # ld.add_action(cam4_gscam2_node)
    # ld.add_action(cam4_apriltag_node)

    # ld.add_action(arov_ekf_global_node)
    ld.add_action(arov_ekf_external_node)
    ld.add_action(video_recorder_node)
    # ld.add_action(bno055_publisher_node)

    # tf2 static transforms
    # ld.add_action(bluerov_camera_transform_node)
    ld.add_action(maddy_odom_map_transform_node)
    # ld.add_action(arov_odom_map_transform_node)
    # ld.add_action(arov_base_link_odom_transform_node)

    return ld
