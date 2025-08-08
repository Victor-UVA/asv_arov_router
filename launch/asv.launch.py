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
            {'rc_override_mapping': [4, 5, 2, 6, 7, 3]}
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
            {'max_cmd_vel_linear': 0.33},
            {'max_cmd_vel_angular': 3.1415/2},
            {'translation_limit': 500},
            {'rotation_limit': 250}
        ]
    )

    # Camera set up
    camera_setup = os.path.join(
        get_package_share_directory('asv_arov_router'),
        'config',
        'camera_configs.yaml'
    )

    cam_apriltag_config = os.path.join(
        get_package_share_directory('asv_arov_router'),
        'config',
        'apriltag_node_config.yaml'
    )

    camera_list = []
    display_list = []
    frame_rates = []

    with open(camera_setup) as stream :
        try :
            camera_setups = yaml.safe_load(stream)

            for camera in camera_setups['cameras'] :
                ld.add_action(Node(
                    package="gscam2",
                    executable="gscam_main",
                    name="gscam",
                    namespace=camera['namespace'],
                    parameters=[
                        {"camera_name": "narrow_stereo"},
                        {"camera_info_url": f"file://{os.path.join(get_package_share_directory('asv_arov_router'), 'config', camera['calibration'])}"},
                        {"gscam_config": camera['gscam']},
                        {"frame_id": f"{camera['namespace']}/camera"}
                    ],
                    remappings=[
                        (f'{camera['namespace']}/image_raw', f'{camera['namespace']}/image_rect')
                    ]
                ))

                camera_list.append(camera['namespace'])
                display_list.append(camera['display'])
                frame_rates.append(camera['rate'])

                if camera['tags'] :
                    ld.add_action(Node(
                        package="apriltag_ros",
                        executable="apriltag_node",
                        name="apriltag",
                        namespace=camera['namespace'],
                        parameters=[
                            cam_apriltag_config
                        ],
                        remappings=[
                            ('/apriltag/image_rect',f'{camera['namespace']}/image_rect'),
                            ('/camera/camera_info',f'{camera['namespace']}/camera_info')
                        ]
                    ))

        except yaml.YAMLError as exc:
            print(exc)

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
        parameters=[
            {'camera_list': camera_list},
            {'display_list': display_list},
            {'frame_rates': frame_rates},
            {'display_size': [1920, 1080]},
            {'display_layout': [1, 1]}
        ]
    )

    bno055_publisher_node = Node(
        package="asv_arov_router",
        executable="bno055_publisher",
        name="bno055_publisher",
        namespace=f'{AROV_NAME}',
    )

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