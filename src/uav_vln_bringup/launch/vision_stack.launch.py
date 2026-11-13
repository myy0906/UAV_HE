import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    bringup_share = (
        get_package_share_directory(
            'uav_vln_bringup'
        )
    )

    bridge_config = os.path.join(
        bringup_share,
        'config',
        'gz_bridge.yaml',
    )

    return LaunchDescription([

        # Gazebo sensor bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='uav_gz_bridge',
            parameters=[
                {
                    'config_file':
                        bridge_config
                }
            ],
            output='screen',
        ),

        # PX4 execution layer
        Node(
            package='uav_vln_px4',
            executable='px4_executor',
            name='px4_executor',
            output='screen',
        ),

        # Red target perception
        Node(
            package='uav_vln_perception',
            executable='red_target_detector',
            name='red_target_detector',
            output='screen',
        ),

        # RGB-D target mapping
        Node(
            package='uav_vln_mapping',
            executable='target_mapper',
            name='target_mapper',
            output='screen',
        ),

    ])
