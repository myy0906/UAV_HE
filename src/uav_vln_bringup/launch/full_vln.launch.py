import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
)

from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import Node


def generate_launch_description():

    share = (

        get_package_share_directory(

            'uav_vln_bringup'
        )
    )

    bridge = os.path.join(

        share,

        'config',

        'gz_bridge.yaml',
    )

    known_map = os.path.join(

        share,

        'config',

        'known_map.json',
    )

    scene = os.path.join(

        share,

        'models',

        'vln_scene',

        'model.sdf',
    )

    parser_mode = LaunchConfiguration(

        'parser_mode'
    )

    base_url = LaunchConfiguration(

        'base_url'
    )

    model = LaunchConfiguration(

        'model'
    )

    spawn_scene = Node(

        package='ros_gz_sim',

        executable='create',

        name='spawn_vln_scene',

        arguments=[

            '-world',
            'default',

            '-file',
            scene,

            '-name',
            'vln_scene',

            '-x',
            '0.0',

            '-y',
            '0.0',

            '-z',
            '0.0',
        ],

        output='screen',
    )

    return LaunchDescription([

        DeclareLaunchArgument(

            'parser_mode',

            default_value='llm',
        ),

        DeclareLaunchArgument(

            'base_url',

            default_value=(
                'http://192.168.189.1:1234/v1'
            ),
        ),

        DeclareLaunchArgument(

            'model',

            default_value=(
                'qwen/qwen3-vl-8b'
            ),
        ),

        # Gazebo RGB-D -> ROS2
        Node(

            package='ros_gz_bridge',

            executable='parameter_bridge',

            name='uav_gz_bridge',

            parameters=[{

                'config_file':
                    bridge,
            }],

            output='screen',
        ),

        # Original PX4 execution layer
        Node(

            package='uav_vln_px4',

            executable='px4_executor',

            name='px4_executor',

            output='screen',
        ),

        # Original HSV perception
        Node(

            package='uav_vln_perception',

            executable='red_target_detector',

            name='red_target_detector',

            output='screen',
        ),

        # Original RGB-D target mapping
        Node(

            package='uav_vln_mapping',

            executable='target_mapper',

            name='target_mapper',

            output='screen',
        ),

        # Known static obstacles
        Node(

            package='uav_vln_mapping',

            executable='known_map',

            name='known_map',

            parameters=[{

                'config_file':
                    known_map,

                'publish_rate':
                    2.0,
            }],

            output='screen',
        ),

        # Final mission manager:
        # JSON + LLM + safety compiler + A*
        Node(

            package='uav_vln_mission',

            executable='mission_manager',

            name='mission_manager',

            parameters=[{

                'config_file':
                    known_map,

                'parser_mode':
                    parser_mode,

                'base_url':
                    base_url,

                'model':
                    model,

                'llm_timeout_s':
                    90.0,

                'target_timeout_s':
                    30.0,

                'target_freshness_s':
                    1.5,
            }],

            output='screen',
        ),

        TimerAction(

            period=2.0,

            actions=[

                spawn_scene,
            ],
        ),
    ])
