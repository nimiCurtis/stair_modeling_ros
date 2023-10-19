import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
def generate_launch_description():

    # Get parameters from yaml
    config = os.path.join(
        get_package_share_directory('stair_modeling_ros'),
        'params',
        'stair_modeling_params.yaml'
    )

    # Declare the "debug" argument
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Debug mode for stair modeling node'
    )

    # Set launch of stair detector
    launch_stair_modeling = TimerAction(
                period=0.5,
                actions=[
                        Node(
                                package='stair_modeling_ros',
                                executable='stair_modeling_ros_node',
                                output='screen',
                                parameters=[config, {'debug': LaunchConfiguration('debug')}],
                        )
                ]
        )

    # Return launch description
    return LaunchDescription([
        debug_arg,
        launch_stair_modeling
    ])
