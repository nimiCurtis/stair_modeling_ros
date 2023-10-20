import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
def generate_launch_description():

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'
    
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
    stair_modeling_node = Node(
                                package='stair_modeling_ros',
                                executable='stair_modeling_ros',
                                output='screen',
                                parameters=[config,
                                            {'debug': LaunchConfiguration('debug')}],
                        )


    # Return launch description
    return LaunchDescription([
        debug_arg,
        stair_modeling_node
    ])
