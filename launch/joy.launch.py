from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_params = os.path.join(get_package_share_directory('imars_bringup'), 'config', 'joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
    )

    joy_to_cmd_vel_node = Node(
        package='imars_bringup',
        executable='JoyToCmdVelNode',
        name='jtc_node',
        parameters=[joy_params],
    )

    return LaunchDescription([
        joy_node,
        joy_to_cmd_vel_node
    ])