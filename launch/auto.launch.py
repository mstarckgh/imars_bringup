from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_name = 'imars_bringup'
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)

    sensor_launch = os.path.join(pkg_share, 'launch', 'sensor.launch.py')
    mapping_launch = os.path.join(pkg_share, 'launch', 'mapping.launch.py')
    localize_launch = os.path.join(pkg_share, 'launch', 'localize.launch.py')
    navigation_launch = os.path.join(pkg_share, 'launch', 'navigation.launch.py')
    joy_launch = os.path.join(pkg_share, 'launch', 'joy.launch.py')

    twist_to_ackermann_node = Node(
        package='imars_bringup',
        executable='TwistToAckermannNode',
    )


    serial_interface_node = Node(
        package='imars_bringup',
        executable='SerialInterfaceNode',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensor_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localize_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch)
        ),
        twist_to_ackermann_node,
        serial_interface_node,
    ])
