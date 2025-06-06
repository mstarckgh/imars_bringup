from launch import LaunchDescription
from launch.conditions import UnlessCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os

def generate_launch_description():
    # SLAM RTAB-Map

    parameters=[{
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': False,  # Bei synchronisierten Topics False setzen
        'wait_imu_to_init': True,
    }]

    remappings=[
        ('imu', '/imars_lite/imu/data'),
        ('rgb/image', '/imars_lite/camera/color/image_raw'),
        ('rgb/camera_info', '/imars_lite/camera/color/camera_info'),
        ('depth/image', '/imars_lite/camera/aligned_depth_to_color/image_raw')]
    
    rtabmap_slam_node = launch_ros.actions.Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=parameters,
        remappings=remappings,
    )

    return LaunchDescription([
        rtabmap_slam_node,
    ])
