from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
                'subscribe_stereo': False,
                'subscribe_rgbd': False,
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
                'Rtabmap/StartNewMap': 'false',
                'Rtabmap/PublishStats': 'true',
                'Rtabmap/PublishLastSignature': 'true',
                'Rtabmap/DetectionRate': '1.0',
                'RGBD/NeighborLinkRefining': 'false',
                'RGBD/OptimizeFromGraphEnd': 'false',
                'RGBD/ProximityBySpace': 'false',
                'Vis/MinInliers': '15',
                'database_path': '/home/matthias/.ros/rtabmap.db'
            }],
            remappings=[
                ('imu', '/imars_lite/imu/data'),
                ('rgb/image', '/imars_lite/camera/color/image_raw'),
                ('rgb/camera_info', '/imars_lite/camera/color/camera_info'),
                ('depth/image', '/imars_lite/camera/aligned_depth_to_color/image_raw')
            ]
        )
    ])