from launch import LaunchDescription
from launch.conditions import UnlessCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os

def generate_launch_description():
    # Pfad zum rs_launch.py-File aus realsense2_camera
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    # Include des rs_launch.py
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'accel_fps': '200',
            'align_depth.enable': 'True',
            'camera_namespace': 'imars_lite',
            'camera_name': 'camera',
            'depth_module.depth_profile': '640x360x30',
            'depth_module.emitter_enable': 'True',
            'enable_gyro': 'True',
            'enable_accel': 'True',
            'enable_sync': 'True',
            'initial_reset': 'True',
            'pointcloud.enable': 'True',
            'rgb_camera.color_profile': '640x360x30',
            'spatial_filter.enable': 'False',
            'unite_imu_method': '2',
        }.items()
    )

    pkg_share = launch_ros.substitutions.FindPackageShare(package='imars_bringup').find('imars_bringup')

    # Robot und Joint State Nodes
    default_model_path = os.path.join(pkg_share, 'urdf/imars_lite.urdf')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    # Odometry Node von RTAB-Map ist meiner Meinung nach auch ein Sensor
    parameters=[{
        'frame_id':'base_link',
        'subscribe_depth':True,
        'subscribe_rgb':True,
        'subscribe_imu':True,
        'subscribe_odom_info':True,
        'approx_sync':False,
        'wait_imu_to_init':True,
        'publish_tf': True,
        'Reg/Force3DoF': 'true',
    }]

    remappings=[
        ('imu', '/imars_lite/imu/data'),
        ('rgb/image', '/imars_lite/camera/color/image_raw'),
        ('rgb/camera_info', '/imars_lite/camera/color/camera_info'),
        ('depth/image', '/imars_lite/camera/aligned_depth_to_color/image_raw'),
        #('odom', '/imars_lite/odom/rtabmap')
    ]
    
    # Visual Odometry Node
    visual_odometry_node = launch_ros.actions.Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=parameters,
        remappings=remappings
    )

    # Imu Madgwickfilter node
    imu_filter_node = launch_ros.actions.Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        parameters=[{'use_mag': False, 
                     'world_frame':'enu', 
                     'publish_tf':False}],
        remappings=[('imu/data_raw', '/imars_lite/camera/imu'),
                    ('imu/data', '/imars_lite/imu/data')
                   ]
    )

    # Vielleicht noch nodes für Radencoder, Ultraschall und LiDAR-Sensoren erstellen

    twist_to_ackermann_node = launch_ros.actions.Node(
        package='imars_bringup',
        executable='TwistToAckermannNode',
    )


    serial_interface_node = launch_ros.actions.Node(
        package='imars_bringup',
        executable='SerialInterfaceNode',
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='False', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        realsense_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        visual_odometry_node,
        imu_filter_node,
        #twist_to_ackermann_node,
        #serial_interface_node,
    ])
