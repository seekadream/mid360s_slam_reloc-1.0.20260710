import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    package_share_directory = get_package_share_directory('sfast_lio')
    
    config_file = os.path.join(package_share_directory, 'config', 'velodyne_re.yaml')
    rviz_config_file = os.path.join(package_share_directory, 'rviz_cfg', 'relocalization_velo.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='velodyne_to_camera_init',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', '--frame-id', 'camera_init', '--child-frame-id', 'velodyne']
        ),
        
        Node(
            package='sfast_lio',
            executable='fastlio_mapping_relocalization',
            name='laserMapping',
            output='screen',
            parameters=[
                config_file,
                {
                    'feature_extract_enable': False,
                    'point_filter_num': 2,
                    'max_iteration': 3,
                    'filter_size_surf': 0.5,
                    'filter_size_map': 0.5,
                    'cube_side_length': 1000.0,
                }
            ]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])
