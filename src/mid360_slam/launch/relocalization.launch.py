import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # 启动参数
    use_rviz = LaunchConfiguration('use_rviz')
    map_path = LaunchConfiguration('map_path')
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='是否启动 RViz 可视化'
    )
    
    declare_map_path = DeclareLaunchArgument(
        'map_path', default_value='/home/pi/mid360_map/map.pcd',
        description='PCD地图文件路径'
    )
    
    # 重定位节点
    relocalization_node = Node(
        executable='/home/pi/mid360_slam_ws/scripts/relocalization.py',
        name='relocalization',
        output='screen',
        parameters=[{
            'map_path': map_path,
            'map_origin_x': -38.778350830078125,
            'map_origin_y': -12.73618221282959,
            'map_origin_z': 0.0,
            'icp_max_iterations': 50,
            'icp_threshold': 0.05,
            'icp_max_distance': 1.0,
            'downsample_size': 0.1,
        }]
    )
    
    # RViz 可视化节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/home/pi/mid360_slam_ws/src/mid360_slam/rviz/relocalization.rviz'],
        condition=IfCondition(use_rviz),
        output='screen',
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_map_path)
    ld.add_action(relocalization_node)
    ld.add_action(rviz_node)
    
    return ld
