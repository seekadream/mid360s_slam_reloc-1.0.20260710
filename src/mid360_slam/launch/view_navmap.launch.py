import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    map_yaml = os.path.expanduser('~/mid360_map/navmap.yaml')
    rviz_config = os.path.expanduser('~/mid360_slam_ws/src/mid360_slam/config/navmap.rviz')
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'topic_name': 'map',
            'frame_id': 'camera_init',
        }]
    )
    
    # 使用ExecuteProcess来配置lifecycle
    configure_cmd = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
        output='screen',
    )
    
    activate_cmd = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
        output='screen',
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )
    
    return LaunchDescription([
        map_server_node,
        TimerAction(
            period=3.0,
            actions=[configure_cmd],
        ),
        TimerAction(
            period=6.0,
            actions=[activate_cmd],
        ),
        TimerAction(
            period=8.0,
            actions=[rviz_node],
        ),
    ])