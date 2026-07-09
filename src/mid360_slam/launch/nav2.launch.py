import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():
    pkg_mid360 = get_package_share_directory('mid360_slam')
    pkg_fast_lio = get_package_share_directory('fast_lio')
    pkg_livox = get_package_share_directory('livox_ros_driver2')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # 配置文件路径
    nav2_params_path = os.path.join(pkg_mid360, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(pkg_mid360, 'config', 'nav2.rviz')
    map_path = os.path.expanduser('~/mid360_map/navmap.yaml')
    livox_config_path = os.path.join(pkg_livox, 'config', 'MID360s_config.json')
    fast_lio_config_path = os.path.join(pkg_fast_lio, 'config')

    # LiDAR 配置参数
    CMD_LINE_BD_CODE = '201435328'
    HOST_IP = '192.168.0.5'
    LIDAR_IP = '192.168.0.126'
    XFER_FORMAT = 1  # 0-PointCloud2, 1-CustomMsg
    PUBLISH_FREQ = 10.0
    FRAME_ID = 'livox_frame'

    # 启动参数
    use_rviz = LaunchConfiguration('use_rviz')
    use_lidar = LaunchConfiguration('use_lidar')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='是否启动 RViz2'
    )
    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar', default_value='true',
        description='是否启动 LiDAR 驱动'
    )

    # LiDAR 驱动节点
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': XFER_FORMAT,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': PUBLISH_FREQ,
            'output_data_type': 0,
            'frame_id': FRAME_ID,
            'lvx_file_path': '/home/pi/livox_test.lvx',
            'user_config_path': livox_config_path,
            'cmdline_input_bd_code': CMD_LINE_BD_CODE,
        }],
        condition=IfCondition(use_lidar),
    )

    # FAST-LIO 定位节点
    mid360_yaml_path = os.path.join(fast_lio_config_path, 'mid360.yaml')
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio_mapping',
        output='screen',
        parameters=[mid360_yaml_path, {'use_sim_time': False}],
    )

    # Nav2 组件 - 地图服务器
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': map_path,
            'frame_id': 'camera_init',
        }],
    )

    # Nav2 组件 - 行为服务器
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': False}],
    )

    # Nav2 组件 - 控制器服务器
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': False}],
    )

    # Nav2 组件 - 规划器服务器
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': False}],
    )

    # Nav2 组件 - BT导航器
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': False}],
    )

    # Nav2 组件 - 路点跟随器
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': False}],
    )

    # 生命周期管理器
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'map_server',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
        }],
    )

    # TF 坐标转换: odom -> body (由FAST-LIO提供)
    # FAST-LIO 已经发布了 camera_init -> body 的TF

    # RViz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    # 鼠标遥控节点
    mouse_teleop_script = os.path.join(
        os.path.expanduser('~'), 'mid360_slam_ws', 'scripts', 'mouse_teleop.py')
    mouse_teleop_node = Node(
        executable='python3',
        name='mouse_teleop',
        arguments=[mouse_teleop_script],
        output='screen',
    )
    
    # 坐标监控节点
    coord_monitor_script = os.path.join(
        os.path.expanduser('~'), 'mid360_slam_ws', 'scripts', 'coordinate_monitor.py')
    coord_monitor_node = Node(
        executable='python3',
        name='coordinate_monitor',
        arguments=[coord_monitor_script],
        output='screen',
    )

    # 坐标显示页面节点
    coord_display_script = os.path.join(
        os.path.expanduser('~'), 'mid360_slam_ws', 'scripts', 'coordinate_display.py')
    coord_display_node = Node(
        executable='python3',
        name='coordinate_display',
        arguments=[coord_display_script],
        output='screen',
    )

    # 坐标记录节点（保存到桌面txt）
    coord_logger_script = os.path.join(
        os.path.expanduser('~'), 'mid360_slam_ws', 'scripts', 'save_coordinates.py')
    coord_logger_node = Node(
        executable='python3',
        name='coordinate_logger',
        arguments=[coord_logger_script],
        output='screen',
    )

    # Nav2 组件

    ld = LaunchDescription()

    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_lidar)

    # 传感器
    ld.add_action(livox_driver_node)
    ld.add_action(fast_lio_node)

    # Nav2 组件
    ld.add_action(map_server_node)
    ld.add_action(controller_server_node)
    ld.add_action(planner_server_node)
    ld.add_action(behavior_server_node)
    ld.add_action(bt_navigator_node)
    ld.add_action(waypoint_follower_node)
    ld.add_action(lifecycle_manager_node)

    # 可视化和控制
    ld.add_action(rviz_node)
    ld.add_action(mouse_teleop_node)
    ld.add_action(coord_monitor_node)
    # coord_display_node 已移除，需要时通过桌面快捷方式单独启动
    ld.add_action(coord_logger_node)

    # 当rviz2关闭时，关闭整个launch进程（包括坐标监控节点）
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    ))

    return ld
