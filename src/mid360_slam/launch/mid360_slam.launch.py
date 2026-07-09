import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import launch

# ============ 用户配置参数 ============
# LiDAR 广播码 (必须与实际设备一致)
CMD_LINE_BD_CODE = '201435328'

# 网络配置
HOST_IP = '192.168.0.108'
LIDAR_IP = '192.168.0.126'

# 数据格式: 0-PointCloud2(PointXYZRTL), 1-自定义格式
# 必须与FAST_LIO的lidar_type匹配: lidar_type=1时用XFER_FORMAT=1
XFER_FORMAT = 1

# 发布频率 (Hz)
PUBLISH_FREQ = 10.0

# 坐标系
FRAME_ID = 'livox_frame'

# 地图保存路径
MAP_FILE_PATH = os.path.expanduser('~/mid360_map/test.pcd')
# ======================================

def generate_launch_description():
    pkg_livox = get_package_share_directory('livox_ros_driver2')
    pkg_fast_lio = get_package_share_directory('fast_lio')
    pkg_mid360 = get_package_share_directory('mid360_slam')

    # LiDAR 配置
    livox_config_path = os.path.join(pkg_livox, 'config', 'MID360s_config.json')
    fast_lio_config_path = os.path.join(pkg_fast_lio, 'config')
    rviz_config_path = os.path.join(pkg_fast_lio, 'rviz_cfg', 'fastlio.rviz')

    # 启动参数
    use_rviz = LaunchConfiguration('use_rviz')
    map_save = LaunchConfiguration('map_save')
    use_pose_printer = LaunchConfiguration('use_pose_printer')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='是否启动 RViz 可视化'
    )
    declare_map_save = DeclareLaunchArgument(
        'map_save', default_value='true',
        description='是否保存 PCD 地图'
    )
    declare_use_pose_printer = DeclareLaunchArgument(
        'use_pose_printer', default_value='true',
        description='是否启动实时位姿输出'
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
        }]
    )

    # FAST_LIO 建图节点
    mid360_yaml_path = os.path.join(fast_lio_config_path, 'mid360.yaml')
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio_mapping',
        output='screen',
        parameters=[mid360_yaml_path,
                    {'use_sim_time': False}],
    )

    # 静态 TF：camera_init -> body（确保 fixed frame 始终已知）
    static_tf1_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_init_to_body',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'body'],
    )

    # 静态 TF：body -> livox_frame（使用 FAST-LIO 外参标定值，连接激光雷达坐标系）
    static_tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_body_to_livox_frame',
        arguments=['-0.011', '-0.02329', '0.04412', '0', '0', '0', 'body', 'livox_frame'],
    )

    # RViz 可视化节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    # 位姿输出节点
    pose_printer_script = os.path.join(
        os.path.expanduser('~'), 'mid360_slam_ws', 'scripts', 'print_pose.py')
    pose_printer_node = ExecuteProcess(
        cmd=['python3', pose_printer_script],
        output='screen',
        condition=IfCondition(use_pose_printer),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_map_save)
    ld.add_action(declare_use_pose_printer)
    ld.add_action(static_tf1_node)
    ld.add_action(static_tf2_node)
    ld.add_action(livox_driver_node)
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)
    ld.add_action(pose_printer_node)

    return ld
