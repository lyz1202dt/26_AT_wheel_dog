import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包的共享目录
    remote_node_dir = get_package_share_directory('remote_node')
    
    # 配置文件路径
    config_file = os.path.join(
        remote_node_dir,
        'config',
        'remote_config.yaml'
    )
    
    # 声明启动参数
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB1',
        description='Serial port for remote control receiver'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial port baudrate'
    )
    
    # 创建节点
    remote_node = Node(
        package='remote_node',
        executable='remote_node',
        name='remote_node',
        output='screen',
        parameters=[
            {'port': LaunchConfiguration('port')},
            {'baudrate': LaunchConfiguration('baudrate')},
        ],
        remappings=[
            # 可以在这里重新映射话题名称
            ('robot_move_cmd', 'robot_move_cmd'),
        ]
    )
    
    return LaunchDescription([
        port_arg,
        baudrate_arg,
        remote_node,
    ])
