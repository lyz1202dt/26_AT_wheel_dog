from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("dog2"),
        "model", "dog2.urdf"
    )
    # 读取URDF内容
    with open(urdf_path, 'r') as inf:
        robot_desc = inf.read()

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}]
    )

    leg_calc = Node(
        package="move_control",
        executable="move_control",
        parameters=[
        {
            "joint_kp": [3.0, 2.8, 2.8],
            "joint_kd": [0.17, 0.14, 0.11],
            "wheel_kd": 0.5,
            "roll_vmc_kp": -600.0,
            "roll_vmc_kd": -50.0,
            "pitch_vmc_kp": 550.0,
            "pitch_vmc_kd": 50.0,
            "lf_grivate": 32.0,
            "rf_grivate": 32.0,
            "lb_grivate": 40.0,
            "rb_grivate": 40.0,
            "driver_or_sim": True
        }
    ]
    )

    # leg_driver节点 - 禁用IMU数据发布，因为我们使用独立的imu_driver
    leg_driver = Node(
        package="leg_driver",
        executable="leg_driver",
        parameters=[
            {"publish_imu": False}  # 禁用leg_driver的IMU数据发布
        ]
    )

    # imu_driver节点 - 使用独立的IMU设备通过串口提供IMU数据
    imu_driver = Node(
        package="imu_driver",
        executable="imu_driver"
    )

    rviz2_config_path = os.path.join(
        get_package_share_directory("launch_pack"),
        "rviz", "display_config.rviz"
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz2_config_path]  # 可选，指定rviz配置文件
    )

    remote_node = Node(
        package="remote_node",
        executable="remote_node",
        name="remote_node",
        output="screen"
    )
    
    
    return LaunchDescription([leg_driver, imu_driver, robot_state_pub, leg_calc, rviz2, remote_node])
