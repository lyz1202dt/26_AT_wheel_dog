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

    leg_driver = Node(
        package="leg_driver",
        executable="leg_driver",
        parameters=[
            {"publish_imu": True}
        ]
    )

    rviz2_config_path=os.path.join(
        get_package_share_directory("launch_pack"),
        "rviz", "display_config.rviz"
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz2_config_path]  # 可选，指定rviz配置文件
    )
    return LaunchDescription([leg_driver,robot_state_pub, leg_calc, rviz2])
