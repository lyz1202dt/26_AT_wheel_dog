from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():

    simulate_env_launch_scripe="dog_mujoco_sim.py"

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
            "joint_kp": [50.0, 50.0, 50.0],
            "joint_kd": [3.0, 3.0, 3.0],
            "wheel_kd": 0.5,
            "roll_vmc_kp": -300.0,
            "roll_vmc_kd": -5.0,
            "pitch_vmc_kp": 400.0,
            "pitch_vmc_kd": 5.0,
            "lf_grivate": 22.0,
            "rf_grivate": 22.0,
            "lb_grivate": 28.0,
            "rb_grivate": 28.0,
            "driver_or_sim": False
        }
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
    remote_node = Node(
        package="remote_node",     
        executable="remote_node",  
        name="remote_node",
        output="screen"      # 输出到屏幕
    )
    
    sim_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('launch_pack'), 'launch', simulate_env_launch_scripe)]))
    
    return LaunchDescription([robot_state_pub,  leg_calc , rviz2 ,sim_launch,remote_node])
