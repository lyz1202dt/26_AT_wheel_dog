#pragma once

#include "fsm/fsm.hpp"
#include "leg/leg_calc.hpp"
#include "leg/vmc.hpp"

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <ctime>
#include <tuple>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <robot_interfaces/msg/robot.hpp>
#include <robot_interfaces/msg/move_cmd.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>

class RobotFSM : public FSM{
public:
    RobotFSM(const std::shared_ptr<rclcpp::Node> node);
    ~RobotFSM();
private:

    //ROS2通信相关话题
    rclcpp::Node::SharedPtr node_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;
    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr legs_target_pub;
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr legs_state_sub;
    rclcpp::Subscription<robot_interfaces::msg::MoveCmd>::SharedPtr move_cmd_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_angular_vel_sub;
    rclcpp::SyncParametersClient::SharedPtr robot_description_param_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> robot_tf_broadcaster;

    //可视化相关
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_publisher;
    std::vector<std::string> joint_names = {"lf_joint1", "lf_joint2", "lf_joint3", "rf_joint1", "rf_joint2", "rf_joint3",
                                            "lb_joint1", "lb_joint2", "lb_joint3", "rb_joint1", "rb_joint2", "rb_joint3"};

    //机器人位姿信息
    tf2::Quaternion robot_rotation;                    //机器人姿态
    geometry_msgs::msg::Twist robot_velocity;          //机器人速度信息


    //解算部分
    KDL::Tree tree;
    std::string urdf_xml;
    KDL::Chain lf_leg_chain;
    KDL::Chain rf_leg_chain;
    KDL::Chain lb_leg_chain;
    KDL::Chain rb_leg_chain;
    std::shared_ptr<LegCalc> lf_leg_calc;
    std::shared_ptr<LegCalc> rf_leg_calc;
    std::shared_ptr<LegCalc> lb_leg_calc;
    std::shared_ptr<LegCalc> rb_leg_calc;

    // 数据缓存部分
    Eigen::Vector3d lf_joint_pos, lf_joint_vel;
    Eigen::Vector3d rf_joint_pos, rf_joint_vel;
    Eigen::Vector3d lb_joint_pos, lb_joint_vel;
    Eigen::Vector3d rb_joint_pos, rb_joint_vel;
    Eigen::Vector3d lf_forward_torque, lf_joint_torque;
    Eigen::Vector3d rf_forward_torque, rf_joint_torque;
    Eigen::Vector3d lb_forward_torque, lb_joint_torque;
    Eigen::Vector3d rb_forward_torque, rb_joint_torque;
    Eigen::Vector3d lf_base_offset,rf_base_offset,lb_base_offset,rb_base_offset;
    Eigen::Vector3d lf_leg_stop_pos,rf_leg_stop_pos,lb_leg_stop_pos,rb_leg_stop_pos;
};
