#pragma once

#include "fsm/fsm.hpp"
#include "leg/leg_calc.hpp"
#include "leg/vmc.hpp"

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <ctime>
#include <functional>
#include <memory>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <tuple>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <map>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <robot_interfaces/msg/move_cmd.hpp>
#include <robot_interfaces/msg/robot.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>

class Robot {
public:
    Robot(const std::shared_ptr<rclcpp::Node> node);
    ~Robot();


    enum DogReqState { // 请求的机器人状态
        DOG_REQ_IDEL=0,
        DOG_REQ_STOP=1,
        DOG_REQ_RUN=2
    };

    static constexpr double WHEEL_RADIUS = 0.065;

    void show_callback();

    Vector3D get_grivate_center_pose(
        const Vector3D& lf_joint_pos, const Vector3D& rf_joint_pos, const Vector3D& lb_joint_pos, const Vector3D& rb_joint_pos);
    robot_interfaces::msg::Leg signal_leg_calc(
        const Vector3D& exp_cart_pos, const Vector3D& exp_cart_vel, const Vector3D& exp_cart_acc, const Vector3D& exp_cart_force,
        std::shared_ptr<LegCalc> leg_calc, Vector3D* torque, double wheel_vel = 0.0, double wheel_force = 0.0);
    void quaternionLowPassFilter(double& w, double& x, double& y, double& z, double w1, double x1, double y1, double z1, double alpha);

    bool add_param_cb(std::function<bool(const rclcpp::Parameter& params)> callback);
    std::vector<std::function<bool(const rclcpp::Parameter& params)>> param_cb_vector;

    bool default_param_cb(const rclcpp::Parameter& params);

    // 机器人主状态机
    FSM<Robot> fsm;

    // ROS2通信相关话题
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;
    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr legs_target_pub;
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr legs_state_sub;
    rclcpp::Subscription<robot_interfaces::msg::MoveCmd>::SharedPtr move_cmd_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_angular_vel_sub;
    rclcpp::SyncParametersClient::SharedPtr robot_description_param_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> robot_tf_broadcaster;

    // 可视化相关
    rclcpp::TimerBase::SharedPtr ui_update_timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_publisher;
    std::vector<std::string> joint_names = {"lf_joint1", "lf_joint2", "lf_joint3", "lf_joint4", "rf_joint1", "rf_joint2",
                                            "rf_joint3", "rf_joint4", "lb_joint1", "lb_joint2", "lb_joint3", "lb_joint4",
                                            "rb_joint1", "rb_joint2", "rb_joint3", "rb_joint4"};
    sensor_msgs::msg::JointState joint_display_msg;

    // 机器人位姿信息
    tf2::Quaternion robot_rotation;           // 机器人姿态
    geometry_msgs::msg::Twist robot_velocity; // 机器人速度信息
    double direction_filter_gate{0.8};        // 方向滤波器参数


    // 解算部分
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

    // 狗腿数据缓存
    Eigen::Vector3d lf_joint_pos, lf_joint_vel, lf_joint_torque, lf_forward_torque;
    Eigen::Vector3d rf_joint_pos, rf_joint_vel, rf_joint_torque, rf_forward_torque;
    Eigen::Vector3d lb_joint_pos, lb_joint_vel, lb_joint_torque, lb_forward_torque;
    Eigen::Vector3d rb_joint_pos, rb_joint_vel, rb_joint_torque, rb_forward_torque;
    Eigen::Vector3d lf_leg_stop_pos, rf_leg_stop_pos, lb_leg_stop_pos, rb_leg_stop_pos;

    // 用户命令输入
    robot_interfaces::msg::MoveCmd move_cmd;

    // VMC对象
    std::shared_ptr<VMC> lf_z_vmc, lf_x_vmc, lf_y_vmc;
    std::shared_ptr<VMC> rf_z_vmc, rf_x_vmc, rf_y_vmc;
    std::shared_ptr<VMC> lb_z_vmc, lb_x_vmc, lb_y_vmc;
    std::shared_ptr<VMC> rb_z_vmc, rb_x_vmc, rb_y_vmc;
    std::shared_ptr<SimpleVMC> roll_vmc, pitch_vmc;


    // 共享参数
    bool legs_data_updated{false};
    Vector3D comm_pos;
    Eigen::Vector3d lf_base_offset, rf_base_offset, lb_base_offset, rb_base_offset;
    double body_height{0.25};
    double robot_lf_grivate{0.0};
    double robot_rf_grivate{0.0};
    double robot_lb_grivate{0.0};
    double robot_rb_grivate{0.0};
};
