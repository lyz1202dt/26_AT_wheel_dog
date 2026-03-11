#pragma once

#include "fsm/base_state.hpp"
#include "leg/step.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <robot_interfaces/msg/jump_cmd.hpp>
#include <robot_interfaces/msg/move_cmd.hpp>

// 前向声明
class Robot;

class JumpState : public BaseState<Robot> {
public:
    JumpState(Robot* robot);

    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;

private:
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
        balance_force_calc(Robot* robot, double cur_roll, double cur_pitch);
    rclcpp::Subscription<robot_interfaces::msg::JumpCmd>::SharedPtr jump_cmd_sub;
    robot_interfaces::msg::JumpCmd jump_cmd;
    rclcpp::Time action_start_time;
    Robot* robot;
    double current_exp_vel, current_body_vel;
    double exp_vel_kp{1.0};
    int stage;
    double ver_acc{0.0}, ver_vel{0.0}, ver_pos{0.0};
    double hor_acc{0.0}, hor_vel{0.0}, hor_pos{0.0};

    double ver_la, ver_lb, ver_lc;
    double hor_la, hor_lb, hor_lc;
    double action_time;
};
