#pragma once

#include "fsm/base_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <robot_interfaces/msg/move_cmd.hpp>
#include <robot_interfaces/msg/jump_cmd.hpp>
#include "leg/step.hpp"

// 前向声明
class Robot;

class JumpState : public BaseState<Robot>{
public:
    JumpState(Robot* robot);
    
    bool enter(Robot* robot, const std::string &last_status) override;
    std::string update(Robot* robot) override;
    
private:
    rclcpp::Subscription<robot_interfaces::msg::JumpCmd>::SharedPtr jump_cmd_sub;
    robot_interfaces::msg::JumpCmd jump_cmd;
    rclcpp::Time action_start_time;
    Robot *robot;
    double current_exp_vel,current_body_vel;
    double exp_vel_kp{0.5};
    int stage;
    double ver_acc{0.0},ver_vel{0.0},ver_pos{0.0};

    double la,lb,lc;
};
