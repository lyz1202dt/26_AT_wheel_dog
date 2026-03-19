#pragma once

#include "fsm/base_state.hpp"
#include "leg/step.hpp"
#include <rclcpp/rclcpp.hpp>


// 前向声明
class Robot;

class ClimbSteps2State : public BaseState<Robot> {
public:
    ClimbSteps2State(Robot* robot);

    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;

private:
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
    balance_force_calc(Robot* robot, double cur_roll, double cur_pitch);

    Vector2D mass_center_pos;
    double mass;
    Vector3D lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos;
    Vector3D lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force;
    Vector3D lf_foot_exp_vel, rf_foot_exp_vel, lb_foot_exp_vel, rb_foot_exp_vel;
    Vector3D lf_foot_exp_acc, rf_foot_exp_acc, lb_foot_exp_acc, rb_foot_exp_acc;
    double lf_wheel_vel, rf_wheel_vel, lb_wheel_vel, rb_wheel_vel;
    double lf_wheel_force, rf_wheel_force, lb_wheel_force, rb_wheel_force;
    Vector3D last_pos_1, last_pos_2;
    int step_state;
    double step_time{2.0};
    rclcpp::Time start_time;
    LegStep lf_leg_step, rf_leg_step, lb_leg_step, rb_leg_step;
    bool lf_leg_trajectory_updated{false}, rf_leg_trajectory_updated{false}, lb_leg_trajectory_updated{false},
        rb_leg_trajectory_updated{false};
    Vector3D lf_cart_force,rf_cart_force,lb_cart_force,rb_cart_force;
    double  step_dy{0.08};
    double exp_vel_kp{3.0};
    double current_exp_vel{0.0};
    double current_body_vel{0.0};
    double foot_obstruct_gate{9.0};
    int req_state{0};
    int last_state{0};
    int current_state{0};
    int foot_climbing_step{0};
    bool foot_trajectory_updated{false};
    rclcpp::Time foot_climbing_time;
    rclcpp::Time last_foot_climbing_end_time;  // 最后一次完成抬腿的时间，用于冷却计时
    bool allow_next_climbing{true};
};
