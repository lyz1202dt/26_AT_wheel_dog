#pragma once

#include "fsm/base_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <robot_interfaces/msg/brigeb.hpp>
#include <leg/leg_calc.hpp>

#include <tf2/LinearMath/Transform.hpp>

// 前向声明
class Robot;

class Brige_B : public BaseState<Robot>{
public:
    Brige_B(Robot* robot);
    
    bool enter(Robot* robot, const std::string &last_status) override;
    std::string update(Robot* robot) override;
    
private:
    Robot* robot;
    
    bool calc_edge_line(const robot_interfaces::msg::Brigeb &msg);
    double get_wheel2edge_distance(std::shared_ptr<LegCalc> leg_calc,const Eigen::Vector3d &joint_pos);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> balance_force_calc(Robot* robot, double cur_roll, double cur_pitch);
    double line_k{0.0};
    double line_b{0.0};
    bool lower_edge{false}; // 是否是台阶下降边缘
    rclcpp::Time last_line_update_time;
    int state{0};
    double front_step_length{0.2};
    double back_step_length{0.2};
    int active_feet{0};
    double vel_kp{2.0};
    double next_step_length{0.12};
    double mass;
    double  step_dy{0.08};
    Eigen::Vector2d mass_center_pos;
    rclcpp::Time start_time;
    LegStep lf_leg_step, rf_leg_step, lb_leg_step, rb_leg_step;

    Vector3D lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos;
    Vector3D lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force;
    Vector3D lf_foot_exp_vel, rf_foot_exp_vel, lb_foot_exp_vel, rb_foot_exp_vel;
    Vector3D lf_foot_exp_acc, rf_foot_exp_acc, lb_foot_exp_acc, rb_foot_exp_acc;
};
