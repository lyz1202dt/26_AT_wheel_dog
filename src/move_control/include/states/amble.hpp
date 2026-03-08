#pragma once

#include "fsm/base_state.hpp"
#include "leg/step.hpp"
#include <rclcpp/rclcpp.hpp>


// 前向声明
class Robot;

class AmbleState : public BaseState<Robot> {
public:
    AmbleState(Robot* robot);

    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;

private:
    Vector3D get_next_available_pos(Vector3D leg_offset,Vector3D current_pos);

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
    double  step_dy{0.08};
};
