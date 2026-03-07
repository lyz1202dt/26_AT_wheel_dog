#pragma once

#include "fsm/base_state.hpp"
#include "leg/step.hpp"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tuple>

class Robot;

class Cross_WallState : public BaseState<Robot> {
public:
    Cross_WallState(Robot* robot);
    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;

private:
    int cross_wall_stage{0};
    rclcpp::Time cross_wall_stage_time;
    Vector3D wall_lf_foot_pos{0,0,0}, wall_rf_foot_pos{0,0,0}, wall_lb_foot_pos{0,0,0}, wall_rb_foot_pos{0,0,0};
    Vector3D lf_foot_exp_pos{0,0,0}, rf_foot_exp_pos{0,0,0}, lb_foot_exp_pos{0,0,0}, rb_foot_exp_pos{0,0,0};
    Vector3D lf_foot_exp_force{0,0,0}, rf_foot_exp_force{0,0,0}, lb_foot_exp_force{0,0,0}, rb_foot_exp_force{0,0,0};
    Vector3D lf_foot_exp_vel{0,0,0}, rf_foot_exp_vel{0,0,0}, lb_foot_exp_vel{0,0,0}, rb_foot_exp_vel{0,0,0};
    Vector3D lf_foot_exp_acc{0,0,0}, rf_foot_exp_acc{0,0,0}, lb_foot_exp_acc{0,0,0}, rb_foot_exp_acc{0,0,0};
    Vector3D lf_forward_torque{0,0,0}, rf_forward_torque{0,0,0}, lb_forward_torque{0,0,0}, rb_forward_torque{0,0,0};
    
    double lf_wheel_vel{0.0},rf_wheel_vel{0.0},lb_wheel_vel{0.0},rb_wheel_vel{0.0};
    double lf_wheel_force{0.0},rf_wheel_force{0.0},lb_wheel_force{0.0},rb_wheel_force{0.0};
    bool enable_posture_safe{true};
    
    LegStep lf_leg_step, rf_leg_step, lb_leg_step, rb_leg_step;
    
};