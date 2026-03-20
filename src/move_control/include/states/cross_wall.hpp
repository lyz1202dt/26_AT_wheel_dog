#pragma once

#include "fsm/base_state.hpp"
#include "leg/step.hpp"
#include <Eigen/Dense>
#include <leg/leg_calc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tuple>

class Robot;

class Cross_WallState : public BaseState<Robot> {
public:
    Cross_WallState(Robot* robot);
    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;


private:
    //使用笛卡尔坐标系
    int cross_wall_stage{-1};
    rclcpp::Time cross_wall_stage_time;
    Vector3D wall_lf_foot_pos{0,0,0}, wall_rf_foot_pos{0,0,0}, wall_lb_foot_pos{0,0,0}, wall_rb_foot_pos{0,0,0};
    Vector3D lf_foot_exp_pos{0,0,0}, rf_foot_exp_pos{0,0,0}, lb_foot_exp_pos{0,0,0}, rb_foot_exp_pos{0,0,0};
    Vector3D lf_foot_exp_force{0,0,0}, rf_foot_exp_force{0,0,0}, lb_foot_exp_force{0,0,0}, rb_foot_exp_force{0,0,0};
    Vector3D lf_foot_exp_vel{0,0,0}, rf_foot_exp_vel{0,0,0}, lb_foot_exp_vel{0,0,0}, rb_foot_exp_vel{0,0,0};
    Vector3D lf_foot_exp_acc{0,0,0}, rf_foot_exp_acc{0,0,0}, lb_foot_exp_acc{0,0,0}, rb_foot_exp_acc{0,0,0};
    Vector3D lf_forward_torque{0,0,0}, rf_forward_torque{0,0,0}, lb_forward_torque{0,0,0}, rb_forward_torque{0,0,0};

    //使用关节角度
    Vector3D lf_joint_exp_pos_{0,0,0},rf_joint_exp_pos_{0,0,0},
             lb_joint_exp_pos_{0,0,0},rb_joint_exp_pos_{0,0,0};
    Vector3D lf_joint_omega{0,0,0}, rf_joint_omega{0,0,0},
             lb_joint_omega{0,0,0},rb_joint_omega{0,0,0};
    Vector3D lf_joint_torque{0,0,0},rf_joint_torque{0,0,0},
             lb_joint_torque{0,0,0},rb_joint_torque{0,0,0};

    double lf_wheel_vel{0.0},rf_wheel_vel{0.0},lb_wheel_vel{0.0},rb_wheel_vel{0.0};
    double lf_wheel_force{0.0},rf_wheel_force{0.0},lb_wheel_force{0.0},rb_wheel_force{0.0};

    bool stopping = false;
    double stop_t = 0.0;
    double stop_T = 0.6;   // 建议 0.3~0.6

    double lf_vel_start, rf_vel_start, lb_vel_start, rb_vel_start;
    double lf_force_start, rf_force_start, lb_force_start, rb_force_start;

    bool enable_posture_safe{true};
    
    LegStep lf_leg_step, rf_leg_step, lb_leg_step, rb_leg_step;
    double cross_x_lf{0.0},cross_y_lf{0.0},cross_z_lf{0.0};
    double cross_x_rf{0.0},cross_y_rf{0.0},cross_z_rf{0.0};
    double cross_x_lb{0.0},cross_y_lb{0.0},cross_z_lb{0.0};
    double cross_x_rb{0.0},cross_y_rb{0.0},cross_z_rb{0.0};
    double time_s{1.0};
    bool change_flag{false};
    bool allow_vel{true};

    float k_F{1.0f};

    //力变量
    Vector2D mass_center_pos;
    double mass;

};