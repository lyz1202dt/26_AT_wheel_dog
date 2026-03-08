#pragma once

#include "fsm/base_state.hpp"
#include "leg/leg_calc.hpp"
#include "leg/step.hpp"
#include "leg/vmc.hpp"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tuple>

class Robot;

class ForceState : public BaseState<Robot> {
public:
    ForceState(Robot* robot);
    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;

private:
    std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d> calc_foot_vel(Robot* robot, Eigen::Vector3d exp_vel);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
        balance_force_calc(Robot* robot, double cur_roll, double cur_pitch);
    Vector3D signal_leg_torque_calc(
        std::shared_ptr<LegCalc> leg_calc, const Vector3D& cur_joint_pos, const Vector3D& exp_foot_force, const Vector3D& foot_vel,
        const Vector3D& foot_acc);

    Eigen::Vector2d lf_exp_vel, rf_exp_vel, lb_exp_vel, rb_exp_vel;
    LegStep lf_leg_step, rf_leg_step, lb_leg_step, rb_leg_step;

    SimpleVMC roll_vmc, pitch_vmc;
    SimpleVMC lf_leg_vmc_z, rf_leg_vmc_z, lb_leg_vmc_z, rb_leg_vmc_z;
    SimpleVMC lf_leg_vmc_x, rf_leg_vmc_x, lb_leg_vmc_x, rb_leg_vmc_x;
    SimpleVMC lf_leg_vmc_y, rf_leg_vmc_y, lb_leg_vmc_y, rb_leg_vmc_y;

    double step_time{0.5};                                        // 整个对角步态全程的时间
    double step_height{0.08};
    double step_support_rate{0.6};
    rclcpp::Time main_phrase_start_time, slave_phrase_start_time; // 第一、二相位步态开始时间
    rclcpp::Time slave_phrase_stop_time;
    bool step1_support_updated{false};
    bool step2_support_updated{false};
    bool step1_flight_updated{false};
    bool step2_flight_updated{false};
};
