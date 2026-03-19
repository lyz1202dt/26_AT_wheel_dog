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
    double mass;
    Vector2D mass_center_pos;
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
        balance_force_calc(Robot* robot, double cur_roll, double cur_pitch);
    Vector3D signal_leg_torque_calc(
        std::shared_ptr<LegCalc> leg_calc, const Vector3D& cur_joint_pos, const Vector3D& exp_foot_force, const Vector3D& foot_vel,
        const Vector3D& foot_acc);

    SimpleVMC roll_vmc, pitch_vmc;
    SimpleVMC lf_leg_vmc_z, rf_leg_vmc_z, lb_leg_vmc_z, rb_leg_vmc_z;
    SimpleVMC lf_leg_vmc_x, rf_leg_vmc_x, lb_leg_vmc_x, rb_leg_vmc_x;
    SimpleVMC lf_leg_vmc_y, rf_leg_vmc_y, lb_leg_vmc_y, rb_leg_vmc_y;

    Vector3D lf_last_vel,rf_last_vel,lb_last_vel,rb_last_vel;
    double filter_gate{0.5};
};
