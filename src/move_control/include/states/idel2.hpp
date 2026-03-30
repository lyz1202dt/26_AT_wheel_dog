#pragma once

#include "fsm/base_state.hpp"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <rclcpp/timer.hpp>
#include "leg/step.hpp"

class Robot;

class Idel2State : public BaseState<Robot> {
public:
    Idel2State(Robot* robot);
    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;

private:
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
        balance_force_calc(Robot* robot, double cur_roll, double cur_pitch);

    bool trajectory_calced{false};
    rclcpp::Time setup_time;
    LegStep lf_leg_step, rf_leg_step, lb_leg_step, rb_leg_step;
    std::tuple<Vector3D, Vector3D, Vector3D> lf_joint_target;
    std::tuple<Vector3D, Vector3D, Vector3D> rf_joint_target;
    std::tuple<Vector3D, Vector3D, Vector3D> lb_joint_target;
    std::tuple<Vector3D, Vector3D, Vector3D> rb_joint_target;
};