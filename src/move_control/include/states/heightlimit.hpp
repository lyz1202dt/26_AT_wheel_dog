#pragma once
#include "fsm/base_state.hpp"
#include "core/robot.hpp"
#include "leg/step.hpp"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

class HeightlimitState : public BaseState<Robot> {
public:
    explicit HeightlimitState(Robot* robot);
    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;

private:
    double low_pass_filter(double input, int wheel_idx);

    int stage = 0;
    rclcpp::Time start_time;
    rclcpp::Time last_time;

    double fall_time  = 1.5;
    double up_time    = 1.5;
    double drive_time = 2.0;
    double wheel_vel  = 5.0;
    double target_distance = 0.55;

    Eigen::Vector3d lf_init, rf_init, lb_init, rb_init;
    LegStep lf_leg_step, rf_leg_step, lb_leg_step, rb_leg_step;

    // 滤波
    double last_filtered_wheel_vel[4];

    // 距离积分
    double total_distance = 0.0;
};
