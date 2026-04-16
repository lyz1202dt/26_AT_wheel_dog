#pragma once

#include "fsm/base_state.hpp"
#include "leg/step.hpp"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tuple>

class Robot;

class WalkState : public BaseState<Robot> {
public:
    WalkState(Robot* robot);
    ~WalkState();
    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;

private:
    std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d> calc_foot_vel(Robot* robot, Eigen::Vector3d exp_vel);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
        balance_force_calc(Robot* robot, double cur_roll, double cur_pitch);

    Eigen::Vector2d lf_exp_vel, rf_exp_vel, lb_exp_vel, rb_exp_vel;
    LegStep lf_leg_step, rf_leg_step, lb_leg_step, rb_leg_step;

    double step_time{0.5};                                        // 整个对角步态全程的时间
    double step_height{0.08};
    double step_support_rate{0.6};
    rclcpp::Time main_phrase_start_time, slave_phrase_start_time; // 第一、二相位步态开始时间
    rclcpp::Time slave_phrase_stop_time;
    bool step1_support_updated{false};
    bool step2_support_updated{false};
    bool step1_flight_updated{false};
    bool step2_flight_updated{false};

    double roll_balance_step_compen{0.3};
    double pitch_balance_step_compen{0.3};
    double exp_roll{0.0},exp_pitch{0.0};
    bool first_update = true;
    Vector3D last_lf_foot_exp_pos{0.0,0.0,0.0},last_rf_foot_exp_pos{0.0,0.0,0.0},
             last_lb_foot_exp_pos{0.0,0.0,0.0},last_rb_foot_exp_pos{0.0,0.0,0.0};


};
