#pragma once

#include "fsm/base_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include "leg/step.hpp"

// 前向声明
class Robot;

class SetupState : public BaseState<Robot>{
public:
    SetupState(Robot* robot);
    
    bool enter(Robot* robot, const std::string &last_status) override;
    std::string update(Robot* robot) override;
    
private:
    int setup_stage{0};
    bool trajectory_calced{false};
    rclcpp::Time setup_time;
    LegStep lf_leg_step, rf_leg_step, lb_leg_step, rb_leg_step;
};
