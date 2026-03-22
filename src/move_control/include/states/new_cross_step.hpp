#ifndef NEW_CROSS_STEP_HPP
#define NEW_CROSS_STEP_HPP

#include "core/robot.hpp"
#include "leg/step.hpp"
#include <rclcpp/rclcpp.hpp>

using Vector3D = Eigen::Vector3d;
using Vector2D = Eigen::Vector2d;

class JumpStepState : public BaseState<Robot>
{
public:
    explicit JumpStepState(Robot* robot);
    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;bool execute(Robot* robot) ;

private:
    bool flag;
    bool stage3_init{false};
    int jump_stage;
    rclcpp::Time jump_stage_time;
    Vector3D foot_force_compen{0.0, 0.0, 0.0}; 
    Vector3D step_lf_foot_pos;
    Vector3D step_rf_foot_pos;
    Vector3D step_lb_foot_pos;
    Vector3D step_rb_foot_pos;
   
    LegStep lf_leg_step, rf_leg_step, lb_leg_step, rb_leg_step;

    Vector3D lf_foot_exp_pos{0,0,0}, rf_foot_exp_pos{0,0,0}, lb_foot_exp_pos{0,0,0}, rb_foot_exp_pos{0,0,0};
    Vector3D lf_foot_exp_vel{0,0,0}, rf_foot_exp_vel{0,0,0}, lb_foot_exp_vel{0,0,0}, rb_foot_exp_vel{0,0,0};
    Vector3D lf_foot_exp_acc{0,0,0}, rf_foot_exp_acc{0,0,0}, lb_foot_exp_acc{0,0,0}, rb_foot_exp_acc{0,0,0};

    float lf_wheel_vel, rf_wheel_vel, lb_wheel_vel, rb_wheel_vel;
    float lf_wheel_force, rf_wheel_force, lb_wheel_force, rb_wheel_force;
};

#endif
