#ifndef NEW_CROSS_STEP_HPP
#define NEW_CROSS_STEP_HPP

#include "core/robot.hpp"
#include "leg/step.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

class JumpStepState : public BaseState<Robot>
{
public:
    explicit JumpStepState(Robot* robot);

    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;

private:
    // 阶段控制
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;
    void default_param_cb(const rclcpp::Parameter& param);
    int jump_stage = 0;
    rclcpp::Time jump_stage_time;
   
    // 足端位置缓存（初始化零向量）
    Eigen::Vector3d step_lf_foot_pos  = Eigen::Vector3d::Zero();
    Eigen::Vector3d step_rf_foot_pos  = Eigen::Vector3d::Zero();
    Eigen::Vector3d step_lb_foot_pos  = Eigen::Vector3d::Zero();
    Eigen::Vector3d step_rb_foot_pos  = Eigen::Vector3d::Zero();

    // 轨迹规划对象
    LegStep lf_leg_step;
    LegStep rf_leg_step;
    LegStep lb_leg_step;
    LegStep rb_leg_step;

    // 期望足端状态（全部初始化为 0）
    Eigen::Vector3d lf_foot_exp_pos   = Eigen::Vector3d::Zero();
    Eigen::Vector3d lf_foot_exp_vel   = Eigen::Vector3d::Zero();
    Eigen::Vector3d lf_foot_exp_acc   = Eigen::Vector3d::Zero();

    Eigen::Vector3d rf_foot_exp_pos   = Eigen::Vector3d::Zero();
    Eigen::Vector3d rf_foot_exp_vel   = Eigen::Vector3d::Zero();
    Eigen::Vector3d rf_foot_exp_acc   = Eigen::Vector3d::Zero();

    Eigen::Vector3d lb_foot_exp_pos   = Eigen::Vector3d::Zero();
    Eigen::Vector3d lb_foot_exp_vel   = Eigen::Vector3d::Zero();
    Eigen::Vector3d lb_foot_exp_acc   = Eigen::Vector3d::Zero();

    Eigen::Vector3d rb_foot_exp_pos   = Eigen::Vector3d::Zero();
    Eigen::Vector3d rb_foot_exp_vel   = Eigen::Vector3d::Zero();
    Eigen::Vector3d rb_foot_exp_acc   = Eigen::Vector3d::Zero();

    // 轮子控制
    double lf_wheel_vel    = 0.0;
    double rf_wheel_vel    = 0.0;
    double lb_wheel_vel    = 0.0;
    double rb_wheel_vel    = 0.0;

    double lf_wheel_force  = 0.0;
    double rf_wheel_force  = 0.0;
    double lb_wheel_force  = 0.0;
    double rb_wheel_force  = 0.0;

    // 辅助变量
    bool flag = false;
    Eigen::Vector3d foot_force_compen = Eigen::Vector3d::Zero();
};

#endif
