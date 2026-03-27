#include "states/new_cross_step.hpp"
#include "core/robot.hpp"
#include "leg/step.hpp"
#include <rcl/timer.h>
#include <rclcpp/logging.hpp>
#include <math.h>

bool use_limit_lf = false;
bool use_limit_rf = false;
bool use_limit_lb = false;
bool use_limit_rb = false;

// 构造函数
JumpStepState::JumpStepState(Robot* robot)
    : BaseState<Robot>("jump_step")
{
    // 1. 声明参数 jump_stage (int 类型)
    robot->node_->declare_parameter<int>("jump_stage", 0);

    // 2. 注册动态参数修改回调（rqt 实时生效关键）
    param_server_ = robot->node_->add_on_set_parameters_callback(
        [this, robot](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            // 遍历更新参数
            for (const auto& param : params) {
                default_param_cb(param);
            }

            RCLCPP_INFO(robot->node_->get_logger(), 
                "[JumpStep] 参数已动态更新 → jump_stage: %d", jump_stage);
            return result;
        }
    );

    // 3. 初始化读取参数
    robot->node_->get_parameter("jump_stage", jump_stage);
}

// 参数回调：rqt 修改后自动更新变量
void JumpStepState::default_param_cb(const rclcpp::Parameter& param)
{
    if (param.get_name() == "jump_stage") {
        jump_stage = param.as_int();
    }
}

// 进入状态时也刷新参数
bool JumpStepState::enter(Robot* robot, const std::string& last_status)
{
    robot->node_->get_parameter("jump_stage", jump_stage);
    RCLCPP_INFO(robot->node_->get_logger(), 
        "进入跳跃状态 → 当前 jump_stage = %d", jump_stage);
    return true;
}

std::string JumpStepState::update(Robot* robot)
{
    robot_interfaces::msg::RobotTarget joints_target;

    // 默认重力补偿
    auto lf_force = Vector3D(0,0,-robot->robot_lf_grivate);
    auto rf_force = Vector3D(0,0,-robot->robot_rf_grivate);
    auto lb_force = Vector3D(0,0,-robot->robot_lb_grivate);
    auto rb_force = Vector3D(0,0,-robot->robot_rb_grivate);

    // ========================== 阶段 0：缓慢靠近台阶 ==========================
    if (jump_stage == 0)
    {
        RCLCPP_INFO(robot->node_->get_logger(), "阶段0：靠近台阶");
        lf_wheel_vel = 0.3;
        rf_wheel_vel = -0.3;
        lb_wheel_vel = 0.3;
        rb_wheel_vel = -0.3;

        lf_wheel_force = 0;
        rf_wheel_force = 0;
        lb_wheel_force = 0;
        rb_wheel_force = 0;

        // 检测前足接触台阶
        auto f_lf = robot->lf_leg_calc->foot_force(robot->lf_joint_pos, robot->lf_joint_torque, robot->lf_forward_torque);
        auto f_rf = robot->rf_leg_calc->foot_force(robot->rf_joint_pos, robot->rf_joint_torque, robot->rf_forward_torque);

        if (f_lf[0] > 2.0 && f_rf[0] > 2.0)
        {
            // 记录当前足端位置
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

         
           
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(-0.07,0.25,0.18), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0,0,0.09), 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, lb_current_pos, 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0,0,0.09), 0.5);
            
            jump_stage_time = robot->node_->get_clock()->now();
            jump_stage = 1;
        }
    }

    // ========================== 阶段 1：抬起左前腿 ==========================
    else if (jump_stage == 1)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段1:抬起左前腿");

        // 只获取轨迹，不规划
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        lf_wheel_vel = 0; rf_wheel_vel = 0; lb_wheel_vel = 0; rb_wheel_vel = 0;

        // ✅ 阶段1完成后 → 规划阶段2轨迹
        if (t > 0.5) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

    lf_leg_step.update_flight_trajectory(lf_current_pos, {0,0,0},
                {0.08, 0, 0.15}, {0,0}, 1, 0.15);
    rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0,0,0.0), 1);
    lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0,0,0.0), 1);
    rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0,0,0.0), 1);
            
            jump_stage = 2;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 2：左前腿落下 ==========================
    else if (jump_stage == 2)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段2：左前腿落下");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        

        // ✅ 阶段2完成后 → 规划阶段3轨迹
        if (t > 1) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
       
            lf_leg_step.update_support_trajectory(lf_current_pos, lf_current_pos, 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, lb_current_pos, 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, rb_current_pos, 0.5);
            rf_leg_step.update_flight_trajectory(rf_current_pos, {0,0,0}, {0.05, 0.0, 0.12}, {0,0}, 0.5, 0.12);
            
            jump_stage = 300;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 3：抬起右前腿 ==========================
    else if (jump_stage == 3)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段3：抬起右前腿");
    
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // ✅ 阶段3完成后 → 规划阶段4轨迹
        if (t > 0.5) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            rf_leg_step.update_flight_trajectory(rf_current_pos + Vector3D(0.05,0,0.12), {0,0,0},{0.03, 0.0, -0.03}, {0,0}, 0.3, 0.0);
            lf_leg_step.update_support_trajectory(lf_current_pos, lf_current_pos, 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, lb_current_pos, 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, rb_current_pos, 0.5);
            
            jump_stage = 4;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 4：右前腿落下 ==========================
    else if (jump_stage == 4)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段4：右前腿落下");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // ✅ 阶段4完成后 → 直接进入阶段5（无轨迹规划）
        if (t > 0.3) {
            jump_stage = 5;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 5：身体推进 ==========================
    else if (jump_stage == 5)
    {
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段5：身体推进");

        lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;

        lf_foot_exp_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        rf_foot_exp_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        lb_foot_exp_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        rb_foot_exp_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        // ✅ 阶段5完成后 → 规划阶段6轨迹
        if (t > 1.0) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lb_leg_step.update_flight_trajectory(lb_current_pos, {0,0,0}, {0.05, 0.0, 0.12}, {0,0}, 0.5, 0.12);
            lf_leg_step.update_support_trajectory(lf_current_pos, lf_current_pos, 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, rf_current_pos, 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, rb_current_pos, 0.5);
            
            jump_stage = 6;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 6：抬起左后腿（上第一节台阶） ==========================
    else if (jump_stage == 6)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段6：抬起左后腿");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // ✅ 阶段6完成后 → 规划阶段7轨迹
        if (t > 0.5) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lb_leg_step.update_flight_trajectory(lb_current_pos + Vector3D(0.05,0,0.12), {0,0,0}, {0.03, 0.0, -0.03}, {0,0}, 0.3, 0.0);
            lf_leg_step.update_support_trajectory(lf_current_pos, lf_current_pos, 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, rf_current_pos, 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, rb_current_pos, 0.5);
            
            jump_stage = 7;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 7：左后腿落下 ==========================
    else if (jump_stage == 7)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段7：左后腿落下");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // ✅ 阶段7完成后 → 规划阶段8轨迹
        if (t > 0.3) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            rb_leg_step.update_flight_trajectory(rb_current_pos, {0,0,0}, {0.05, 0.0, 0.12}, {0,0}, 0.5, 0.12);
            lf_leg_step.update_support_trajectory(lf_current_pos, lf_current_pos, 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, rf_current_pos, 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, lb_current_pos, 0.5);
            
            jump_stage = 8;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 8：抬起右后腿 ==========================
    else if (jump_stage == 8)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段8：抬起右后腿");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // ✅ 阶段8完成后 → 规划阶段9轨迹
        if (t > 0.5) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            rb_leg_step.update_flight_trajectory(rb_current_pos + Vector3D(0.05,0,0.12), {0,0,0}, {0.03, 0.0, -0.03}, {0,0}, 0.3, 0.0);
            lf_leg_step.update_support_trajectory(lf_current_pos, lf_current_pos, 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, rf_current_pos, 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, lb_current_pos, 0.5);
            
            jump_stage = 9;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 9：右后腿落下 ==========================
    else if (jump_stage == 9)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段9：右后腿落下");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // ✅ 阶段9完成后 → 进入阶段10
        if (t > 0.3) {
            jump_stage = 10;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 10：整体推进，准备上第二节台阶 ==========================
    else if (jump_stage == 10)
    {
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段10：推进到第二节台阶");

        lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;

        lf_foot_exp_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        rf_foot_exp_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        lb_foot_exp_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        rb_foot_exp_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        // ✅ 阶段10完成后 → 规划阶段11轨迹
        if (t > 1.0) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_flight_trajectory(lf_current_pos, {0,0,0}, {0.05, 0.0, 0.12}, {0,0}, 0.5, 0.12);
            rf_leg_step.update_support_trajectory(rf_current_pos, rf_current_pos, 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, lb_current_pos, 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, rb_current_pos, 0.5);
            
            jump_stage = 11;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 11：左前腿 上 第二节台阶 ==========================
    else if (jump_stage == 11)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段11：左前腿上第二节台阶");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // ✅ 阶段11完成后 → 规划阶段12轨迹
        if (t > 0.5) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_flight_trajectory(lf_current_pos + Vector3D(0.05,0,0.12), {0,0,0}, {0.03, 0.0, -0.03}, {0,0}, 0.3, 0.0);
            rf_leg_step.update_support_trajectory(rf_current_pos, rf_current_pos, 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, lb_current_pos, 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, rb_current_pos, 0.5);
            
            jump_stage = 12;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 12：左前腿落下 ==========================
    else if (jump_stage == 12)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段12：左前腿落下");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // ✅ 阶段12完成后 → 规划阶段13轨迹
        if (t > 0.3) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            rf_leg_step.update_flight_trajectory(rf_current_pos, {0,0,0}, {0.05, 0.0, 0.12}, {0,0}, 0.5, 0.12);
            lf_leg_step.update_support_trajectory(lf_current_pos, lf_current_pos, 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, lb_current_pos, 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, rb_current_pos, 0.5);
            
            jump_stage = 13;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 13：右前腿 上 第二节台阶 ==========================
    else if (jump_stage == 13)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段13：右前腿上第二节台阶");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // ✅ 阶段13完成后 → 规划阶段14轨迹
        if (t > 0.5) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            rf_leg_step.update_flight_trajectory(rf_current_pos + Vector3D(0.05,0,0.12), {0,0,0}, {0.03, 0.0, -0.03}, {0,0}, 0.3, 0.0);
            lf_leg_step.update_support_trajectory(lf_current_pos, lf_current_pos, 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, lb_current_pos, 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, rb_current_pos, 0.5);
            
            jump_stage = 14;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 14：右前腿落下 ==========================
    else if (jump_stage == 14)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段14：右前腿落下");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // ✅ 阶段14完成后 → 进入阶段15
        if (t > 0.3) {
            jump_stage = 15;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 15：身体推进 ==========================
    else if (jump_stage == 15)
    {
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段15：身体推进");

        lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;

        lf_foot_exp_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        rf_foot_exp_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        lb_foot_exp_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        rb_foot_exp_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        // ✅ 阶段15完成后 → 规划阶段16轨迹
        if (t > 1.0) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lb_leg_step.update_flight_trajectory(lb_current_pos, {0,0,0}, {0.05, 0.0, 0.12}, {0,0}, 0.5, 0.12);
            lf_leg_step.update_support_trajectory(lf_current_pos, lf_current_pos, 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, rf_current_pos, 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, rb_current_pos, 0.5);
            
            jump_stage = 16;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 16：左后腿 上 第二节台阶 ==========================
    else if (jump_stage == 16)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段16：左后腿上第二节台阶");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // ✅ 阶段16完成后 → 规划阶段17轨迹
        if (t > 0.5) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lb_leg_step.update_flight_trajectory(lb_current_pos + Vector3D(0.05,0,0.12), {0,0,0}, {0.03, 0.0, -0.03}, {0,0}, 0.3, 0.0);
            lf_leg_step.update_support_trajectory(lf_current_pos, lf_current_pos, 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, rf_current_pos, 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, rb_current_pos, 0.5);
            
            jump_stage = 17;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 17：左后腿落下 ==========================
    else if (jump_stage == 17)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段17：左后腿落下");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // ✅ 阶段17完成后 → 规划阶段18轨迹
        if (t > 0.3) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            rb_leg_step.update_flight_trajectory(rb_current_pos, {0,0,0}, {0.05, 0.0, 0.12}, {0,0}, 0.5, 0.12);
            lf_leg_step.update_support_trajectory(lf_current_pos, lf_current_pos, 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, rf_current_pos, 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, lb_current_pos, 0.5);
            
            jump_stage = 18;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 18：右后腿 上 第二节台阶 ==========================
    else if (jump_stage == 18)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段18：右后腿上第二节台阶");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // ✅ 阶段18完成后 → 规划阶段19轨迹
        if (t > 0.5) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            rb_leg_step.update_flight_trajectory(rb_current_pos + Vector3D(0.05,0,0.12), {0,0,0}, {0.03, 0.0, -0.03}, {0,0}, 0.3, 0.0);
            lf_leg_step.update_support_trajectory(lf_current_pos, lf_current_pos, 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, rf_current_pos, 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, lb_current_pos, 0.5);
            
            jump_stage = 19;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 19：右后腿落下，全部完成 ==========================
    else if (jump_stage == 19)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段19：全部完成");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        if (t > 0.5) {
            RCLCPP_INFO(robot->node_->get_logger(), "✅ 双台阶攀爬完成！");
            return "stop";
        }
    }

    // ========================== 统一输出 ==========================
    joints_target.legs[0] = robot->lf_leg_calc->signal_leg_calc(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_force, &robot->lf_forward_torque, lf_wheel_vel, lf_wheel_force);
    joints_target.legs[1] = robot->rf_leg_calc->signal_leg_calc(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_force, &robot->rf_forward_torque, rf_wheel_vel, rf_wheel_force);
    joints_target.legs[2] = robot->lb_leg_calc->signal_leg_calc(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_force, &robot->lb_forward_torque, lb_wheel_vel, lb_wheel_force);
    joints_target.legs[3] = robot->rb_leg_calc->signal_leg_calc(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_force, &robot->rb_forward_torque, rb_wheel_vel, rb_wheel_force);
    
    auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
    auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
    auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
    auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

    static int cnt = 0;
    cnt++;
    if(cnt>=100)
    {
        cnt = 0;
        RCLCPP_ERROR(robot->node_->get_logger(),"\033[ lf_current_pos = (%.2f, %.2f, %.2f) rf_current_pos = (%.2f, %.2f, %.2f) lb_current_pos = (%.2f, %.2f, %.2f) rb_current_pos = (%.2f, %.2f, %.2f)\033[0m", 
        lf_current_pos[0], lf_current_pos[1], lf_current_pos[2], 
        rf_current_pos[0], rf_current_pos[1], rf_current_pos[2], 
        lb_current_pos[0], lb_current_pos[1], lb_current_pos[2], 
        rb_current_pos[0], rb_current_pos[1], rb_current_pos[2]);
    }
    robot->legs_target_pub->publish(joints_target);
    return "jump_step";
}
