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

JumpStepState::JumpStepState(Robot* robot)
    : BaseState<Robot>("jump_step")
{
    
}

bool JumpStepState::enter(Robot* robot, const std::string& last_status)
{
    jump_stage = 0;
    return true;
}

std::string JumpStepState::update(Robot* robot)
{
    robot_interfaces::msg::RobotTarget joints_target;

    // 默认力
    auto lf_force = Vector3D(0,0,-robot->robot_lf_grivate);
    auto rf_force = Vector3D(0,0,-robot->robot_rf_grivate);
    auto lb_force = Vector3D(0,0,-robot->robot_lb_grivate);
    auto rb_force = Vector3D(0,0,-robot->robot_rb_grivate);

    // ========================== 阶段 0：前进 ==========================
    if (jump_stage == 0)
    {
        RCLCPP_INFO(robot->node_->get_logger(), "正在前进");
        lf_wheel_vel = 0.3;
        rf_wheel_vel = -0.3;
        lb_wheel_vel = 0.3;
        rb_wheel_vel = -0.3;

        lf_wheel_force = 0;
        rf_wheel_force = 0;
        lb_wheel_force = 0;
        rb_wheel_force = 0;

        auto f_lf = robot->lf_leg_calc->foot_force(robot->lf_joint_pos, robot->lf_joint_torque, robot->lf_forward_torque);
        auto f_rf = robot->rf_leg_calc->foot_force(robot->rf_joint_pos, robot->rf_joint_torque, robot->rf_forward_torque);

        if (f_lf[0] > 5.0 && f_rf[0] > 5.0)
        {  
           // RCLCPP_INFO(robot->node_->get_logger(),"f_lf: (%.2f,%.2f,%.2f) ", f_lf[0], f_lf[1], f_lf[2]);
            auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            step_lf_foot_pos = lf_cart_pos;
            step_rf_foot_pos = rf_cart_pos;
            step_lb_foot_pos = lb_cart_pos;
            step_rb_foot_pos = rb_cart_pos;

            // 前腿：伸直轨迹
            lf_leg_step.update_flight_trajectory(step_lf_foot_pos, {0,0,0}, 
                {0.0, 0.0, -0.11}, {0,0}, 0.23, -0.13);
            rf_leg_step.update_flight_trajectory(step_rf_foot_pos, {0,0,0}, 
                {0.0, 0.0, -0.11}, {0,0}, 0.23, -0.13);

            // 后腿支撑
            lb_leg_step.update_support_trajectory(step_lb_foot_pos, step_lb_foot_pos, 0.23);
            rb_leg_step.update_support_trajectory(step_rb_foot_pos, step_rb_foot_pos, 0.23);

            jump_stage_time = robot->node_->get_clock()->now();
            jump_stage = 1;
        }
    }

    // ========================== 阶段 1：腿伸直 ==========================
    else if (jump_stage == 1)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "腿伸直中，时间：%.2f", t); 

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        lf_wheel_vel = 0.0;
        rf_wheel_vel = 0.0;
        lb_wheel_vel = 0.25;  
        rb_wheel_vel = -0.25;

        // 伸直完成 → 进入【迅速收腿】状态
        if (!success || t > 1.0)
        {
            auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_flight_trajectory(lf_cart_pos, {0,0,0}, 
                {0.028, 0.0, 0.062}, {0,0}, 0.30, 0.065);  // 向上快速收腿
            rf_leg_step.update_flight_trajectory(rf_cart_pos, {0,0,0}, 
                {0.028, 0.0, 0.062}, {0,0}, 0.30, 0.065);
            lb_leg_step.update_support_trajectory(lb_cart_pos, Vector3D(0,0.01,-0.03), 0.30);
            rb_leg_step.update_support_trajectory(rb_cart_pos, Vector3D(0,0.01,-0.03), 0.30);
            jump_stage_time = robot->node_->get_clock()->now();
            jump_stage = 2; // 收腿状态
        }
    }

    // ========================== 阶段 2：迅速收腿 ==========================
    else if (jump_stage == 2)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "迅速收腿，时间：%.2f", t); 
        foot_force_compen = Vector3D(50.0, 0.0,300);
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        flag = true;
        lf_force += foot_force_compen;
        rf_force += foot_force_compen;

        if (t > 0.30)
        {
            jump_stage = 3;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }
       
      // ========================== 阶段 3：等待稳定 ==========================
    else if (jump_stage == 3){
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "等待稳定：%.2f", t); 

        if(t > 1.0){
            jump_stage = 4;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }
    else if (jump_stage == 4)
{
    // ================== 轮子慢速推进 ==================
    lf_wheel_vel = 0.2;
    rf_wheel_vel = -0.2;
    lb_wheel_vel = 0.2;
    rb_wheel_vel = -0.2;

    static bool stage4_triggered = false;

    double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
    RCLCPP_INFO(robot->node_->get_logger(), "阶段4推进: %.2f", t);

    // ================== 触发预加载 ==================
    if (!stage4_triggered && t > 1.5 && flag)
    {
        stage4_triggered = true;

        auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        //  重置时间（关键）
        jump_stage_time = robot->node_->get_clock()->now();

        lf_leg_step.update_flight_trajectory(lf_cart_pos, {0,0,0}, 
                Vector3D(0.0,0,-0.074), {0,0}, 0.30, 0.075);  // 向上快速收腿
        rf_leg_step.update_flight_trajectory(rf_cart_pos, {0,0,0}, 
                Vector3D(0.0,0,-0.074), {0,0}, 0.30, 0.075);
        // 后腿稳住
        lb_leg_step.update_support_trajectory(lb_cart_pos, lb_cart_pos, 0.30);
        rb_leg_step.update_support_trajectory(rb_cart_pos, rb_cart_pos, 0.30);
    }

    // ================== 执行 ==================
    if (stage4_triggered)
    {
        bool success = false;
        double traj_t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(traj_t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(traj_t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(traj_t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(traj_t, success);


        // ================== 进入阶段5 ==================
        if (traj_t > 0.30)
        {
            jump_stage = 5;
            jump_stage_time = robot->node_->get_clock()->now();

            stage4_triggered = false;
        }
    }
}
    else if (jump_stage == 5)
{
    // ================== 轮子继续推 ==================
    lf_wheel_vel = 0.35;
    rf_wheel_vel = -0.35;
    lb_wheel_vel = 0.35;
    rb_wheel_vel = -0.35;

    static bool stage5_triggered = false;

    // ================== 只触发一次 ==================
    if (!stage5_triggered)
    {
        stage5_triggered = true;

        auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        //  重置时间
        jump_stage_time = robot->node_->get_clock()->now();

        // ================== 前腿爆发抬腿 ==================
        lf_leg_step.update_flight_trajectory(lf_cart_pos, {0,0,0}, 
            {0.11, 0.0, 0.1}, {0,0}, 0.22, 0.12);

        rf_leg_step.update_flight_trajectory(rf_cart_pos, {0,0,0}, 
            {0.11, 0.0, 0.1}, {0,0}, 0.22, 0.12);

        // 后腿支撑
        lb_leg_step.update_support_trajectory(lb_cart_pos, Vector3D(0.01,0.01,-0.082), 0.22);
        rb_leg_step.update_support_trajectory(rb_cart_pos, Vector3D(0.01,0.01,-0.082), 0.22);
    }

    // ================== 执行轨迹 ==================
    bool success = false;
    double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();

    std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
    std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
    std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
    std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

    // 力补偿
    Vector3D foot_force_compen(120.0, 0.0, 260.0);
    lf_force += foot_force_compen;
    rf_force += foot_force_compen;

    if (t > 0.20)
    {
        jump_stage = 6;
        jump_stage_time = robot->node_->get_clock()->now();

        stage5_triggered = false;
    }
}

    else if (jump_stage == 6){
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "等待稳定：%.2f", t); 

        if(t > 1.0){
            jump_stage = 7;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }


 // ========================== 阶段7：后腿蹲下（蓄力） ==========================
//     else if (jump_stage == 7)
// {
//     lf_wheel_vel = 0.15;
//     rf_wheel_vel = -0.15;
//     lb_wheel_vel = 0.15;
//     rb_wheel_vel = -0.15;

//     static bool triggered = false;

//     double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
//     RCLCPP_INFO(robot->node_->get_logger(), "阶段7 蓄力下蹲: %.2f", t);

//     if (!triggered)
//     {
//         triggered = true;

//         auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
//         auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
//         auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
//         auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

//         jump_stage_time = robot->node_->get_clock()->now();

//         // 前腿锁住
//         lf_leg_step.update_support_trajectory(lf_cart_pos, lf_cart_pos, 0.4);
//         rf_leg_step.update_support_trajectory(rf_cart_pos, rf_cart_pos, 0.4);

//         // 🔥 后腿蹲下（关键）
//         lb_leg_step.update_support_trajectory(lb_cart_pos, Vector3D(-0.02, 0.0, -0.08), 0.4);
//         rb_leg_step.update_support_trajectory(rb_cart_pos, Vector3D(-0.02, 0.0, -0.08), 0.4);
//     }

//     bool success = false;
//     double traj_t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();

//     std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(traj_t, success);
//     std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(traj_t, success);
//     std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(traj_t, success);
//     std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(traj_t, success);

//     if (traj_t > 0.4)
//     {
//         jump_stage = 70;
//         jump_stage_time = robot->node_->get_clock()->now();
//         triggered = false;
//     }
// }


// ========================== 阶段70：后腿绷直（预加载） ==========================
    else if (jump_stage == 7)
{
    lf_wheel_vel = 0.0;
    rf_wheel_vel = 0.0;
    lb_wheel_vel = 0.0;
    rb_wheel_vel = 0.0;

    static bool triggered = false;

    double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
    RCLCPP_INFO(robot->node_->get_logger(), "阶段70 后腿绷直: %.2f", t);

    if (!triggered)
    {
        triggered = true;

        auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        jump_stage_time = robot->node_->get_clock()->now();

        // 前腿锁住
        lf_leg_step.update_support_trajectory(lf_cart_pos, Vector3D(0.01,0,0.09), 0.30);
        rf_leg_step.update_support_trajectory(rf_cart_pos, Vector3D(0.01,0,0.09), 0.30);

        lb_leg_step.update_flight_trajectory(lb_cart_pos, {0,0,0}, 
            Vector3D(0.03, 0.01, -0.07), {0,0}, 0.22, -0.07);

        rb_leg_step.update_flight_trajectory(rb_cart_pos, {0,0,0}, 
            Vector3D(0.03, 0.01, -0.07), {0,0}, 0.22, -0.07);
    }

    bool success = false;
    double traj_t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();

    std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(traj_t, success);
    std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(traj_t, success);
    std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(traj_t, success);
    std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(traj_t, success);


    if (traj_t > 0.30)
    {
        jump_stage = 8;
        jump_stage_time = robot->node_->get_clock()->now();
        triggered = false;
    }
}


// ========================== 阶段8：爆发起跳 ==========================
    else if (jump_stage == 8)
{
    bool success = false;

    // ❗ 起跳时轮子必须停
    lf_wheel_vel = 0.1;
    rf_wheel_vel = 0.1;
    lb_wheel_vel = 0.1;
    rb_wheel_vel = 0.1;

    static bool triggered = false;

    double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
    RCLCPP_INFO(robot->node_->get_logger(), "阶段8 起跳爆发: %.2f", t);

    if (!triggered)
    {
        triggered = true;

        auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        jump_stage_time = robot->node_->get_clock()->now();

        // 前腿轻微抬起
        lf_leg_step.update_support_trajectory(lf_cart_pos, Vector3D(0.03,0,0.10), 0.13);
        rf_leg_step.update_support_trajectory(rf_cart_pos, Vector3D(0.03,0,0.10), 0.13);

        lb_leg_step.update_flight_trajectory(lb_cart_pos, lb_cart_pos, 
            Vector3D(0.07, -0.01, 0.02), {0,0}, 0.13, 0.02);

        rb_leg_step.update_flight_trajectory(rb_cart_pos, rb_cart_pos, 
            Vector3D(0.07, -0.01, 0.02), {0,0}, 0.13, 0.02);
        
    }

    double traj_t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();

    std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(traj_t, success);
    std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(traj_t, success);
    std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(traj_t, success);
    std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(traj_t, success);
   
    Vector3D foot_force_compen(100.0, 0.0, 260.0);
    lb_force += Vector3D(0.0, 0.0, 300.0);  
    rb_force += Vector3D(0.0, 0.0, 300.0);


    if (traj_t > 0.13)
    {
        jump_stage = 90;
        jump_stage_time = robot->node_->get_clock()->now();
        triggered = false;
    }
}

    // ========================== 阶段 9：轮子驱动（第二次循环准备）==========================
    else if (jump_stage == 9)
    {
        RCLCPP_INFO(robot->node_->get_logger(), "【阶段 9】轮子驱动，准备第二次循环");
        
        lf_wheel_vel = 0.1;
        rf_wheel_vel = -0.1;
        lb_wheel_vel = 0.1;
        rb_wheel_vel = -0.1;

        auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        lf_leg_step.update_support_trajectory(lf_cart_pos, lf_cart_pos, 0.23);
        rf_leg_step.update_support_trajectory(rf_cart_pos, rf_cart_pos, 0.23);
        lb_leg_step.update_support_trajectory(lb_cart_pos, lb_cart_pos, 0.23);
        rb_leg_step.update_support_trajectory(rb_cart_pos, rb_cart_pos, 0.23);

        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        bool success = false;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        if (t > 0.5)
        {
            jump_stage = 8;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 8：第二次循环 - 前腿第一次跳 ==========================
    else if (jump_stage == 8)
    {
        bool success = false;
        RCLCPP_INFO(robot->node_->get_logger(), "【阶段 8】第二次循环 - 前腿第一次跳");

        auto f_lf = robot->lf_leg_calc->foot_force(robot->lf_joint_pos, robot->lf_joint_torque, robot->lf_forward_torque);
        auto f_rf = robot->rf_leg_calc->foot_force(robot->rf_joint_pos, robot->rf_joint_torque, robot->rf_forward_torque);

        if (f_lf[0] > 2.0 && f_rf[0] > 2.0)
        {
            RCLCPP_INFO(robot->node_->get_logger(), "前腿已站稳，开始第二次循环前腿跳跃");
            
            auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            jump_stage_time = robot->node_->get_clock()->now();
            
            lf_leg_step.update_flight_trajectory(lf_cart_pos, {0,0,0}, 
                {0.03, 0.0, 0.15}, {0,0}, 0.8, 0.15);
            rf_leg_step.update_flight_trajectory(rf_cart_pos, {0,0,0}, 
                {0.03, 0.0, 0.15}, {0,0}, 0.8, 0.15);

            lb_leg_step.update_support_trajectory(lb_cart_pos, step_lb_foot_pos, 0.8);
            rb_leg_step.update_support_trajectory(rb_cart_pos, step_rb_foot_pos, 0.8);
        }

        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        Vector3D foot_force_compen(0.0, 0.0, 290.0);
        lf_force += foot_force_compen;
        rf_force += foot_force_compen;

        lf_wheel_vel = 0.0;
        rf_wheel_vel = 0.0;
        lb_wheel_vel = 0.3;
        rb_wheel_vel = -0.3;

        if (t > 0.8)
        {
            jump_stage = 9;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 9：第二次循环 - 轮子驱动 ==========================
    else if (jump_stage == 9)
    {
        RCLCPP_INFO(robot->node_->get_logger(), "【阶段 9】第二次循环 - 轮子驱动");
        
        lf_wheel_vel = 0.7;
        rf_wheel_vel = -0.7;
        lb_wheel_vel = 0.7;
        rb_wheel_vel = -0.7;

        auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        lf_leg_step.update_support_trajectory(lf_cart_pos, lf_cart_pos, 0.23);
        rf_leg_step.update_support_trajectory(rf_cart_pos, rf_cart_pos, 0.23);
        lb_leg_step.update_support_trajectory(lb_cart_pos, lb_cart_pos, 0.23);
        rb_leg_step.update_support_trajectory(rb_cart_pos, rb_cart_pos, 0.23);

        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        bool success = false;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        if (t > 0.5)
        {
            jump_stage = 10;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 10：第二次循环 - 前腿第二次跳 ==========================
    else if (jump_stage == 10)
    {
        bool success = false;
        RCLCPP_INFO(robot->node_->get_logger(), "【阶段 10】第二次循环 - 前腿第二次跳");

        auto f_lf = robot->lf_leg_calc->foot_force(robot->lf_joint_pos, robot->lf_joint_torque, robot->lf_forward_torque);
        auto f_rf = robot->rf_leg_calc->foot_force(robot->rf_joint_pos, robot->rf_joint_torque, robot->rf_forward_torque);

        if (f_lf[0] > 2.0 && f_rf[0] > 2.0)
        {
            RCLCPP_INFO(robot->node_->get_logger(), "✅ 前腿已站稳，开始第二次循环前腿第二次跳跃");
            
            auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            jump_stage_time = robot->node_->get_clock()->now();
            
            lf_leg_step.update_flight_trajectory(lf_cart_pos, {0,0,0}, 
                {0.03, 0.0, 0.15}, {0,0}, 0.8, 0.15);
            rf_leg_step.update_flight_trajectory(rf_cart_pos, {0,0,0}, 
                {0.03, 0.0, 0.15}, {0,0}, 0.8, 0.15);

            lb_leg_step.update_support_trajectory(lb_cart_pos, step_lb_foot_pos, 0.8);
            rb_leg_step.update_support_trajectory(rb_cart_pos, step_rb_foot_pos, 0.8);
        }

        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        Vector3D foot_force_compen(0.0, 0.0, 290.0);
        lf_force += foot_force_compen;
        rf_force += foot_force_compen;

        lf_wheel_vel = 0.0;
        rf_wheel_vel = 0.0;
        lb_wheel_vel = 0.3;
        rb_wheel_vel = -0.3;

        if (t > 0.8)
        {
            jump_stage = 11;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 11：第二次循环 - 轮子驱动准备后腿跳 ==========================
    else if (jump_stage == 11)
    {
        RCLCPP_INFO(robot->node_->get_logger(), "【阶段 10】第二次循环 - 轮子驱动准备后腿跳");
        
        lf_wheel_vel = 0.7;
        rf_wheel_vel = -0.7;
        lb_wheel_vel = 0.7;
        rb_wheel_vel = -0.7;

        auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        lf_leg_step.update_support_trajectory(lf_cart_pos, lf_cart_pos, 0.23);
        rf_leg_step.update_support_trajectory(rf_cart_pos, rf_cart_pos, 0.23);
        lb_leg_step.update_support_trajectory(lb_cart_pos, lb_cart_pos, 0.23);
        rb_leg_step.update_support_trajectory(rb_cart_pos, rb_cart_pos, 0.23);

        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        bool success = false;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        if (t > 0.5)
        {
            jump_stage = 12;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 12：第二次循环 - 后腿跳 ==========================
    else if (jump_stage == 12)
    {
        bool success = false;
        RCLCPP_INFO(robot->node_->get_logger(), "【阶段 12】第二次循环 - 后腿跳");

        auto f_lb = robot->lb_leg_calc->foot_force(robot->lb_joint_pos, robot->lb_joint_torque, robot->lb_forward_torque);
        auto f_rb = robot->rb_leg_calc->foot_force(robot->rb_joint_pos, robot->rb_joint_torque, robot->rb_forward_torque);

        if (f_lb[0] > 2.0 && f_rb[0] > 2.0)
        {
            RCLCPP_INFO(robot->node_->get_logger(), "✅ 后腿已站稳，开始第二次循环后腿跳跃");
            
            auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            jump_stage_time = robot->node_->get_clock()->now();
            
            lf_leg_step.update_support_trajectory(lf_cart_pos, step_lf_foot_pos, 0.8);
            rf_leg_step.update_support_trajectory(rf_cart_pos, step_rf_foot_pos, 0.8);

            lb_leg_step.update_flight_trajectory(lb_cart_pos, {0,0,0}, 
                {0.03, 0.0, 0.15}, {0,0}, 0.8, 0.15);
            rb_leg_step.update_flight_trajectory(rb_cart_pos, {0,0,0}, 
                {0.03, 0.0, 0.15}, {0,0}, 0.8, 0.15);
        }

        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        Vector3D foot_force_compen(0.0, 0.0, 290.0);
        lb_force += foot_force_compen;
        rb_force += foot_force_compen;

        lf_wheel_vel = 0.0;
        rf_wheel_vel = 0.0;
        lb_wheel_vel = 0.0;
        rb_wheel_vel = 0.0;

        if (t > 0.8)
        {
            jump_stage = 12;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 12：最后阶段 - 轮子持续驱动 ==========================
    else if (jump_stage == 12)
    {
        RCLCPP_INFO(robot->node_->get_logger(), "【阶段 12】最后阶段 - 轮子持续驱动");
        
        lf_wheel_vel = 0.7;
        rf_wheel_vel = -0.7;
        lb_wheel_vel = 0.7;
        rb_wheel_vel = -0.7;

        auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        lf_leg_step.update_support_trajectory(lf_cart_pos, lf_cart_pos, 0.23);
        rf_leg_step.update_support_trajectory(rf_cart_pos, rf_cart_pos, 0.23);
        lb_leg_step.update_support_trajectory(lb_cart_pos, lb_cart_pos, 0.23);
        rb_leg_step.update_support_trajectory(rb_cart_pos, rb_cart_pos, 0.23);

        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        bool success = false;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        if (t > 0.6)
        {
            jump_stage = 13;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 13：最后阶段 - 后腿跳第一次 ==========================
    else if (jump_stage == 13)
    {
        bool success = false;
        RCLCPP_INFO(robot->node_->get_logger(), "【阶段 13】最后阶段 - 后腿跳第一次");

        auto f_lb = robot->lb_leg_calc->foot_force(robot->lb_joint_pos, robot->lb_joint_torque, robot->lb_forward_torque);
        auto f_rb = robot->rb_leg_calc->foot_force(robot->rb_joint_pos, robot->rb_joint_torque, robot->rb_forward_torque);

        if (f_lb[0] > 2.0 && f_rb[0] > 2.0)
        {
            RCLCPP_INFO(robot->node_->get_logger(), "✅ 后腿已站稳，开始最后第一次后腿跳跃");
            
            auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            jump_stage_time = robot->node_->get_clock()->now();
            
            lf_leg_step.update_support_trajectory(lf_cart_pos, step_lf_foot_pos, 0.8);
            rf_leg_step.update_support_trajectory(rf_cart_pos, step_rf_foot_pos, 0.8);

            lb_leg_step.update_flight_trajectory(lb_cart_pos, {0,0,0}, 
                {0.03, 0.0, 0.15}, {0,0}, 0.8, 0.15);
            rb_leg_step.update_flight_trajectory(rb_cart_pos, {0,0,0}, 
                {0.03, 0.0, 0.15}, {0,0}, 0.8, 0.15);
        }

        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        Vector3D foot_force_compen(0.0, 0.0, 290.0);
        lb_force += foot_force_compen;
        rb_force += foot_force_compen;

        lf_wheel_vel = 0.0;
        rf_wheel_vel = 0.0;
        lb_wheel_vel = 0.0;
        rb_wheel_vel = 0.0;

        if (t > 0.8)
        {
            jump_stage = 14;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 14：最后阶段 - 轮子驱动准备最后一次跳 ==========================
    else if (jump_stage == 14)
    {
        RCLCPP_INFO(robot->node_->get_logger(), "【阶段 14】最后阶段 - 轮子驱动准备最后一次跳");
        
        lf_wheel_vel = 0.7;
        rf_wheel_vel = -0.7;
        lb_wheel_vel = 0.7;
        rb_wheel_vel = -0.7;

        auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        lf_leg_step.update_support_trajectory(lf_cart_pos, lf_cart_pos, 0.23);
        rf_leg_step.update_support_trajectory(rf_cart_pos, rf_cart_pos, 0.23);
        lb_leg_step.update_support_trajectory(lb_cart_pos, lb_cart_pos, 0.23);
        rb_leg_step.update_support_trajectory(rb_cart_pos, rb_cart_pos, 0.23);

        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        bool success = false;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        if (t > 0.5)
        {
            jump_stage = 15;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 15：最后阶段 - 后腿跳第二次（完毕）==========================
    else if (jump_stage == 15)
    {
        bool success = false;
        RCLCPP_INFO(robot->node_->get_logger(), "【阶段 15】最后阶段 - 后腿跳第二次，完毕");

        auto f_lb = robot->lb_leg_calc->foot_force(robot->lb_joint_pos, robot->lb_joint_torque, robot->lb_forward_torque);
        auto f_rb = robot->rb_leg_calc->foot_force(robot->rb_joint_pos, robot->rb_joint_torque, robot->rb_forward_torque);

        if (f_lb[0] > 2.0 && f_rb[0] > 2.0)
        {
            RCLCPP_INFO(robot->node_->get_logger(), "✅ 后腿已站稳，开始最后第二次后腿跳跃");
            
            auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            jump_stage_time = robot->node_->get_clock()->now();
            
            lf_leg_step.update_support_trajectory(lf_cart_pos, step_lf_foot_pos, 0.8);
            rf_leg_step.update_support_trajectory(rf_cart_pos, step_rf_foot_pos, 0.8);

            lb_leg_step.update_flight_trajectory(lb_cart_pos, {0,0,0}, 
                {0.03, 0.0, 0.15}, {0,0}, 0.8, 0.15);
            rb_leg_step.update_flight_trajectory(rb_cart_pos, {0,0,0}, 
                {0.03, 0.0, 0.15}, {0,0}, 0.8, 0.15);
        }

        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        Vector3D foot_force_compen(0.0, 0.0, 290.0);
        lb_force += foot_force_compen;
        rb_force += foot_force_compen;

        lf_wheel_vel = 0.0;
        rf_wheel_vel = 0.0;
        lb_wheel_vel = 0.0;
        rb_wheel_vel = 0.0;

        // 完毕 → 返回 stop 状态
        if (t > 0.8)
        {
            RCLCPP_INFO(robot->node_->get_logger(), "🎉 整个流程完毕！");
            return "stop";
        }
    }

    // ========================== 统一输出 ==========================
    joints_target.legs[0] = robot->lf_leg_calc->signal_leg_calc(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_force, &robot->lf_forward_torque, lf_wheel_vel, lf_wheel_force);
    joints_target.legs[1] = robot->rf_leg_calc->signal_leg_calc(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_force, &robot->rf_forward_torque, rf_wheel_vel, rf_wheel_force);
    joints_target.legs[2] = robot->lb_leg_calc->signal_leg_calc(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_force, &robot->lb_forward_torque, lb_wheel_vel, lb_wheel_force);
    joints_target.legs[3] = robot->rb_leg_calc->signal_leg_calc(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_force, &robot->rb_forward_torque, rb_wheel_vel, rb_wheel_force);

    static int cnt = 0;
    cnt++;
    if(cnt>=100)
    {
        cnt = 0;
        RCLCPP_ERROR(robot->node_->get_logger(),
            "\033[31mlf_foot_exp_pos = (%.2f, %.2f, %.2f) rf_foot_exp_pos = (%.2f, %.2f, %.2f) lb_foot_exp_pos = (%.2f, %.2f, %.2f) rb_foot_exp_pos = (%.2f, %.2f, %.2f)\033[0m",
            lf_foot_exp_pos.x(), lf_foot_exp_pos.y(), lf_foot_exp_pos.z(), rf_foot_exp_pos.x(), rf_foot_exp_pos.y(), rf_foot_exp_pos.z(), lb_foot_exp_pos.x(), lb_foot_exp_pos.y(), lb_foot_exp_pos.z(), rb_foot_exp_pos.x(), rb_foot_exp_pos.y(), rb_foot_exp_pos.z());
    }

    robot->legs_target_pub->publish(joints_target);
    return "jump_step";
}