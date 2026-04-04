#include "states/new_cross_step.hpp"
#include "core/robot.hpp"
#include "leg/step.hpp"
#include <cmath>
#include <rcl/timer.h>
#include <rclcpp/logging.hpp>
#include <math.h>


JumpStepState::JumpStepState(Robot* robot)
    : BaseState<Robot>("jump_step")
{
      
    F.lb_force = Vector3D::Zero();
    F.lf_force = Vector3D::Zero();
    F.rf_force = Vector3D::Zero();
    F.rb_force = Vector3D::Zero();
    
    // 计算机器人总质量和质心位置（防御性编程）
    double total_gravity = robot->robot_lf_grivate + robot->robot_rf_grivate + 
                          robot->robot_lb_grivate + robot->robot_rb_grivate;
    
    if (total_gravity > 1e-6) {
        mass = total_gravity / 9.8;

        if (robot->lf_leg_calc && robot->rf_leg_calc && 
            robot->lb_leg_calc && robot->rb_leg_calc) {
            
            mass_center_pos = Vector2D(
                robot->robot_lf_grivate * robot->lf_leg_calc->pos_offset[0] +
                robot->robot_rf_grivate * robot->rf_leg_calc->pos_offset[0] +
                robot->robot_lb_grivate * robot->lb_leg_calc->pos_offset[0] +
                robot->robot_rb_grivate * robot->rb_leg_calc->pos_offset[0],

                robot->robot_lf_grivate * robot->lf_leg_calc->pos_offset[1] +
                robot->robot_rf_grivate * robot->rf_leg_calc->pos_offset[1] +
                robot->robot_lb_grivate * robot->lb_leg_calc->pos_offset[1] +
                robot->robot_rb_grivate * robot->rb_leg_calc->pos_offset[1]
            ) / total_gravity;
        } else {
            mass = 0;
            mass_center_pos = Vector2D::Zero();
        }
    } else {
        mass = 0;
        mass_center_pos = Vector2D::Zero();
    }
}

// void JumpStepState::bodyShiftForRearLegClimb(
//     Robot* robot,
//     int& shift_step,
//     rclcpp::Time& shift_start_time,
//     double shift_y
// )
// {
//     if (!robot || !robot->node_) return;

//     auto lf_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
//     auto rf_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
//     auto lb_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
//     auto rb_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

//     // ===================== 步骤0：初始化偏移轨迹 =====================
//     if (shift_step == 0)
//     {
//         RCLCPP_INFO(robot->node_->get_logger(), "[质心偏移] 开始侧移身体 Y = %.2f", shift_y);

        
//         lf_leg_step.update_support_trajectory(lf_pos,  Vector3D(lf_pos[0], -shift_y, lf_pos[2]), 0.6);
//         rf_leg_step.update_support_trajectory(rf_pos,  Vector3D(rf_pos[0], -shift_y, rf_pos[2]), 0.6);
//         lb_leg_step.update_support_trajectory(lb_pos,  Vector3D(lb_pos[0], -shift_y, lb_pos[2]), 0.6);
//         rb_leg_step.update_support_trajectory(rb_pos,  Vector3D(rb_pos[0], -shift_y, rb_pos[2]), 0.6);

//         shift_start_time = robot->node_->get_clock()->now();
//         shift_step = 1;
//     }

//     // ===================== 步骤1：执行偏移轨迹 =====================
//     else if (shift_step == 1)
//     {
//         bool success = false;
//         double t = (robot->node_->get_clock()->now() - shift_start_time).seconds();

//         std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
//         std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
//         std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
//         std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

//         computeFootForces(&F, robot, mass, mass_center_pos,
//             lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
//             true, true, true, true);
//         if (!success)
//         {
//             shift_step = 2;
//             RCLCPP_INFO(robot->node_->get_logger(), "[质心偏移] 完成！");
//         }
//     }
// }


bool JumpStepState::enter(Robot* robot, const std::string& last_status)
{
    (void)last_status;
    jump_stage = 0;
    jump_stage_time = robot->node_->get_clock()->now();
    return true;
}

std::string JumpStepState::update(Robot* robot)
{
   
    F.lf_force = Vector3D(0, 0, -robot->robot_lf_grivate);
    F.rf_force = Vector3D(0, 0, -robot->robot_rf_grivate);
    F.lb_force = Vector3D(0, 0, -robot->robot_lb_grivate);
    F.rb_force = Vector3D(0, 0, -robot->robot_rb_grivate);

    if (jump_stage == 0)
    {
        RCLCPP_INFO(robot->node_->get_logger(), "阶段 0：靠近台阶");
        lf_wheel_vel =  0.3;
        rf_wheel_vel = -0.3;
        lb_wheel_vel =  0.3;
        rb_wheel_vel = -0.3;

        lf_wheel_force = 0;
        rf_wheel_force = 0;
        lb_wheel_force = 0;
        rb_wheel_force = 0;

        // 检测前足接触台阶
        auto f_lf = robot->lf_leg_calc->foot_force(robot->lf_joint_pos, robot->lf_joint_torque, robot->lf_forward_torque);
        auto f_rf = robot->rf_leg_calc->foot_force(robot->rf_joint_pos, robot->rf_joint_torque, robot->rf_forward_torque);

        if (f_lf[0] > 9.0 && f_rf[0] > 5.0)
        {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.03,0.08,0.20), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(-0.01,0,0.06), 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, lb_current_pos, 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(-0.02,-0.01,0.07), 0.5);

          
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

    std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
    std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
    std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
    std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
   computeFootForces(&F, robot, mass, mass_center_pos,
                lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
                false, true, true, true);
    lf_wheel_vel = 0;
    rf_wheel_vel = 0;
    lb_wheel_vel = 0;
    rb_wheel_vel = 0;

    if (success==false)
    {
        auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

        lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.18,0.04,0.14), 0.5);
        rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.00, 0.01, -0.01), 0.5);
        lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.02, 0.00, 0.00), 0.5);
        rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.05, -0.03, 0.05), 0.5);
        
        jump_stage = 2;
        jump_stage_time = robot->node_->get_clock()->now();
    }
}

    // ========================== 阶段 2：左前腿落下 ==========================
    else if (jump_stage == 2)
    {  
    lf_wheel_vel = 0.1;
    rf_wheel_vel = -0.1;
    lb_wheel_vel = 0.1;
    rb_wheel_vel = -0.1;
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段2：左前腿落下");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
          computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            false, true, true, true);
        if (success==false) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
       
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.14,0.01,0.11), 0.5);
            rf_leg_step.update_flight_trajectory(rf_current_pos, {0,0,0}, 
                {0.03,-0.10,0.25}, {0,0}, 0.5, 0.25);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.01, 0.01, 0.07), 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.01, -0.01, 0.07), 0.5);
            jump_stage = 3;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    else if (jump_stage == 3)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段3：抬起右前腿");
    
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, false, true, true);

        if (success==false) { 
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            rf_leg_step.update_flight_trajectory(rf_current_pos , {0,0,0},
                {0.13, 0,0.18}, {0,0}, 0.5, 0.20);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.12,0,0.12), 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.01, 0.01, 0.07), 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.02, -0.01, 0.06), 0.5);
           
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
         computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, false, true, true);
        if (success==false) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(-0.01, 0.00, 0.01), 1.0);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(-0.01, -0.00, 0.01), 1.0);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.02, 0.022, 0.03), 1.0);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.02, -0.022, 0.03), 1.0);
           
            jump_stage = 5;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 5：身体推进 ==========================
    else if (jump_stage == 5)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段5：身体推进");
        lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F,robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, true, true);
        if (success==false) {
           
            jump_stage = 6;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    else if (jump_stage ==6) {
     lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;
     double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
     if(t>3.0){
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.10,0.07,0.25), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0,0,0.06), 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.02, 0.022, 0.03), 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0,0,0.06), 0.5);
             
            jump_stage=7;
            jump_stage_time = robot->node_->get_clock()->now();
    }
    }

    else if (jump_stage == 7)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段6：前腿上第二节台阶-抬起左前腿");
       lf_wheel_vel = 0; rf_wheel_vel = -0; lb_wheel_vel = 0; rb_wheel_vel = -0;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            false, true, true, true);
        if (!success) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

       lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.18,0.02,0.17), 0.5);
       rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(-0.01,0.0,0.06), 0.5);
       lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(-0.01,0.0, 0.0), 0.5);
       rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(-0.01,0.0, 0.06), 0.5);
            
            jump_stage = 8;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 7：左前腿落在第二节台阶 ==========================
    else if (jump_stage == 8)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段7：左前腿落在第二节台阶");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
         computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            false, true, true, true);
        if (success==false) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.18,0.02,0.15), 0.5);
            rf_leg_step.update_flight_trajectory(rf_current_pos, {0,0,0}, 
                {0.13,-0.12,0.30}, {0,0}, 0.5, 0.30);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.03, 0.01, 0.06), 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.03, -0.01, 0.06), 0.5);
           
            jump_stage = 9;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 8：抬起右前腿 ==========================
    else if (jump_stage == 9)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段8：抬起右前腿");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, false, true, true);
        if (success==false) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            rf_leg_step.update_flight_trajectory(rf_current_pos , {0,0,0},
                {0.20, -0.05,0.20}, {0,0}, 0.5, 0.20);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.18,0.02,0.15), 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.01, 0.01, 0.06), 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.01, -0.01, 0.06), 0.5);
           
            jump_stage = 10;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 9：右前腿落在第二节台阶 ==========================
    else if (jump_stage == 10)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段9：右前腿落在第二节台阶");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, false, true, true);
        if (success==false) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.01, 0.00, 0.07), 1.0);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.01, -0.01, 0.07), 1.0);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.00, -0.01, -0.05), 1.0);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.00, 0.01, -0.05), 1.0);
             
            // lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.0,0.0,0.06), 0.5);
            // rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.03,-0.0,0.12), 0.5);
            // lb_leg_step.update_flight_trajectory(lb_current_pos, {0,0,0},
            //      {0.02, 0.03, -0.09}, {0,0}, 0.5, -0.09);
            // rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(-0.07,0.05,-0.09), 0.5);
            jump_stage = 11;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 10：整体推进==========================
    else if (jump_stage == 11)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段11：整体推进");
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, true, true);
        if (success==false ) {
        if(t>3)
        {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.0,0.0,0.06), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.03,-0.0,0.12), 0.5);
            lb_leg_step.update_flight_trajectory(lb_current_pos, {0,0,0},
                 {0.02, 0.03, -0.08}, {0,0}, 0.5, -0.09);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(-0.07,0.05,-0.09), 0.5);
           
            jump_stage = 12;
            jump_stage_time = robot->node_->get_clock()->now();
        }
        }
    }
    // ========================== 阶段 12：后腿上第一节台阶-抬起左后腿 ==========================
    else if (jump_stage == 12)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段12：质心偏移");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, false, true);
        if (success==false ) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.0,0.0,0.06), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.03,-0.0,0.12), 0.5);
            lb_leg_step.update_flight_trajectory(lb_current_pos, {0,0,0},
                 {0.03, 0.06, 0.05}, {0,0}, 0.5, 0.07);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(-0.07,0.05,-0.09), 0.5);
            
            jump_stage = 13;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }
    // ========================== 阶段 12：左后腿落在第一节台阶 ==========================
    else if (jump_stage == 13)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段 12：左后腿落在第一节台阶 ");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        if (success==false ) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            rb_leg_step.update_flight_trajectory(rb_current_pos, {0,0,0}, 
                {0.00, 0.03, 0.03}, {0,0}, 0.5, 0.07);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(-0.05,0.08,0.15), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.02,0.01,0.14), 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.04, 0.08, 0.03), 0.5);
            
            jump_stage = 14; 
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    // ========================== 阶段 13：抬起右后腿（质心偏移） ==========================
    else if (jump_stage == 14)
    {

        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段13：抬起右后腿（质心偏移）");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, true, false);

        if (!success ) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.01, 0.00, 0.01), 1);
            rf_leg_step.update_support_trajectory(rf_current_pos,Vector3D(0.01, -0.00, 0.01), 1);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.03, 0.02, 0.03), 1);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.00, -0.02, 0.03), 1);
            jump_stage = 15;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    else if (jump_stage == 15)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段14：回正姿态");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, true, true);
        if (success == false) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.10,0.07,0.25), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0,0,0.06), 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.02, 0.022, 0.03), 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0,0,0.06), 0.5);

            jump_stage = 16;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    else if (jump_stage == 16)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段17：前腿上第三节台阶-抬起左前腿");
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            false, true, true, true);
        lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;
        if (success == false) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

       lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.18,0.02,0.17), 0.5);
       rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(-0.01,0.0,0.06), 0.5);
       lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(-0.01,0.0, 0.0), 0.5);
       rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(-0.01,0.0, 0.06), 0.5);
            
            jump_stage = 17;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    else if (jump_stage == 17)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "左前腿落在第三节台阶");
       lf_wheel_vel = 0; rf_wheel_vel = -0; lb_wheel_vel = 0; rb_wheel_vel = -0;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            false, true, true, true);
        if (!success) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

           lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.18,0.02,0.15), 0.5);
            rf_leg_step.update_flight_trajectory(rf_current_pos, {0,0,0}, 
                {0.13,-0.12,0.30}, {0,0}, 0.5, 0.30);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.03, 0.01, 0.06), 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.03, -0.01, 0.06), 0.5);
            
            jump_stage = 18;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    
    else if (jump_stage == 18)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段18：第三节台阶-抬起右前腿");
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
         computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, false, true, true);
        if (success==false) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            rf_leg_step.update_flight_trajectory(rf_current_pos , {0,0,0},
                {0.20, -0.05,0.20}, {0,0}, 0.5, 0.20);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.18,0.02,0.15), 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.01, 0.01, 0.06), 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.01, -0.01, 0.06), 0.5);
           
            jump_stage = 19;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    
    else if (jump_stage == 19)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段19：右前腿落在第三节台阶");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, false, true, true);
        if (success==false) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.01, 0.00, 0.07), 1.0);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.01, -0.01, 0.07), 1.0);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.00, -0.01, -0.05), 1.0);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.00, 0.01, -0.05), 1.0);
            jump_stage = 20;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

   
    else if (jump_stage == 20)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段20：整体推进");
        lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, true, true);
        if (success==false) {
            if(t>3.0){
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.0,0.0,0.06), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.03,-0.02,0.12), 0.5);
            lb_leg_step.update_flight_trajectory(lb_current_pos, {0,0,0},
                 {0.025, 0.03, -0.06}, {0,0}, 0.5, -0.08);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(-0.07,0.05,-0.09), 0.5);
           
            jump_stage = 21;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }
    }

    else if (jump_stage == 21)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段21：质心偏移");
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, false, true);
        if (success==false ) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.0,0.0,0.06), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.03,-0.0,0.12), 0.5);
            lb_leg_step.update_flight_trajectory(lb_current_pos, {0,0,0},
                 {0.06, 0.02, 0.03}, {0,0}, 0.5, 0.03);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(-0.07,0.05,-0.09), 0.5);
            
            jump_stage = 22;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }
    else if (jump_stage == 22)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段22：左后腿落在第二节台阶");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, false, true);
        if (success==false ) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            
            rb_leg_step.update_flight_trajectory(rb_current_pos, {0,0,0}, 
                {0.05, 0.01, 0.04}, {0,0}, 0.5, 0.08);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.02,0.08,0.15), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.02,0.01,0.14), 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.04, 0.08, 0.03), 0.5);
            
            jump_stage = 23;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }
  
    else if (jump_stage == 23)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段 23： 抬起右后腿（质心偏移");
       lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, true, false);
        if (success==false ) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.01, 0.00, 0.01), 1);
            rf_leg_step.update_support_trajectory(rf_current_pos,Vector3D(0.01, -0.00, 0.01), 1);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.03, 0.02, 0.03), 1);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.00, -0.02, 0.03), 1);
            jump_stage = 24; 
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    else if (jump_stage == 24)
    {

        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段24：回正姿态");
        lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, true, true);
        if (!success ) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(-0.01, 0.00, 0.01), 1.0);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(-0.01, -0.00, 0.01), 1.0);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.02, 0.022, 0.03), 1.0);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.02, -0.022, 0.03), 1.0);
           
            jump_stage = 25;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }
    else if (jump_stage == 25)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段25：身体推进");
        lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F,robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, true, true);
        if (success==false) {
           
            jump_stage = 26;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    else if (jump_stage ==26) {
     lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;
     double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
     if(t>3.0){
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.10,0.07,0.25), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0,0,0.06), 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.02, 0.022, 0.03), 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0,0,0.06), 0.5);
             
            jump_stage=27;
            jump_stage_time = robot->node_->get_clock()->now();
    }
    }

    else if (jump_stage == 27)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段27：前腿上第四节台阶-抬起左前腿");
       lf_wheel_vel = 0; rf_wheel_vel = -0; lb_wheel_vel = 0; rb_wheel_vel = -0;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            false, true, true, true);
        if (!success) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

       lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.18,0.02,0.17), 0.5);
       rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(-0.01,0.0,0.06), 0.5);
       lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(-0.01,0.0, 0.0), 0.5);
       rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(-0.01,0.0, 0.06), 0.5);
            
            jump_stage = 28;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    else if ( jump_stage == 28) 
   
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段28：左前腿落在第四节台阶");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
         computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            false, true, true, true);
        if (success==false) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.18,0.02,0.15), 0.5);
            rf_leg_step.update_flight_trajectory(rf_current_pos, {0,0,0}, 
                {0.13,-0.12,0.30}, {0,0}, 0.5, 0.30);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.03, 0.01, 0.06), 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.03, -0.01, 0.06), 0.5);
           
            jump_stage = 29;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    else if (jump_stage == 29)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段29：抬起右前腿");
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, false, true, true);
        if (success==false) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            rf_leg_step.update_flight_trajectory(rf_current_pos , {0,0,0},
                {0.20, -0.05,0.20}, {0,0}, 0.5, 0.20);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.18,0.02,0.15), 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.01, 0.01, 0.06), 0.5);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.01, -0.01, 0.06), 0.5);
           
            jump_stage = 30;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    else if (jump_stage == 30)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段30：右前腿落在第二节台阶");
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, false, true, true);
        if (success==false) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.01, 0.00, 0.07), 1.0);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.01, -0.01, 0.07), 1.0);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.00, -0.01, -0.05), 1.0);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.00, 0.01, -0.05), 1.0);
             
            jump_stage = 31;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    else if (jump_stage == 31)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段31：整体推进");
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, true, true);
        if (success==false ) {
        if(t>3)
        {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.0,0.0,0.06), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.03,-0.0,0.12), 0.5);
            lb_leg_step.update_flight_trajectory(lb_current_pos, {0,0,0},
                 {0.02, 0.03, -0.08}, {0,0}, 0.5, -0.09);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(-0.07,0.05,-0.09), 0.5);
           
            jump_stage = 32;
            jump_stage_time = robot->node_->get_clock()->now();
        }
        }
    }
    
    else if (jump_stage == 32)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段32：质心偏移");
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, false, true);
        if (success==false ) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.0,0.0,0.06), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.03,-0.0,0.12), 0.5);
            lb_leg_step.update_flight_trajectory(lb_current_pos, {0,0,0},
                 {0.04, 0.06, 0.045}, {0,0}, 0.5, 0.065);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(-0.07,0.05,-0.09), 0.5);
            
            jump_stage = 33;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }
    
    else if (jump_stage == 33)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段 33：左后腿落在第san节台阶 ");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        if (success==false ) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
        
            rb_leg_step.update_flight_trajectory(rb_current_pos, {0,0,0}, 
                {0.00, 0.03, 0.03}, {0,0}, 0.5, 0.07);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(-0.05,0.08,0.15), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.02,0.01,0.14), 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.04, 0.08, 0.03), 0.5);
            
            jump_stage = 34; 
            jump_stage_time = robot->node_->get_clock()->now();

    }
    }
    else if (jump_stage == 34)
    {

        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段34：抬起右后腿（质心偏移）");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, true, false);

        if (!success ) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.01, 0.00, 0.01), 1);
            rf_leg_step.update_support_trajectory(rf_current_pos,Vector3D(0.01, -0.00, 0.01), 1);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.03, 0.02, 0.03), 1);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.00, -0.02, 0.03), 1);
            jump_stage = 35;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    else if (jump_stage == 35)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段35：回正姿态");

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, true, true);
        if (success == false) {
            if(t>3.0){
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.0,0.0,0.04), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.03,-0.0,0.08), 0.5);
            lb_leg_step.update_flight_trajectory(lb_current_pos, {0,0,0},
                 {0.02, 0.06, -0.06}, {0,0}, 0.5, -0.06);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(-0.07,0.07,-0.05), 0.5);
           
            jump_stage = 36;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }
}
    else if (jump_stage == 36)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段36：质心偏移");
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, false, true);
        if (success==false ) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
//             lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.0,0.0,0.06), 0.5);
//             rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.03,-0.0,0.12), 0.5);
//             lb_leg_step.update_flight_trajectory(lb_current_pos, {0,0,0},
//                  {0.04, 0.06, 0.045}, {0,0}, 0.5, 0.065);
//             rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(-0.07,0.05,-0.09), 0.5);
            
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.03,0.0,0.04), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.03,-0.0,0.08), 0.5);
            lb_leg_step.update_flight_trajectory(lb_current_pos, {0,0,0},
                 {0.07, 0.04, 0.01}, {0,0}, 0.5, 0.02);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(-0.06,-0.02,-0.05), 0.5);
           
            jump_stage = 37;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }
    
    else if (jump_stage == 37)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段 37：左后腿落在第si节台阶 ");
         lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, false, true);
        if (success==false ) {
            jump_stage = 38; 
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }
    else if(jump_stage == 38){
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
        
            rb_leg_step.update_flight_trajectory(rb_current_pos, {0,0,0}, 
                {-0.05, -0.05, -0.04}, {0,0}, 0.5, 0.05);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.07,-0.05,0.06), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.07,-0.05,0.06), 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.07, -0.05, 0.06), 0.5);
            jump_stage_time = robot->node_->get_clock()->now();
            jump_stage = 39;
    }
    else if (jump_stage == 39)
    {

        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段39：抬起右后腿（质心偏移）");
        lf_wheel_vel = 0.1; rf_wheel_vel = -0.1; lb_wheel_vel = 0.1; rb_wheel_vel = -0.1;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, true, false);

        if (!success ) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
            rb_leg_step.update_flight_trajectory(rb_current_pos, {0,0,0}, 
                {-0.03, -0.05, 0.05}, {0,0}, 0.5, 0.06);
            // rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.0, -0.08, 0.08), 0.5);
            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.03,-0.05,-0.01), 0.5);
            rf_leg_step.update_support_trajectory(rf_current_pos, Vector3D(0.02,-0.05,-0.02), 0.5);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.05, -0.05, -0.00), 0.5);
            jump_stage = 40;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }

    else if (jump_stage == 40)
    {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段40：YOU");
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        computeFootForces(&F, robot, mass, mass_center_pos,
            lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos,
            true, true, true, false);
        if (success == false) {
            auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_leg_step.update_support_trajectory(lf_current_pos, Vector3D(0.01, 0.00, 0.01), 1);
            rf_leg_step.update_support_trajectory(rf_current_pos,Vector3D(0.01, -0.00, 0.01), 1);
            lb_leg_step.update_support_trajectory(lb_current_pos, Vector3D(0.03, 0.02, 0.03), 1);
            rb_leg_step.update_support_trajectory(rb_current_pos, Vector3D(0.00, -0.02, 0.03), 1);
            
            jump_stage = 41;
            jump_stage_time = robot->node_->get_clock()->now();
        }
    }
    else if (jump_stage == 41) 
   {
        bool success = false;
        double t = (robot->node_->get_clock()->now() - jump_stage_time).seconds();
        RCLCPP_INFO(robot->node_->get_logger(), "阶段41：全部完成");
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        if ( success == false) {
            RCLCPP_INFO(robot->node_->get_logger(), "双台阶攀爬完成！");
            return "idel";
        }
    }

    // ========================== 统一输出 ==========================
    joints_target.legs[0] = robot->lf_leg_calc->signal_leg_calc(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc,F.lf_force, &robot->lf_forward_torque, lf_wheel_vel, lf_wheel_force);
    joints_target.legs[1] = robot->rf_leg_calc->signal_leg_calc(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc,F.rf_force, &robot->rf_forward_torque, rf_wheel_vel, rf_wheel_force);
    joints_target.legs[2] = robot->lb_leg_calc->signal_leg_calc(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc,F.lb_force, &robot->lb_forward_torque, lb_wheel_vel, lb_wheel_force);
    joints_target.legs[3] = robot->rb_leg_calc->signal_leg_calc(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc,F.rb_force, &robot->rb_forward_torque, rb_wheel_vel, rb_wheel_force);
    
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