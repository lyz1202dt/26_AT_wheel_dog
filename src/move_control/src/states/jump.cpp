#include "states/jump.hpp"
#include "core/robot.hpp"
#include <robot_interfaces/msg/robot.hpp>

JumpState::JumpState(Robot* robot)
    : BaseState<Robot>("jump") {
    this->robot  = robot;
    jump_cmd_sub = robot->node_->create_subscription<robot_interfaces::msg::JumpCmd>(
        "jump_cmd", 10, [this](const robot_interfaces::msg::JumpCmd& msg) {
            RCLCPP_INFO(this->robot->node_->get_logger(), "执行跳跃动作");
            if ((this->robot->node_->get_clock()->now() - jump_cmd.stamp).seconds()
                > 0.3) {  // 如果跳跃命令过期了（超过0.3s），就忽略这个命令{
                RCLCPP_WARN(this->robot->node_->get_logger(), "跳跃命令过期，忽略");
                return;
            }
            jump_cmd = msg;
            stage    = 1; // 切换到跳跃阶段
        });
}

bool JumpState::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    la      = 0.0;
    lb      = 0.0;
    lc      = 0.0;
    ver_acc = 0.0;
    ver_vel = 0.0;
    ver_pos = 0.0;

    stage           = 0;
    current_exp_vel = current_body_vel =
        (robot->lf_wheel_omega - robot->rf_wheel_omega + robot->lb_wheel_omega - robot->rb_wheel_omega) * 0.25 * robot->WHEEL_RADIUS;
    return true;
}

std::string JumpState::update(Robot* robot) {

    Vector3D lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos;
    Vector3D lf_foot_exp_force = Vector3D::Zero(), rf_foot_exp_force = Vector3D::Zero(), lb_foot_exp_force = Vector3D::Zero(),
             rb_foot_exp_force = Vector3D::Zero();
    Vector3D lf_foot_exp_vel = Vector3D::Zero(), rf_foot_exp_vel = Vector3D::Zero(), lb_foot_exp_vel = Vector3D::Zero(),
             rb_foot_exp_vel = Vector3D::Zero();
    Vector3D lf_foot_exp_acc = Vector3D::Zero(), rf_foot_exp_acc = Vector3D::Zero(), lb_foot_exp_acc = Vector3D::Zero(),
             rb_foot_exp_acc = Vector3D::Zero();

    double lf_wheel_vel = 0.0f, rf_wheel_vel = 0.0f, lb_wheel_vel = 0.0f, rb_wheel_vel = 0.0f;
    double lf_wheel_force = 0.0f, rf_wheel_force = 0.0f, lb_wheel_force = 0.0f, rb_wheel_force = 0.0f;

    if (stage == 0) {
        if (robot->move_cmd.step_mode == 6) {
            double acc                    = exp_vel_kp * (robot->move_cmd.vx - current_body_vel); // 在状态6下只关注x方向速度
            current_exp_vel               = current_exp_vel + acc * 0.004;
            double current_exp_foot_force = robot->robot_mass * acc * 0.25;

            lf_wheel_force = current_exp_foot_force;
            rf_wheel_force = -current_exp_foot_force;
            lb_wheel_force = current_exp_foot_force;
            rb_wheel_force = -current_exp_foot_force;
        }
    }
    if (stage == 1)  // 进入执行跳跃动作阶段，现在进入准备状态，压低身体质心
    {
        lf_wheel_vel = current_exp_vel;
        rf_wheel_vel = -current_exp_vel;
        lb_wheel_vel = current_exp_vel;
        rb_wheel_vel = -current_exp_vel;

        action_start_time = robot->node_->get_clock()->now();
        ver_acc           = 2.0 * (jump_cmd.ready_jump_height - robot->body_height) / (jump_cmd.t1 * jump_cmd.t1 / 4.0);
        la                = ver_acc * 0.5;
        lb                = 0.0;
        lc                = robot->body_height;

        stage = 2;
    }
    if (stage == 2) {
        double t          = (robot->node_->get_clock()->now() - action_start_time).seconds();
        ver_vel           = 2.0 * la * t + lb;
        ver_pos           = la * t * t * 0.5 + lb * t + lc;
        lf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lf_grivate * (1.0 + ver_acc / 9.8));
        rf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rf_grivate * (1.0 + ver_acc / 9.8));
        lb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lb_grivate * (1.0 + ver_acc / 9.8));
        rb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rb_grivate * (1.0 + ver_acc / 9.8));

        if (t > jump_cmd.t1 / 2.0)
            stage = 3;
    }
    if (stage == 3) {
        ver_acc           = -ver_acc;
        la                = ver_acc * 0.5;
        lb                = ver_vel;
        lc                = (jump_cmd.ready_jump_height - robot->body_height) * 0.5 + robot->body_height;
        action_start_time = robot->node_->get_clock()->now();
        stage             = 4;
    }
    if (stage == 4) {
        double t          = (robot->node_->get_clock()->now() - action_start_time).seconds();
        ver_vel           = 2.0 * la * t + lb;
        ver_pos           = la * t * t * 0.5 + lb * t + lc;
        lf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lf_grivate * (1.0 + ver_acc / 9.8));
        rf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rf_grivate * (1.0 + ver_acc / 9.8));
        lb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lb_grivate * (1.0 + ver_acc / 9.8));
        rb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rb_grivate * (1.0 + ver_acc / 9.8));
        if (t > jump_cmd.t1 / 2.0)
            stage = 5;
    }
    if (stage == 5)  // 准备跳跃结束，进入起跳阶段
    {
        ver_acc = 2.0 * (jump_cmd.finished_jump_height - jump_cmd.ready_jump_height) / (jump_cmd.t2 * jump_cmd.t2);
        la      = ver_acc * 0.5;
        lb      = 0.0;
        lc      = jump_cmd.ready_jump_height;

        action_start_time = robot->node_->get_clock()->now();
        stage             = 6;
    }
    if (stage == 6)  // 执行起跳动作
    {
        double t          = (robot->node_->get_clock()->now() - action_start_time).seconds();
        ver_vel           = 2.0 * la * t + lb;
        ver_pos           = la * t * t * 0.5 + lb * t + lc;
        lf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lf_grivate * (1.0 + ver_acc / 9.8));
        rf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rf_grivate * (1.0 + ver_acc / 9.8));
        lb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lb_grivate * (1.0 + ver_acc / 9.8));
        rb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rb_grivate * (1.0 + ver_acc / 9.8));
        if (t > jump_cmd.t2)
            stage = 7;
    }
    if (stage == 7)  // 起跳动作结束，进入飞行阶段
    {
        ver_acc           = 0.0;
        ver_vel           = 0.0;
        ver_pos           = jump_cmd.fly_height;
        action_start_time = robot->node_->get_clock()->now();
        stage             = 8;
    }
    if (stage == 8)  // 等待飞行时间结束
    {
        double t = (robot->node_->get_clock()->now() - action_start_time).seconds();
        if (t > jump_cmd.t2)
            stage = 9;
    }
    if (stage == 9)  // 飞行时间结束，进入落地阶段，准备伸腿缓冲
    {
        action_start_time = robot->node_->get_clock()->now();
        la                = 2.0 * (jump_cmd.touch_height - jump_cmd.fly_height) / (jump_cmd.t3 * jump_cmd.t3 / 4.0) * 0.5;
        lb                = 0.0;
        lc                = jump_cmd.fly_height;
        stage             = 10;
    }
    if (stage == 10) {
        double t = (robot->node_->get_clock()->now() - action_start_time).seconds();
        ver_vel  = 2.0 * la * t + lb;
        ver_pos  = la * t * t * 0.5 + lb * t + lc;
        if (t > jump_cmd.t3 / 2.0)
            stage = 11;
    }
    if (stage == 11) // 反转加速度方向，进入伸腿的减速阶段
    {
        la                = -la;
        lb                = ver_vel;
        lc                = jump_cmd.fly_height + 0.5 * (jump_cmd.touch_height - jump_cmd.fly_height);
        action_start_time = robot->node_->get_clock()->now();
        stage             = 12;
    }
    if (stage == 12) // 伸腿完成
    {
        double t = (robot->node_->get_clock()->now() - action_start_time).seconds();
        ver_vel  = 2.0 * la * t + lb;
        ver_pos  = la * t * t * 0.5 + lb * t + lc;
        if (t > jump_cmd.t3 / 2.0)
            stage = 13;
    }
    if (stage == 13) // 等待加速度传感器检测到一个极大的纵向加速度，判断为落地完成并切入VMC)
    {
        ver_acc = 0.0;
        ver_vel = 0.0;
        ver_pos = jump_cmd.touch_height;


        // TODO:暂时没加加速度传感器，直接切入VMC
        stage = 14;
    }
    if (stage == 14) {                                                                        // VMC计算
        auto lf_cart_pos   = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto lf_cart_vel   = robot->lf_leg_calc->foot_vel(robot->lf_joint_pos, robot->lf_joint_vel);
        auto lf_cart_force = robot->lf_leg_calc->foot_force(robot->lf_joint_pos, robot->lf_joint_torque, robot->lf_forward_torque);
        std::tie(lf_foot_exp_pos[2], lf_foot_exp_vel[2], lf_foot_exp_acc[2]) =
            robot->lf_z_vmc->targetUpdate(0.0, lf_cart_pos[2], 0.0, lf_cart_vel[2], -lf_cart_force[2]);
        lf_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_lf_grivate);

        auto rf_cart_pos   = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto rf_cart_vel   = robot->rf_leg_calc->foot_vel(robot->rf_joint_pos, robot->rf_joint_vel);
        auto rf_cart_force = robot->rf_leg_calc->foot_force(robot->rf_joint_pos, robot->rf_joint_torque, robot->rf_forward_torque);
        std::tie(rf_foot_exp_pos[2], rf_foot_exp_vel[2], rf_foot_exp_acc[2]) =
            robot->rf_z_vmc->targetUpdate(0.0, rf_cart_pos[2], 0.0, rf_cart_vel[2], -rf_cart_force[2]);
        rf_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_rf_grivate);

        auto lb_cart_pos   = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto lb_cart_vel   = robot->lb_leg_calc->foot_vel(robot->lb_joint_pos, robot->lb_joint_vel);
        auto lb_cart_force = robot->lb_leg_calc->foot_force(robot->lb_joint_pos, robot->lb_joint_torque, robot->lb_forward_torque);
        std::tie(lb_foot_exp_pos[2], lb_foot_exp_vel[2], lb_foot_exp_acc[2]) =
            robot->lb_z_vmc->targetUpdate(0.0, lb_cart_pos[2], 0.0, lb_cart_vel[2], -lb_cart_force[2]);
        lb_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_lb_grivate);

        auto rb_cart_pos   = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
        auto rb_cart_vel   = robot->rb_leg_calc->foot_vel(robot->rb_joint_pos, robot->rb_joint_vel);
        auto rb_cart_force = robot->rb_leg_calc->foot_force(robot->rb_joint_pos, robot->rb_joint_torque, robot->rb_forward_torque);
        std::tie(rb_foot_exp_pos[2], rb_foot_exp_vel[2], rb_foot_exp_acc[2]) =
            robot->rb_z_vmc->targetUpdate(0.0, rb_cart_pos[2], 0.0, rb_cart_vel[2], -rb_cart_force[2]);
        rb_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_rb_grivate);

        double acc                    = exp_vel_kp * (robot->move_cmd.vx - current_body_vel); // 在当前状态下只关注x方向速度
        current_exp_vel               = current_exp_vel + acc * 0.004;
        double current_exp_foot_force = robot->robot_mass * acc * 0.25;

        lf_wheel_force = current_exp_foot_force;
        rf_wheel_force = -current_exp_foot_force;
        lb_wheel_force = current_exp_foot_force;
        rb_wheel_force = -current_exp_foot_force;


        if (robot->move_cmd.step_mode == 1) {       //检查是否要切换状态
            return "stop";
        }else if(robot->move_cmd.step_mode == 2){
            return "walk";
        }else if(robot->move_cmd.step_mode==3){
            return "climb_steps";
        }
    }
    // 不论任何时间都计算期望速度
    lf_wheel_vel = current_exp_vel;
    rf_wheel_vel = -current_exp_vel;
    lb_wheel_vel = current_exp_vel;
    rb_wheel_vel = -current_exp_vel;

    robot_interfaces::msg::Robot joints_target;
    joints_target.legs[0] = robot->signal_leg_calc(
        lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, robot->lf_leg_calc, &robot->lf_forward_torque, lf_wheel_vel,
        lf_wheel_force);
    joints_target.legs[1] = robot->signal_leg_calc(
        rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, robot->rf_leg_calc, &robot->rf_forward_torque, rf_wheel_vel,
        rf_wheel_force);
    joints_target.legs[2] = robot->signal_leg_calc(
        lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, robot->lb_leg_calc, &robot->lb_forward_torque, lb_wheel_vel,
        lb_wheel_force);
    joints_target.legs[3] = robot->signal_leg_calc(
        rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, robot->rb_leg_calc, &robot->rb_forward_torque, rb_wheel_vel,
        rb_wheel_force);
    robot->legs_target_pub->publish(joints_target);

    return "jump";
}
