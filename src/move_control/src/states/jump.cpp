#include "states/jump.hpp"
#include "core/robot.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <robot_interfaces/msg/robot.hpp>

JumpState::JumpState(Robot* robot)
    : BaseState<Robot>("jump") {
    this->robot  = robot;
    // 创建跳跃命令订阅器，接收跳跃参数
    jump_cmd_sub = robot->node_->create_subscription<robot_interfaces::msg::JumpCmd>(
        "jump_cmd", 10, [this](const robot_interfaces::msg::JumpCmd& msg) {
            jump_cmd = msg;
            RCLCPP_INFO(this->robot->node_->get_logger(), "执行跳跃动作");
            // 检查命令是否过期（超过0.3秒），如果过期则忽略
            if ((this->robot->node_->get_clock()->now() - jump_cmd.stamp).seconds()
                > 0.3) {
                RCLCPP_WARN(this->robot->node_->get_logger(), "跳跃命令过期，忽略");
                return;
            }

            stage = 1;  // 切换到跳跃准备阶段
        });
}

// 进入跳跃状态时的初始化函数
bool JumpState::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    // 初始化垂直方向的运动参数
    ver_la  = 0.0;  // 垂直加速度系数a (s = a*t^2 + b*t + c)
    ver_lb  = 0.0;  // 垂直速度系数b
    ver_lc  = 0.0;  // 垂直位置系数c
    ver_acc = 0.0;  // 垂直加速度
    ver_vel = 0.0;  // 垂直速度
    ver_pos = 0.0;  // 垂直位置

    stage = 0;  // 初始阶段为0（等待跳跃命令）
    return true;
}

// 跳跃状态的主更新函数
std::string JumpState::update(Robot* robot) {

    // 初始化各足端的期望位置、力、速度、加速度
    Vector3D lf_foot_exp_pos = Vector3D::Zero(), rf_foot_exp_pos = Vector3D::Zero(), lb_foot_exp_pos = Vector3D::Zero(),
             rb_foot_exp_pos   = Vector3D::Zero();
    Vector3D lf_foot_exp_force = Vector3D::Zero(), rf_foot_exp_force = Vector3D::Zero(), lb_foot_exp_force = Vector3D::Zero(),
             rb_foot_exp_force = Vector3D::Zero();
    Vector3D lf_foot_exp_vel = Vector3D::Zero(), rf_foot_exp_vel = Vector3D::Zero(), lb_foot_exp_vel = Vector3D::Zero(),
             rb_foot_exp_vel = Vector3D::Zero();
    Vector3D lf_foot_exp_acc = Vector3D::Zero(), rf_foot_exp_acc = Vector3D::Zero(), lb_foot_exp_acc = Vector3D::Zero(),
             rb_foot_exp_acc = Vector3D::Zero();

    // 初始化轮子的速度和力
    double lf_wheel_vel = 0.0f, rf_wheel_vel = 0.0f, lb_wheel_vel = 0.0f, rb_wheel_vel = 0.0f;
    double lf_wheel_force = 0.0f, rf_wheel_force = 0.0f, lb_wheel_force = 0.0f, rb_wheel_force = 0.0f;

    // 计算当前身体速度（基于轮子转速）
    double current_body_vel =
        (robot->lf_wheel_omega - robot->rf_wheel_omega + robot->lb_wheel_omega - robot->rb_wheel_omega) * 0.25 * robot->WHEEL_RADIUS;
    // 计算期望加速度（基于PID控制）
    double acc                    = exp_vel_kp * (robot->move_cmd.vx - current_body_vel);
    // 计算每个足端需要施加的水平力
    double current_exp_foot_force = robot->robot_mass * acc * 0.25;

    // 阶段0：等待跳跃命令或处理普通移动
    if (stage == 0) {
        if (robot->move_cmd.step_mode == 6) {
            // 步态模式6：施加水平力进行加速
            lf_wheel_force = current_exp_foot_force;
            rf_wheel_force = -current_exp_foot_force;
            lb_wheel_force = current_exp_foot_force;
            rb_wheel_force = -current_exp_foot_force;
        }

        if (robot->move_cmd.step_mode == 1)
            return "stop";  // 如果收到停止命令，切换到停止状态
    }
    
    // 阶段1：开始执行跳跃动作 - 进入准备下蹲阶段
    if (stage == 1) {
        action_start_time = robot->node_->get_clock()->now();  // 记录动作开始时间
        
        // 计算下蹲阶段的加速度（匀加速运动到目标高度）
        // 使用公式：s = (1/2)*a*t^2，这里t = t1/2（分两段加速减速）
        ver_acc = (robot->body_height - jump_cmd.ready_jump_height) / (jump_cmd.t1 * jump_cmd.t1 / 4.0);
        ver_la  = ver_acc * 0.5;  // 二次项系数（0.5*a*t^2）
        ver_lb  = 0.0;            // 一次项系数(v0*t)
        ver_lc  = 0.0;            // 常数项(s0)

        stage = 2;
        RCLCPP_INFO(robot->node_->get_logger(), "下蹲阶段1");
    }
    
    // 阶段2：下蹲加速阶段（质心向下加速）
    if (stage == 2) {
        double t = (robot->node_->get_clock()->now() - action_start_time).seconds();
        // 计算当前时刻的速度和位置
        ver_vel = 2.0 * ver_la * t + ver_lb;
        ver_pos = ver_la * t * t + ver_lb * t + ver_lc;

        // 计算各足端期望力：调整重力分量以产生向下加速度
        // F = m*g*(1 - a/g)，当a向下时，需要减小支撑力
        lf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lf_grivate * (1.0 - ver_acc / 9.8));
        rf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rf_grivate * (1.0 - ver_acc / 9.8));
        lb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lb_grivate * (1.0 - ver_acc / 9.8));
        rb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rb_grivate * (1.0 - ver_acc / 9.8));

        // 设置足端期望位置、速度、加速度
        lf_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        rf_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        lb_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        rb_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);

        lf_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        rf_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        lb_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        rb_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);

        lf_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        rf_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        lb_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        rb_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);

        // 如果下蹲时间达到一半，切换到减速阶段
        if (t > jump_cmd.t1 / 2.0)
            stage = 3;
    }
    
    // 阶段3：准备下蹲减速阶段
    if (stage == 3) {
        ver_acc = -ver_acc;  // 反转加速度方向（开始减速）
        ver_la  = ver_acc * 0.5;
        ver_lb  = ver_vel;   // 初始速度为当前速度
        ver_lc  = ver_pos;   // 初始位置为当前位置

        action_start_time = robot->node_->get_clock()->now();
        stage             = 4;
        RCLCPP_INFO(robot->node_->get_logger(), "下蹲阶段2");
    }
    
    // 阶段4：下蹲减速阶段（质心向下减速到目标位置）
    if (stage == 4) {
        double t = (robot->node_->get_clock()->now() - action_start_time).seconds();
        ver_vel  = 2.0 * ver_la * t + ver_lb;
        ver_pos  = ver_la * t * t + ver_lb * t + ver_lc;

        // 同样的力计算逻辑，但现在是向上加速度（减速向下运动）
        lf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lf_grivate * (1.0 - ver_acc / 9.8));
        rf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rf_grivate * (1.0 - ver_acc / 9.8));
        lb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lb_grivate * (1.0 - ver_acc / 9.8));
        rb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rb_grivate * (1.0 - ver_acc / 9.8));

        // 设置足端期望状态
        lf_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        rf_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        lb_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        rb_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);

        lf_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        rf_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        lb_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        rb_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);

        lf_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        rf_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        lb_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        rb_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);

        // 下蹲完成，准备起跳
        if (t > jump_cmd.t1 / 2.0)
            stage = 5;
    }
    
    // 阶段5：准备起跳阶段
    if (stage == 5) {
        // 计算起跳加速度：使用能量守恒 v^2 = 2*a*s
        ver_acc = -jump_cmd.v0 * jump_cmd.v0 / (2.0 * (jump_cmd.finished_jump_height - jump_cmd.ready_jump_height));
        ver_la  = ver_acc * 0.5;
        ver_lb  = 0.0;
        ver_lc  = ver_pos;  // 从当前位置开始起跳

        // 计算起跳时间：基于水平速度分量
        action_time = jump_cmd.v0 * std::cos(jump_cmd.v0_dir) / std::abs(ver_acc);
        // 计算水平加速度（用于调整水平速度）
        hor_acc = -jump_cmd.v0 * std::sin(jump_cmd.v0_dir) / action_time;

        hor_la = hor_acc * 0.5;
        hor_lb = 0.0;
        hor_lc = 0.0;

        RCLCPP_INFO(robot->node_->get_logger(), "起跳阶段");

        action_start_time = robot->node_->get_clock()->now();
        stage             = 6;
    }
    
    // 阶段6：执行起跳动作
    if (stage == 6) {
        double t = (robot->node_->get_clock()->now() - action_start_time).seconds();
        // 计算垂直和水平方向的运动状态
        ver_vel = 2.0 * ver_la * t + ver_lb;
        ver_pos = ver_la * t * t + ver_lb * t + ver_lc;

        hor_vel = 2.0 * hor_la * t + hor_lb;
        hor_pos = hor_la * t * t + hor_lb * t + hor_lc;

        // 定义反对称矩阵函数，用于计算力矩
        auto skew = [](const Eigen::Vector3d& r) {
            Eigen::Matrix3d S;
            // clang-format off
                S << 0.0f   , -r.z(), r.y(),
                    r.z()   , 0.0f  , -r.x(),
                    -r.y()  , r.x() , 0;
            // clang-format on
            return S;
        };
        
        // 使用伪逆方法求解足端力分配问题
        // A矩阵：6x12，前3行表示力平衡，后3行表示力矩平衡
        // b向量：期望的合力和合力矩
        Eigen::Vector<double, 6> b;
        Eigen::Matrix<double, 6, 12> A;

        // 力平衡方程：F1 + F2 + F3 + F4 = m*a
        A.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        A.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        A.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
        A.block<3, 3>(0, 9) = Eigen::Matrix3d::Identity();

        // 力矩平衡方程：r1×F1 + r2×F2 + r3×F3 + r4×F4 = I*alpha
        auto foot_exp_pos   = Vector3D(hor_pos, 0.0, ver_pos);
        A.block<3, 3>(3, 0) = skew(foot_exp_pos + robot->lf_leg_calc->pos_offset - robot->comm_pos);
        A.block<3, 3>(3, 3) = skew(foot_exp_pos + robot->rf_leg_calc->pos_offset - robot->comm_pos);
        A.block<3, 3>(3, 6) = skew(foot_exp_pos + robot->lb_leg_calc->pos_offset - robot->comm_pos);
        A.block<3, 3>(3, 9) = skew(foot_exp_pos + robot->rb_leg_calc->pos_offset - robot->comm_pos);

        // 期望的合力：质量×加速度（包含重力）
        b.block<3, 1>(0, 0) = robot->robot_mass * Vector3D(hor_acc, 0.0, ver_acc + 9.8);
        // 期望的合力矩：假设为0（无旋转）
        b.block<3, 1>(3, 0) = Eigen::Vector3d::Zero();

        // 求解最小二乘问题得到各足端力
        auto F = A.completeOrthogonalDecomposition().solve(b);

        lf_foot_exp_force = F.block<3, 1>(0, 0);
        rf_foot_exp_force = F.block<3, 1>(3, 0);
        lb_foot_exp_force = F.block<3, 1>(6, 0);
        rb_foot_exp_force = F.block<3, 1>(9, 0);

        // 设置足端期望状态（包含水平和垂直分量）
        lf_foot_exp_pos = Vector3D(hor_pos, 0.0, ver_pos);
        rf_foot_exp_pos = Vector3D(hor_pos, 0.0, ver_pos);
        lb_foot_exp_pos = Vector3D(hor_pos, 0.0, ver_pos);
        rb_foot_exp_pos = Vector3D(hor_pos, 0.0, ver_pos);

        lf_foot_exp_vel = Vector3D(hor_vel, 0.0, ver_vel);
        rf_foot_exp_vel = Vector3D(hor_vel, 0.0, ver_vel);
        lb_foot_exp_vel = Vector3D(hor_vel, 0.0, ver_vel);
        rb_foot_exp_vel = Vector3D(hor_vel, 0.0, ver_vel);

        lf_foot_exp_acc = Vector3D(hor_acc, 0.0, ver_acc);
        rf_foot_exp_acc = Vector3D(hor_acc, 0.0, ver_acc);
        lb_foot_exp_acc = Vector3D(hor_acc, 0.0, ver_acc);
        rb_foot_exp_acc = Vector3D(hor_acc, 0.0, ver_acc);

        // 将足端水平力转换为轮子力
        lf_wheel_force += -lf_foot_exp_force[0];
        rf_wheel_force -= -rf_foot_exp_force[0];
        lb_wheel_force += -lb_foot_exp_force[0];
        rb_wheel_force -= -rb_foot_exp_force[0];

        RCLCPP_INFO(robot->node_->get_logger(), "ver_pos=%lf", ver_pos);

        // 起跳时间结束，进入飞行阶段
        if (t > action_time)
            stage = 7;
    }
    
    // 阶段7：起跳完成，进入飞行阶段
    if (stage == 7) {
        ver_acc = 0.0;
        ver_vel = 0.0;
        ver_pos = robot->body_height - jump_cmd.fly_height;  // 飞行高度
        action_start_time = robot->node_->get_clock()->now();
        stage = 8;
        RCLCPP_INFO(robot->node_->get_logger(), "飞行阶段");
    }
    
    // 阶段8：飞行阶段（空中姿态保持）
    if (stage == 8) {
        double t = (robot->node_->get_clock()->now() - action_start_time).seconds();

        // 飞行阶段足端期望状态（保持固定高度）
        lf_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        rf_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        lb_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        rb_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);

        lf_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        rf_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        lb_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        rb_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);

        lf_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        rf_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        lb_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        rb_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);

        // 飞行时间结束，准备落地
        if (t > jump_cmd.t2)
            stage = 9;
    }
    
    // 阶段9：准备落地阶段1（开始伸腿缓冲）
    if (stage == 9) {
        action_start_time = robot->node_->get_clock()->now();
        // 计算落地缓冲加速度
        ver_acc = (jump_cmd.touch_height - jump_cmd.fly_height) / (jump_cmd.t3 * jump_cmd.t3 / 4.0);
        ver_la  = 0.5 * ver_acc;
        ver_lb  = 0.0;
        ver_lc  = robot->body_height - jump_cmd.fly_height;
        stage   = 10;

        RCLCPP_INFO(robot->node_->get_logger(), "准备落地阶段1");
    }
    
    // 阶段10：落地缓冲加速阶段
    if (stage == 10) {
        double t = (robot->node_->get_clock()->now() - action_start_time).seconds();
        ver_vel = 2.0 * ver_la * t + ver_lb;
        ver_pos = ver_la * t * t + ver_lb * t + ver_lc;

        // 设置足端期望状态进行缓冲
        lf_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        rf_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        lb_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        rb_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);

        lf_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        rf_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        lb_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        rb_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);

        lf_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        rf_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        lb_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        rb_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);

        // 缓冲时间过半，准备减速
        if (t > jump_cmd.t3 / 2.0)
            stage = 11;
    }
    
    // 阶段11：准备落地阶段2（缓冲减速）
    if (stage == 11) {
        ver_la = -ver_la;  // 反转加速度
        ver_lb = ver_vel;  // 当前速度作为初始速度
        ver_lc = ver_pos;  // 当前位置作为初始位置
        action_start_time = robot->node_->get_clock()->now();
        stage = 12;
        RCLCPP_INFO(robot->node_->get_logger(), "准备落地阶段2");
    }
    
    // 阶段12：落地缓冲减速阶段
    if (stage == 12) {
        double t = (robot->node_->get_clock()->now() - action_start_time).seconds();
        ver_vel = 2.0 * ver_la * t + ver_lb;
        ver_pos = ver_la * t * t + ver_lb * t + ver_lc;

        // 继续设置缓冲状态
        lf_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        rf_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        lb_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);
        rb_foot_exp_pos = Vector3D(0.0, 0.0, ver_pos);

        lf_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        rf_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        lb_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);
        rb_foot_exp_vel = Vector3D(0.0, 0.0, ver_vel);

        lf_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        rf_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        lb_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);
        rb_foot_exp_acc = Vector3D(0.0, 0.0, ver_acc);

        // 缓冲完成，准备检测落地
        if (t > jump_cmd.t3 / 2.0)
            stage = 13;
    }
    
    // 阶段13：等待落地检测
    if (stage == 13) {
        ver_acc = 0.0;
        ver_vel = 0.0;
        // ver_pos 保持为最后一次的值

        // TODO:暂时没加加速度传感器，直接切入VMC（虚拟模型控制）
        stage = 14;
    }
    
    // 阶段14：落地后VMC控制阶段（姿态平衡）
    if (stage == 14) {
        // 获取当前机器人姿态（roll, pitch, yaw）
        double cur_roll, cur_pitch, cur_yaw;
        tf2::Matrix3x3(robot->robot_rotation).getRPY(cur_roll, cur_pitch, cur_yaw);
        // 计算平衡所需的虚拟力
        std::tie(lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force) =
            balance_force_calc(robot, cur_roll, cur_pitch);

        // 检查是否倾倒（角度过大）
        if ((cur_roll > 40 * 3.14 / 180 || cur_roll < -40 * 3.14 / 180 || cur_pitch > 50 * 3.14 / 180
             || cur_pitch < -50 * 3.14 / 180))
            return "idel";  // 切入空闲状态

        // 对每个足端应用VMC控制（垂直方向阻抗控制）
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

        // 恢复轮子控制力
        lf_wheel_force = current_exp_foot_force;
        rf_wheel_force = -current_exp_foot_force;
        lb_wheel_force = current_exp_foot_force;
        rb_wheel_force = -current_exp_foot_force;

        // 检查是否要切换到停止状态
        if (robot->move_cmd.step_mode == 1) {
            return "stop";
        }
    }
    
    // 不论任何阶段都计算期望轮子速度（用于前进控制）
    lf_wheel_vel = robot->move_cmd.vx;
    rf_wheel_vel = -robot->move_cmd.vx;
    lb_wheel_vel = robot->move_cmd.vx;
    rb_wheel_vel = -robot->move_cmd.vx;

    // 计算关节目标值并发布
    robot_interfaces::msg::RobotTarget joints_target;
    joints_target.legs[0] = robot->lf_leg_calc->signal_leg_calc(
        lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, &robot->lf_forward_torque, lf_wheel_vel, lf_wheel_force);
    joints_target.legs[1] = robot->rf_leg_calc->signal_leg_calc(
        rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, &robot->rf_forward_torque, rf_wheel_vel, rf_wheel_force);
    joints_target.legs[2] = robot->lb_leg_calc->signal_leg_calc(
        lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, &robot->lb_forward_torque, lb_wheel_vel, lb_wheel_force);
    joints_target.legs[3] = robot->rb_leg_calc->signal_leg_calc(
        rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, &robot->rb_forward_torque, rb_wheel_vel, rb_wheel_force);
    robot->legs_target_pub->publish(joints_target);

    return "jump";  // 继续保持跳跃状态
}

// 计算平衡虚拟力的函数
std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
    JumpState::balance_force_calc(Robot* robot, double cur_roll, double cur_pitch) {

    // 使用VMC控制器计算roll和pitch方向的虚拟力矩
    double roll_offset_virtual_torque  = robot->roll_vmc->update(cur_roll, robot->robot_velocity.angular.x, 0.0);
    double pitch_offset_virtual_torque = robot->pitch_vmc->update(cur_pitch, robot->robot_velocity.angular.y, 0.0);

    // 初始化各足端力
    Eigen::Vector3d lf_force = Eigen::Vector3d::Zero(), rf_force = Eigen::Vector3d::Zero(), lb_force = Eigen::Vector3d::Zero(),
                    rb_force = Eigen::Vector3d::Zero();
    
    // 根据pitch力矩分配到各足端（力臂为x方向距离）
    lf_force[2] += pitch_offset_virtual_torque * robot->lf_leg_calc->pos_offset[0];
    rf_force[2] += pitch_offset_virtual_torque * robot->rf_leg_calc->pos_offset[0];
    lb_force[2] += pitch_offset_virtual_torque * robot->lb_leg_calc->pos_offset[0];
    rb_force[2] += pitch_offset_virtual_torque * robot->rb_leg_calc->pos_offset[0];

    // 根据roll力矩分配到各足端（力臂为y方向距离）
    lf_force[2] += roll_offset_virtual_torque * robot->lf_leg_calc->pos_offset[1];
    rf_force[2] += roll_offset_virtual_torque * robot->rf_leg_calc->pos_offset[1];
    lb_force[2] += roll_offset_virtual_torque * robot->lb_leg_calc->pos_offset[1];
    rb_force[2] += roll_offset_virtual_torque * robot->rb_leg_calc->pos_offset[1];

    return {lf_force, rf_force, lb_force, rb_force};
}