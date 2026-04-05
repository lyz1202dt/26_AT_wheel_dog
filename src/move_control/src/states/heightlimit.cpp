#include "states/heightlimit.hpp"
#include "core/robot.hpp"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

HeightlimitState::HeightlimitState(Robot* robot)
    : BaseState<Robot>("heightlimit")
{
    // 动态参数
    robot->node_->declare_parameter("fall_time",    1.5);
    robot->node_->declare_parameter("up_time",      1.5);
    robot->node_->declare_parameter("drive_time",   2.0);
    robot->node_->declare_parameter("wheel_vel",    30.0);
    robot->node_->declare_parameter("target_distance", 1.0);  // 目标距离 1.0m

    // 参数回调
    robot->add_param_cb([this](const rclcpp::Parameter& param) {
        auto name = param.get_name();
        if (name == "fall_time")    fall_time = param.as_double();
        else if (name == "up_time") up_time = param.as_double();
        else if (name == "drive_time") drive_time = param.as_double();
        else if (name == "wheel_vel") wheel_vel = param.as_double();
        else if (name == "target_distance") target_distance = param.as_double();
        return true;
    });

    // 滤波初始化
    for (int i = 0; i < 4; i++) {
        last_filtered_wheel_vel[i] = 0.0;
    }

    // 上一时刻时间初始化
    last_time = robot->node_->get_clock()->now();
}

bool HeightlimitState::enter(Robot* robot, const std::string& last_status)
{
    // 重置状态
    stage = 0;
    total_distance = 0.0;  // 距离清零

    // 记录进入时的腿部初始角度（站起来要回去）
    lf_init = robot->lf_joint_pos;
    rf_init = robot->rf_joint_pos;
    lb_init = robot->lb_joint_pos;
    rb_init = robot->rb_joint_pos;

    RCLCPP_INFO(robot->node_->get_logger(), "我准备好了lf_init:(%lf,%lf,%lf), rf_init:(%lf,%lf,%lf), lb_init:(%lf,%lf,%lf), rb_init:(%lf,%lf,%lf)",
                lf_init[0], lf_init[1], lf_init[2], rf_init[0], rf_init[1], rf_init[2], lb_init[0], lb_init[1], lb_init[2],
                rb_init[0], rb_init[1], rb_init[2]);
    return true;
}


double HeightlimitState::low_pass_filter(double input, int wheel_idx)
{
    const double alpha = 0.1;
    double filtered = alpha * input + (1.0 - alpha) * last_filtered_wheel_vel[wheel_idx];
    last_filtered_wheel_vel[wheel_idx] = filtered;
    return filtered;
}

std::string HeightlimitState::update(Robot* robot)
{

    robot_interfaces::msg::RobotTarget msg{};
    for(int i=0;i<3;i++)
    {
        msg.legs[0].joints[i].kp = robot->kp[i];
        msg.legs[0].joints[i].kd = robot->kd[i];
        msg.legs[0].wheel.kd = robot->wheel_kd;

        msg.legs[1].joints[i].kp = robot->kp[i];
        msg.legs[1].joints[i].kd = robot->kd[i];
        msg.legs[1].wheel.kd = robot->wheel_kd;

        msg.legs[2].joints[i].kp = robot->kp[i];
        msg.legs[2].joints[i].kd = robot->kd[i];
        msg.legs[2].wheel.kd = robot->wheel_kd;

        msg.legs[3].joints[i].kp = robot->kp[i];
        msg.legs[3].joints[i].kd = robot->kd[i];
        msg.legs[3].wheel.kd = robot->wheel_kd;
    }

    auto current_time = robot->node_->get_clock()->now();
    double dt = (current_time - last_time).seconds();
    last_time = current_time;
    double raw_wheel[4];
    raw_wheel[0] = robot->lf_wheel_omega;
    raw_wheel[1] = robot->rf_wheel_omega;
    raw_wheel[2] = robot->lb_wheel_omega;
    raw_wheel[3] = robot->rb_wheel_omega;
    raw_wheel[0] = (float)(robot->lf_wheel_omega-robot->rf_wheel_omega+robot->lb_wheel_omega-robot->rb_wheel_omega)/4;
    raw_wheel[1] = (float)(robot->lf_wheel_omega-robot->rf_wheel_omega+robot->lb_wheel_omega-robot->rb_wheel_omega)/4;
    raw_wheel[2] = (float)(robot->lf_wheel_omega-robot->rf_wheel_omega+robot->lb_wheel_omega-robot->rb_wheel_omega)/4;
    raw_wheel[3] = (float)(robot->lf_wheel_omega-robot->rf_wheel_omega+robot->lb_wheel_omega-robot->rb_wheel_omega)/4;
    RCLCPP_INFO(robot->node_->get_logger(), "raw_wheel: [%lf, %lf, %lf, %lf]", 
    raw_wheel[0], raw_wheel[1], raw_wheel[2], raw_wheel[3]);
    double filtered_wheel[4];
    for (int i = 0; i < 4; i++) {
        filtered_wheel[i] = low_pass_filter(raw_wheel[i], i);
    }


    const double wheel_radius = robot->WHEEL_RADIUS;
    double avg_vel = 0.0;
    for (int i = 0; i < 4; i++) {
        avg_vel += filtered_wheel[i] * wheel_radius;
    }
    avg_vel /= 4.0;

    double delta_distance = avg_vel * dt;
    total_distance += delta_distance;


    // 阶段 0：开始蹲下
    // ==============================
    if (stage == 0) {
        start_time = robot->node_->get_clock()->now();

        Eigen::Vector3d lf_tar(0.1, 1.1, -0.7);
        Eigen::Vector3d rf_tar(-0.1,-1.1, 0.7);
        Eigen::Vector3d lb_tar(-0.1,1.1, -0.7);
        Eigen::Vector3d rb_tar(0.1, -1.1, 0.7);

        lf_leg_step.update_support_trajectory(lf_init, lf_tar, fall_time);
        rf_leg_step.update_support_trajectory(rf_init, rf_tar, fall_time);
        lb_leg_step.update_support_trajectory(lb_init, lb_tar, fall_time);
        rb_leg_step.update_support_trajectory(rb_init, rb_tar, fall_time);

        stage = 1;
    }

    // ==============================
    // 阶段 1：执行蹲下
    // ==============================
    else if (stage == 1) {
        bool ok;
        double t = (robot->node_->get_clock()->now() - start_time).seconds();
        auto lf = lf_leg_step.get_target(t, ok);
        auto rf = rf_leg_step.get_target(t, ok);
        auto lb = lb_leg_step.get_target(t, ok);
        auto rb = rb_leg_step.get_target(t, ok);

        //robot_interfaces::msg::RobotTarget msg;
        for(int i=0;i<3;i++){
            msg.legs[0].joints[i].rad = (float)std::get<0>(lf)[i];
            msg.legs[1].joints[i].rad = (float)std::get<0>(rf)[i];
            msg.legs[2].joints[i].rad = (float)std::get<0>(lb)[i];
            msg.legs[3].joints[i].rad = (float)std::get<0>(rb)[i];
        }
        robot->legs_target_pub->publish(msg);

        if (t > fall_time) {
            RCLCPP_INFO(robot->node_->get_logger(), "开始前进");
            start_time = robot->node_->get_clock()->now();
            stage = 2;
        }
    }

    // ==============================
    // 阶段 2：蹲下 + 轮子转动
    // ==============================
    else if (stage == 2) {
        // robot_interfaces::msg::RobotTarget msg;
        for(int i=0;i<3;i++){
            bool dummy;
            msg.legs[0].joints[i].rad = (float)std::get<0>(lf_leg_step.get_target(fall_time, dummy))[i];
            msg.legs[1].joints[i].rad = (float)std::get<0>(rf_leg_step.get_target(fall_time, dummy))[i];
            msg.legs[2].joints[i].rad = (float)std::get<0>(lb_leg_step.get_target(fall_time, dummy))[i];
            msg.legs[3].joints[i].rad = (float)std::get<0>(rb_leg_step.get_target(fall_time, dummy))[i];
        }

        msg.legs[0].wheel.omega =  (float)low_pass_filter(wheel_vel, 0);
        msg.legs[1].wheel.omega = -(float)low_pass_filter(wheel_vel, 1);
        msg.legs[2].wheel.omega =  (float)low_pass_filter(wheel_vel, 2);
        msg.legs[3].wheel.omega = -(float)low_pass_filter(wheel_vel, 3);

        robot->legs_target_pub->publish(msg);

        // ========================
        // 核心：距离到 0.6m 就站起来
        // ========================
        RCLCPP_INFO(robot->node_->get_logger(), "已行驶: %.2f m", total_distance);
        if (total_distance >= target_distance) {
            RCLCPP_INFO(robot->node_->get_logger(), "已到达 %.2f 米，准备站起来", target_distance);
            stage = 3;
        }
    }

    // ==============================
    // 阶段 3：准备站起
    // ==============================
    else if (stage == 3) {
        start_time = robot->node_->get_clock()->now();

        Eigen::Vector3d lf_down(0.1, 1.1, -0.7);
        Eigen::Vector3d rf_down(-0.1,-1.1, 0.7);
        Eigen::Vector3d lb_down(-0.1,1.1, -0.7);
        Eigen::Vector3d rb_down(0.1, -1.1, 0.7);

        lf_leg_step.update_support_trajectory(lf_down, lf_init, up_time);
        rf_leg_step.update_support_trajectory(rf_down, rf_init, up_time);
        lb_leg_step.update_support_trajectory(lb_down, lb_init, up_time);
        rb_leg_step.update_support_trajectory(rb_down, rb_init, up_time);

        stage = 4;
    }

    // ==============================
    // 阶段 4：站起
    // ==============================
    else if (stage == 4) {
        bool ok;
        double t = (robot->node_->get_clock()->now() - start_time).seconds();
        auto lf = lf_leg_step.get_target(t, ok);
        auto rf = rf_leg_step.get_target(t, ok);
        auto lb = lb_leg_step.get_target(t, ok);
        auto rb = rb_leg_step.get_target(t, ok);

        //robot_interfaces::msg::RobotTarget msg;
        for(int i=0;i<3;i++){
            msg.legs[0].joints[i].rad = (float)std::get<0>(lf)[i];
            msg.legs[1].joints[i].rad = (float)std::get<0>(rf)[i];
            msg.legs[2].joints[i].rad = (float)std::get<0>(lb)[i];
            msg.legs[3].joints[i].rad = (float)std::get<0>(rb)[i];
        }
        robot->legs_target_pub->publish(msg);

        if (t > up_time) {
            RCLCPP_INFO(robot->node_->get_logger(), "动作完成");
            return "idel";
        }
    }
    return "heightlimit";
}
