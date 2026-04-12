#include "states/brige_b.hpp"

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/brigeb.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "core/robot.hpp"

Brige_B::Brige_B(Robot* robot)
    : BaseState<Robot>("brige_b") {

    mass = (robot->robot_lf_grivate + robot->robot_rf_grivate + robot->robot_lb_grivate + robot->robot_rb_grivate) / 9.8;
    mass_center_pos =
        Vector2D(
            robot->robot_lf_grivate * robot->lf_leg_calc->pos_offset[0] + robot->robot_rf_grivate * robot->rf_leg_calc->pos_offset[0]
                + robot->robot_lb_grivate * robot->lb_leg_calc->pos_offset[0] + robot->robot_rb_grivate * robot->rb_leg_calc->pos_offset[0],
            robot->robot_lf_grivate * robot->lf_leg_calc->pos_offset[1] + robot->robot_rf_grivate * robot->rf_leg_calc->pos_offset[1]
                + robot->robot_lb_grivate * robot->lb_leg_calc->pos_offset[1] + robot->robot_rb_grivate * robot->rb_leg_calc->pos_offset[1])
        / (mass * 9.8);


    this->robot           = robot;
    last_line_update_time = robot->node_->get_clock()->now();

    lf_foot_exp_pos   = Vector3D::Zero();
    rf_foot_exp_pos   = Vector3D::Zero();
    lb_foot_exp_pos   = Vector3D::Zero();
    rb_foot_exp_pos   = Vector3D::Zero();
    lf_foot_exp_force = Vector3D::Zero();
    rf_foot_exp_force = Vector3D::Zero();
    lb_foot_exp_force = Vector3D::Zero();
    rb_foot_exp_force = Vector3D::Zero();
    lf_foot_exp_vel   = Vector3D::Zero();
    rf_foot_exp_vel   = Vector3D::Zero();
    lb_foot_exp_vel   = Vector3D::Zero();
    rb_foot_exp_vel   = Vector3D::Zero();
    lf_foot_exp_acc   = Vector3D::Zero();
    rf_foot_exp_acc   = Vector3D::Zero();
    lb_foot_exp_acc   = Vector3D::Zero();
    rb_foot_exp_acc   = Vector3D::Zero();

    rclcpp::Subscription<robot_interfaces::msg::Brigeb>::SharedPtr brigeb_sub =
        robot->node_->create_subscription<robot_interfaces::msg::Brigeb>(
            "brigeb_info", 10, [this](const robot_interfaces::msg::Brigeb& msg) { calc_edge_line(msg); });
}

bool Brige_B::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    state       = 0;
    active_feet = 0;
    return true;
}

std::string Brige_B::update(Robot* robot) {
    Eigen::Vector3d lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos;
    Eigen::Vector3d lf_foot_exp_force = Eigen::Vector3d::Zero(), rf_foot_exp_force = Eigen::Vector3d::Zero(),
                    lb_foot_exp_force = Eigen::Vector3d::Zero(), rb_foot_exp_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d lf_foot_exp_vel = Eigen::Vector3d::Zero(), rf_foot_exp_vel = Eigen::Vector3d::Zero(),
                    lb_foot_exp_vel = Eigen::Vector3d::Zero(), rb_foot_exp_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d lf_foot_exp_acc = Eigen::Vector3d::Zero(), rf_foot_exp_acc = Eigen::Vector3d::Zero(),
                    lb_foot_exp_acc = Eigen::Vector3d::Zero(), rb_foot_exp_acc = Eigen::Vector3d::Zero();

    auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
    auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
    auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
    auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

    double exp_vel        = robot->move_cmd.vx;
    double lf_wheel_force = vel_kp * (exp_vel - robot->lb_wheel_omega * robot->WHEEL_RADIUS); // P控制器，计算轮子需要的前馈力
    double rf_wheel_force = vel_kp * (-exp_vel - robot->rb_wheel_omega * robot->WHEEL_RADIUS);
    double lb_wheel_force = vel_kp * (exp_vel - robot->lf_wheel_omega * robot->WHEEL_RADIUS);
    double rb_wheel_force = vel_kp * (-exp_vel - robot->rf_wheel_omega * robot->WHEEL_RADIUS);


    if (state == 0) {                                                                         // 检测边缘线
        // 1s内更新边缘线信息，判定状态跳转条件
        if (robot->node_->get_clock()->now() - last_line_update_time < rclcpp::Duration::from_seconds(1.0)) {
            double lf_distance = get_wheel2edge_distance(robot->lf_leg_calc, robot->lf_joint_pos);
            double rf_distance = get_wheel2edge_distance(robot->rf_leg_calc, robot->rf_joint_pos);
            double lb_distance = get_wheel2edge_distance(robot->lb_leg_calc, robot->lb_joint_pos);
            double rb_distance = get_wheel2edge_distance(robot->rb_leg_calc, robot->rb_joint_pos);

            double front_min_distance = std::min(lf_distance, rf_distance);
            double back_min_distance  = std::min(lb_distance, rb_distance);
            double min_distance       = std::min(front_min_distance, back_min_distance);
            if (min_distance < 0.08)                                                                  // 开始进行对要迈的腿的检查
            {
                state = 1;
                next_step_length =
                    std::max(std::max(lf_distance, rf_distance), std::max(lb_distance, rb_distance))+0.2; // 下一个迈腿长度为几个间隔的最大值
            }
        }

        lf_foot_exp_force = Vector3D(lf_wheel_force, 0.0, -robot->robot_lf_grivate);
        rf_foot_exp_force = Vector3D(rf_wheel_force, 0.0, -robot->robot_rf_grivate);
        lb_foot_exp_force = Vector3D(lb_wheel_force, 0.0, -robot->robot_lb_grivate);
        rb_foot_exp_force = Vector3D(rb_wheel_force, 0.0, -robot->robot_rb_grivate);
    }
    if (state == 1) {
        RCLCPP_INFO(robot->node_->get_logger(), "狗身向右平移");
        double x_target = 0.0, y_target = step_dy;
        lf_leg_step.update_support_trajectory(lf_cart_pos, Vector3D(x_target, y_target, 0.0), 2.0);
        rf_leg_step.update_support_trajectory(rf_cart_pos, Vector3D(0.0, y_target, 0.0), 2.0);
        lb_leg_step.update_support_trajectory(lb_cart_pos, Vector3D(x_target, y_target, 0.0), 2.0);
        rb_leg_step.update_support_trajectory(rb_cart_pos, Vector3D(0.0, y_target, 0.0), 2.0);
        start_time = robot->node_->get_clock()->now();
        state      = 2;
    }
    if (state == 2) {                                                                                 // 身体向右侧平移
        bool success;
        double t = (robot->node_->get_clock()->now() - start_time).seconds();

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        auto lf_pos = (lf_foot_exp_pos + robot->lf_leg_calc->pos_offset).head(2); // 在简化的二维平面模型中，提供支撑力的位置
        auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

        // 使用Eigen求解每条腿的支撑力
        // 构建方程组: A * f = b
        // 约束条件:
        // 1. 垂直力平衡: f_lf + f_rf + f_lb + f_rb = mg
        // 2. 绕质心的力矩平衡(x方向): (lf_pos.x - mass_center_pos.x)*f_lf + ... = 0
        // 3. 绕质心的力矩平衡(y方向): (lf_pos.y - mass_center_pos.y)*f_lf + ... = 0
        Eigen::Matrix<double, 3, 4> A;
        Eigen::Vector3d b;

        // 第一行: 力平衡约束
        A(0, 0) = 1.0; // lf
        A(0, 1) = 1.0; // rf
        A(0, 2) = 1.0; // lb
        A(0, 3) = 1.0; // rb
        b(0)    = mass * 9.8;

        // 第二行: 绕质心的x方向力矩平衡
        A(1, 0) = lf_pos.x() - mass_center_pos.x();
        A(1, 1) = rf_pos.x() - mass_center_pos.x();
        A(1, 2) = lb_pos.x() - mass_center_pos.x();
        A(1, 3) = rb_pos.x() - mass_center_pos.x();
        b(1)    = 0.0;

        // 第三行: 绕质心的y方向力矩平衡
        A(2, 0) = lf_pos.y() - mass_center_pos.y();
        A(2, 1) = rf_pos.y() - mass_center_pos.y();
        A(2, 2) = lb_pos.y() - mass_center_pos.y();
        A(2, 3) = rb_pos.y() - mass_center_pos.y();
        b(2)    = 0.0;

        // 使用最小二乘法求解超定方程组
        Eigen::Vector4d forces = A.transpose() * (A * A.transpose()).inverse() * b;

        // 将求解的垂直力存入各腿的期望力向量（z分量）
        lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
        rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
        lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));
        rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(3));

        if (!success)
            state = 3;
    }
    if (state == 3) {
        RCLCPP_INFO(robot->node_->get_logger(), "左后腿向前摆动");
        Vector3D next_available_pos(next_step_length,step_dy,0.0);
        lb_leg_step.update_flight_trajectory(lb_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 2.0, 0.12);
        start_time = robot->node_->get_clock()->now();
        state      = 4;
    }
    if (state == 4) // 左后腿向前摆动
    {
        bool success;
        double t = (robot->node_->get_clock()->now() - start_time).seconds();

        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);

        // 3足支撑力计算 (lf, rf, rb支撑，lb摆动)
        auto lf_pos = (lf_foot_exp_pos + robot->lf_leg_calc->pos_offset).head(2);
        auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
        auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

        Eigen::Matrix3d A;
        Eigen::Vector3d b;

        // 力平衡约束
        A(0, 0) = 1.0; // lf
        A(0, 1) = 1.0; // rf
        A(0, 2) = 1.0; // rb
        b(0)    = mass * 9.8;

        // 绕质心的x方向力矩平衡
        A(1, 0) = lf_pos.x() - mass_center_pos.x();
        A(1, 1) = rf_pos.x() - mass_center_pos.x();
        A(1, 2) = rb_pos.x() - mass_center_pos.x();
        b(1)    = 0.0;

        // 绕质心的y方向力矩平衡
        A(2, 0) = lf_pos.y() - mass_center_pos.y();
        A(2, 1) = rf_pos.y() - mass_center_pos.y();
        A(2, 2) = rb_pos.y() - mass_center_pos.y();
        b(2)    = 0.0;

        // 求解3x3方程组
        Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

        lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
        rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
        lb_foot_exp_force = Vector3D::Zero(); // 摆动腿无支撑力
        rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));


        if (!success)
            state = 5;
    }
    if (state == 5) {
        RCLCPP_INFO(robot->node_->get_logger(), "左前腿向前摆动");
        Vector3D next_available_pos(next_step_length,step_dy,0.0);
        lf_leg_step.update_flight_trajectory(lf_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 2.0, 0.12);
        start_time = robot->node_->get_clock()->now();
        state      = 6;
    }
    if (state == 6)                           // 左前腿向前摆动
    {
        bool success;
        double t = (robot->node_->get_clock()->now() - start_time).seconds();

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);

        // 3足支撑力计算 (rf, lb, rb支撑，lf摆动)
        auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

        Eigen::Matrix3d A;
        Eigen::Vector3d b;

        // 力平衡约束
        A(0, 0) = 1.0; // rf
        A(0, 1) = 1.0; // lb
        A(0, 2) = 1.0; // rb
        b(0)    = mass * 9.8;

        // 绕质心的x方向力矩平衡
        A(1, 0) = rf_pos.x() - mass_center_pos.x();
        A(1, 1) = lb_pos.x() - mass_center_pos.x();
        A(1, 2) = rb_pos.x() - mass_center_pos.x();
        b(1)    = 0.0;

        // 绕质心的y方向力矩平衡
        A(2, 0) = rf_pos.y() - mass_center_pos.y();
        A(2, 1) = lb_pos.y() - mass_center_pos.y();
        A(2, 2) = rb_pos.y() - mass_center_pos.y();
        b(2)    = 0.0;

        // 求解3x3方程组
        Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

        lf_foot_exp_force = Vector3D::Zero(); // 摆动腿无支撑力
        rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
        lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
        rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));

        if (!success)
            state = 7;
    }
    if (state == 7) {
        RCLCPP_INFO(robot->node_->get_logger(), "狗身向左平移");
        double x_target = next_step_length, y_target = -step_dy;
        lf_leg_step.update_support_trajectory(lf_cart_pos, Vector3D(0.0, y_target, 0.0), 2.0);
        rf_leg_step.update_support_trajectory(rf_cart_pos, Vector3D(x_target, y_target, 0.0), 2.0);
        lb_leg_step.update_support_trajectory(lb_cart_pos, Vector3D(0.0, y_target, 0.0), 2.0);
        rb_leg_step.update_support_trajectory(rb_cart_pos, Vector3D(x_target, y_target, 0.0), 2.0);
        start_time = robot->node_->get_clock()->now();
        state      = 8;
    }
    if (state == 8)                           // 整体向左前方移动
    {
        bool success;
        double t = (robot->node_->get_clock()->now() - start_time).seconds();

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // 计算每个腿要承担的垂直力
        auto lf_pos = (lf_foot_exp_pos + robot->lf_leg_calc->pos_offset).head(2);
        auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

        // 构建方程组: A * f = b
        // 约束条件与state==2相同
        Eigen::Matrix<double, 3, 4> A;
        Eigen::Vector3d b;

        // 第一行: 力平衡约束
        A(0, 0) = 1.0; // lf
        A(0, 1) = 1.0; // rf
        A(0, 2) = 1.0; // lb
        A(0, 3) = 1.0; // rb
        b(0)    = mass * 9.8;

        // 第二行: 绕质心的x方向力矩平衡
        A(1, 0) = lf_pos.x() - mass_center_pos.x();
        A(1, 1) = rf_pos.x() - mass_center_pos.x();
        A(1, 2) = lb_pos.x() - mass_center_pos.x();
        A(1, 3) = rb_pos.x() - mass_center_pos.x();
        b(1)    = 0.0;

        // 第三行: 绕质心的y方向力矩平衡
        A(2, 0) = lf_pos.y() - mass_center_pos.y();
        A(2, 1) = rf_pos.y() - mass_center_pos.y();
        A(2, 2) = lb_pos.y() - mass_center_pos.y();
        A(2, 3) = rb_pos.y() - mass_center_pos.y();
        b(2)    = 0.0;

        // 使用最小二乘法求解超定方程组
        Eigen::Vector4d forces = A.transpose() * (A * A.transpose()).inverse() * b;

        // 将求解的垂直力存入各腿的期望力向量（z分量）
        lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
        rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
        lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));
        rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(3));

        if (!success)
            state = 9;
    }
    if (state == 9) {
        RCLCPP_INFO(robot->node_->get_logger(), "右后腿向前摆动");
        Vector3D next_available_pos(0.0,0.0,0.0);
        rb_leg_step.update_flight_trajectory(rb_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 2.0, 0.12);
        start_time = robot->node_->get_clock()->now();
        state      = 10;
    }
    if (state == 10) // 右后腿向前摆动
    {
        bool success;
        double t                                                    = (robot->node_->get_clock()->now() - start_time).seconds();
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

        // 3足支撑力计算 (lf, rf, lb支撑，rb摆动)
        auto lf_pos = (lf_foot_exp_pos + robot->lf_leg_calc->pos_offset).head(2);
        auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);

        Eigen::Matrix3d A;
        Eigen::Vector3d b;

        // 力平衡约束
        A(0, 0) = 1.0; // lf
        A(0, 1) = 1.0; // rf
        A(0, 2) = 1.0; // lb
        b(0)    = mass * 9.8;

        // 绕质心的x方向力矩平衡
        A(1, 0) = lf_pos.x() - mass_center_pos.x();
        A(1, 1) = rf_pos.x() - mass_center_pos.x();
        A(1, 2) = lb_pos.x() - mass_center_pos.x();
        b(1)    = 0.0;

        // 绕质心的y方向力矩平衡
        A(2, 0) = lf_pos.y() - mass_center_pos.y();
        A(2, 1) = rf_pos.y() - mass_center_pos.y();
        A(2, 2) = lb_pos.y() - mass_center_pos.y();
        b(2)    = 0.0;

        // 求解3x3方程组
        Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

        lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
        rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
        lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));
        rb_foot_exp_force = Vector3D::Zero(); // 摆动腿无支撑力

        if (!success)
            state = 11;
    }
    if (state == 11) {
        RCLCPP_INFO(robot->node_->get_logger(), "右前腿向前摆动");
        Vector3D next_available_pos(0.0,0.0,0.0);
        rf_leg_step.update_flight_trajectory(rf_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 2.0, 0.12);
        start_time = robot->node_->get_clock()->now();
        state      = 12;
    }
    if (state == 12)                          // 右前腿向前摆动
    {
        bool success;
        double t                                                    = (robot->node_->get_clock()->now() - start_time).seconds();
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);

        // 3足支撑力计算 (lf, lb, rb支撑，rf摆动)
        auto lf_pos = (lf_foot_exp_pos + robot->lf_leg_calc->pos_offset).head(2);
        auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

        Eigen::Matrix3d A;
        Eigen::Vector3d b;

        // 力平衡约束
        A(0, 0) = 1.0; // lf
        A(0, 1) = 1.0; // lb
        A(0, 2) = 1.0; // rb
        b(0)    = mass * 9.8;

        // 绕质心的x方向力矩平衡
        A(1, 0) = lf_pos.x() - mass_center_pos.x();
        A(1, 1) = lb_pos.x() - mass_center_pos.x();
        A(1, 2) = rb_pos.x() - mass_center_pos.x();
        b(1)    = 0.0;

        // 绕质心的y方向力矩平衡
        A(2, 0) = lf_pos.y() - mass_center_pos.y();
        A(2, 1) = lb_pos.y() - mass_center_pos.y();
        A(2, 2) = rb_pos.y() - mass_center_pos.y();
        b(2)    = 0.0;

        // 求解3x3方程组
        Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

        lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
        rf_foot_exp_force = Vector3D::Zero(); // 摆动腿无支撑力
        lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
        rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));

        if (!success) {
            state = 0;
            if (robot->move_cmd.step_mode == 1)
                return "stop";
        }
    }


    robot_interfaces::msg::RobotTarget joints_target;
    joints_target.legs[0] = robot->lf_leg_calc->signal_leg_calc(
        lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, &robot->lf_forward_torque, 0.0, 0.0);
    joints_target.legs[1] = robot->rf_leg_calc->signal_leg_calc(
        rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, &robot->rf_forward_torque, 0.0, 0.0);
    joints_target.legs[2] = robot->lb_leg_calc->signal_leg_calc(
        lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, &robot->lb_forward_torque, 0.0, 0.0);
    joints_target.legs[3] = robot->rb_leg_calc->signal_leg_calc(
        rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, &robot->rb_forward_torque, 0.0, 0.0);
    robot->legs_target_pub->publish(joints_target);

    return "brige_b";
}

// 计算当前轮子到不可接触区域边缘的距离
double Brige_B::get_wheel2edge_distance(std::shared_ptr<LegCalc> leg_calc, const Eigen::Vector3d& joint_pos) {
    auto cart_pos                 = leg_calc->foot_pos(joint_pos);
    Eigen::Vector2d xy_pos_offset = cart_pos.head(2) + leg_calc->pos_offset.head(2);
    double b_ = line_k * xy_pos_offset.x() + line_b - xy_pos_offset.y(); // 直线在各自足端当前位置的坐标系中的纵截距（作为轮子到直线的距离）

    double raw_distance = b_;         // 如果是下降边缘，那么直接返回纵截距；如果是上升边缘，那么需要减去空隙的宽度

    while (raw_distance > 0.4 + 0.15) // 如果离检测到的边缘线距离过长，那么可能是后腿，按照台阶长度0.4来计算，
        raw_distance = raw_distance - 0.4 - 0.15;

    return raw_distance;
}

bool Brige_B::calc_edge_line(const robot_interfaces::msg::Brigeb& msg) {
    constexpr double kEps = 1e-8;

    lower_edge = false;

    try {
        auto transfer = robot->tf_buffer_->lookupTransform("body_link", "camera_link", tf2::TimePointZero);

        // Manually construct tf2::Transform from geometry_msgs::TransformStamped
        tf2::Vector3 translation(transfer.transform.translation.x, 
                                  transfer.transform.translation.y, 
                                  transfer.transform.translation.z);
        tf2::Quaternion rotation(transfer.transform.rotation.x,
                                 transfer.transform.rotation.y,
                                 transfer.transform.rotation.z,
                                 transfer.transform.rotation.w);
        tf2::Transform camera_to_body(rotation, translation);

        const tf2::Vector3 point_in_camera(msg.point.x, msg.point.y, msg.point.z);
        const tf2::Vector3 direction_in_camera(msg.direction.x, msg.direction.y, msg.direction.z);

        const tf2::Vector3 point_in_body     = camera_to_body * point_in_camera;
        const tf2::Vector3 direction_in_body = camera_to_body.getBasis() * direction_in_camera;

        const double dx = direction_in_body.x();
        const double dy = direction_in_body.y();

        if (std::hypot(dx, dy) < kEps) {
            return false;
        }

        if (std::abs(dx) < kEps) {
            return false;
        }

        line_k                = dy / dx;
        line_b                = point_in_body.y() - line_k * point_in_body.x();
        lower_edge            = true;
        last_line_update_time = robot->node_->get_clock()->now();
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(robot->node_->get_logger(), "计算边缘线失败: %s", ex.what());
        return false;
    }
}


std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
    Brige_B::balance_force_calc(Robot* robot, double cur_roll, double cur_pitch) {

    double sin_pitch = std::sin(cur_pitch);
    double sin_roll  = std::sin(cur_roll);

    double roll_offset_virtual_torque  = robot->roll_vmc->update(cur_roll, robot->robot_velocity.angular.x, 0.0);
    double pitch_offset_virtual_torque = robot->pitch_vmc->update(cur_pitch, robot->robot_velocity.angular.y, 0.0);

    // TODO:计算四个足端的期望的平衡虚拟力(pitch)
    Eigen::Vector3d lf_force = Eigen::Vector3d::Zero(), rf_force = Eigen::Vector3d::Zero(), lb_force = Eigen::Vector3d::Zero(),
                    rb_force = Eigen::Vector3d::Zero();
    lf_force[2] += pitch_offset_virtual_torque * robot->lf_leg_calc->pos_offset[0];
    rf_force[2] += pitch_offset_virtual_torque * robot->rf_leg_calc->pos_offset[0];
    lb_force[2] += pitch_offset_virtual_torque * robot->lb_leg_calc->pos_offset[0];
    rb_force[2] += pitch_offset_virtual_torque * robot->rb_leg_calc->pos_offset[0];

    // lf_force[0] += pitch_offset_virtual_torque * sin_pitch*pitch_balance_force_compen;
    // rf_force[0] += pitch_offset_virtual_torque * sin_pitch * pitch_balance_force_compen;
    // lb_force[0] += pitch_offset_virtual_torque * sin_pitch * pitch_balance_force_compen;
    // rb_force[0] += pitch_offset_virtual_torque * sin_pitch * pitch_balance_force_compen;

    // TODO:计算四个足端的期望的平衡虚拟力(roll)
    lf_force[2] += roll_offset_virtual_torque * robot->lf_leg_calc->pos_offset[1];
    rf_force[2] += roll_offset_virtual_torque * robot->rf_leg_calc->pos_offset[1];
    lb_force[2] += roll_offset_virtual_torque * robot->lb_leg_calc->pos_offset[1];
    rb_force[2] += roll_offset_virtual_torque * robot->rb_leg_calc->pos_offset[1];

    // lf_force[1] += roll_offset_virtual_torque * sin_roll * roll_balance_force_compen;
    // rf_force[1] += roll_offset_virtual_torque * sin_roll * roll_balance_force_compen;
    // lb_force[1] += roll_offset_virtual_torque * sin_roll * roll_balance_force_compen;
    // rb_force[1] += roll_offset_virtual_torque * sin_roll * roll_balance_force_compen;
    return {lf_force, rf_force, lb_force, rb_force};
}
