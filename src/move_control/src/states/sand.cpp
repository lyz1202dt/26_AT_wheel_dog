#include "states/sand.hpp"
#include "core/robot.hpp"
#include <rclcpp/logging.hpp>


double Time_body = 4.0;
SandState::SandState(Robot* robot)
    : BaseState<Robot>("sand") {
    (void)robot;
    steady_clock = robot->node_->get_clock();//std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    mass = (robot->robot_lf_grivate + robot->robot_rf_grivate + robot->robot_lb_grivate + robot->robot_rb_grivate) / 9.8;
    mass_center_pos =
        Vector2D(
            robot->robot_lf_grivate * robot->lf_leg_calc->pos_offset[0] + robot->robot_rf_grivate * robot->rf_leg_calc->pos_offset[0]
                + robot->robot_lb_grivate * robot->lb_leg_calc->pos_offset[0] + robot->robot_rb_grivate * robot->rb_leg_calc->pos_offset[0],
            robot->robot_lf_grivate * robot->lf_leg_calc->pos_offset[1] + robot->robot_rf_grivate * robot->rf_leg_calc->pos_offset[1]
                + robot->robot_lb_grivate * robot->lb_leg_calc->pos_offset[1] + robot->robot_rb_grivate * robot->rb_leg_calc->pos_offset[1])
        / (mass * 9.8);
}

bool SandState::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    step_state = 0;
    start_time = sand_now();

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
    last_pos_1        = Vector3D::Zero();
    last_pos_2        = Vector3D::Zero();
    sand_first = true;
    return true;
}





std::string SandState::update(Robot* robot) {
    std::string next_state("sand");
    auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
    auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
    auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
    auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

    lf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lf_grivate);
    rf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rf_grivate);
    lb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lb_grivate);
    rb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rb_grivate);

    // 位控站立状态
    if (step_state == 0) {
        if (std::abs(robot->move_cmd.vx) > 0.1 || std::abs(robot->move_cmd.vy) > 0.1) {
            step_state = 1;
        }
    }
    if (step_state == 1) {
        RCLCPP_INFO(robot->node_->get_logger(), "狗身向右平移");
        double x_target = -(0.5 * (last_pos_1[0] + last_pos_2[0])), y_target = step_dy, f_z_target = 0.0, b_z_target = 0.0;
        if(sand_first) {
            f_z_target = 0.0;
            b_z_target = 0.0;
        }else {
            f_z_target = 0.08;
            b_z_target = 0.0;
        }
            
        lf_step.update_support_trajectory(lf_cart_pos, Vector3D(x_target, y_target, f_z_target), Time_body);
        rf_step.update_support_trajectory(rf_cart_pos, Vector3D(0.0, y_target, f_z_target), Time_body);
        lb_step.update_support_trajectory(lb_cart_pos, Vector3D(x_target, y_target, b_z_target), Time_body);
        rb_step.update_support_trajectory(rb_cart_pos, Vector3D(0.0, y_target, b_z_target), Time_body);
        start_time = sand_now();
        step_state = 2;
    }
    if (step_state == 2) {                                                        // 身体向右侧平移
        bool success;
        double t = (sand_now() - start_time).seconds();

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_step.get_target(t, success);
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
            step_state = 3;
    }
    if (step_state == 3) {
        RCLCPP_INFO(robot->node_->get_logger(), "左后腿向前摆动");
        Vector3D next_available_pos = get_next_available_pos(robot, robot->lb_leg_calc->pos_offset, lb_cart_pos);
        last_pos_1                  = next_available_pos;
        lb_leg_step.update_flight_trajectory(lb_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 3.0, 0.20);
        start_time = sand_now();
        step_state = 4;
    }
    if (step_state == 4) // 左后腿向前摆动
    {
        bool success;
        double t = (sand_now() - start_time).seconds();

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
            step_state = 5;
    }
    if (step_state == 5) {
        RCLCPP_INFO(robot->node_->get_logger(), "左前腿向前摆动");
        Vector3D next_available_pos = get_next_available_pos(robot, robot->lf_leg_calc->pos_offset, lf_cart_pos);
        last_pos_2                  = next_available_pos;
        lf_leg_step.update_flight_trajectory(lf_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 3.0, 0.20);
        start_time = sand_now();
        step_state = 6;
    }
    if (step_state == 6)                      // 左前腿向前摆动
    {
        bool success;
        double t = (sand_now() - start_time).seconds();

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
            step_state = 7;
    }
    if (step_state == 7) {
        RCLCPP_INFO(robot->node_->get_logger(), "狗身向左平移");
        double x_target = -(0.5 * (last_pos_1[0] + last_pos_2[0])), y_target = -step_dy, f_z_target = 0.0, b_z_target = 0.0, lb_z_target = 0.0;
        if(sand_first) {
            f_z_target = 0.08;
            lb_z_target = b_z_target = 0.0;
        }else {
            f_z_target = 0.08;
            b_z_target = 0.0;
            lb_z_target = 0.08;
        }
            
        lf_step.update_support_trajectory(lf_cart_pos, Vector3D(0.0, y_target, f_z_target), Time_body);
        rf_step.update_support_trajectory(rf_cart_pos, Vector3D(x_target, y_target, f_z_target), Time_body);
        lb_step.update_support_trajectory(lb_cart_pos, Vector3D(0.0, y_target, lb_z_target), Time_body);
        rb_step.update_support_trajectory(rb_cart_pos, Vector3D(x_target, y_target, b_z_target), Time_body);
        start_time = sand_now();
        step_state = 8;
    }
    if (step_state == 8)                      // 整体向左前方移动
    {
        bool success;
        double t = (sand_now() - start_time).seconds();

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_step.get_target(t, success);

        // 计算每个腿要承担的垂直力
        auto lf_pos = (lf_foot_exp_pos + robot->lf_leg_calc->pos_offset).head(2);
        auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

        // 构建方程组: A * f = b
        // 约束条件与step_state==2相同
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
            step_state = 9;
    }
    if (step_state == 9) {
        RCLCPP_INFO(robot->node_->get_logger(), "右后腿向前摆动");
        Vector3D next_available_pos = get_next_available_pos(robot, robot->rb_leg_calc->pos_offset, rb_cart_pos);
        last_pos_1                  = next_available_pos;
        rb_leg_step.update_flight_trajectory(rb_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 3.0, 0.20);
        start_time = sand_now();
        step_state = 10;
    }
    if (step_state == 10) // 右后腿向前摆动
    {
        bool success;
        double t                                                    = (sand_now() - start_time).seconds();
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
            step_state = 11;
    }
    if (step_state == 11) {
        RCLCPP_INFO(robot->node_->get_logger(), "右前腿向前摆动");
        Vector3D next_available_pos = get_next_available_pos(robot, robot->rf_leg_calc->pos_offset, rf_cart_pos);
        last_pos_2                  = next_available_pos;
        rf_leg_step.update_flight_trajectory(rf_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 3.0, 0.20);
        start_time = sand_now();
        step_state = 12;
    }
    if (step_state == 12)                     // 右前腿向前摆动
    {
        bool success;
        double t                                                    = (sand_now() - start_time).seconds();
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
        rf_foot_exp_force = Vector3D::Zero();                                     // 摆动腿无支撑力
        lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
        rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));

        if (!success) {
            if(sand_first)
            {
                sand_first = false;
                step_state = 0;
            }else
                step_state = 13;
        }
    }

    if (step_state == 13) {
        RCLCPP_INFO(robot->node_->get_logger(), "准备停止stop");
        lf_step.update_support_trajectory(lf_cart_pos, Vector3D(0.0, 0.0, 0.0), Time_body);
        rf_step.update_support_trajectory(rf_cart_pos, Vector3D(0.0, 0.0, 0.0), Time_body);
        lb_step.update_support_trajectory(lb_cart_pos, Vector3D(0.0, 0.0, 0.0), Time_body);
        rb_step.update_support_trajectory(rb_cart_pos, Vector3D(0.0, 0.0, 0.0), Time_body);
        start_time = sand_now();
        step_state = 14;
    }

    if (step_state == 14) {
        bool success;
        double t                                                    = (sand_now() - start_time).seconds();
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_step.get_target(t, success);
        auto lf_pos = (lf_foot_exp_pos + robot->lf_leg_calc->pos_offset).head(2); // 在简化的二维平面模型中，提供支撑力的位置
        auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

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

        if (!success) {
            auto now                = robot->node_->get_clock()->now();
            main_phrase_start_time  = now;
            slave_phrase_start_time = now;
            slave_phrase_stop_time =
                now
                + rclcpp::Duration(
                    std::chrono::duration<double>(
                        (std::abs(2.0 * step_support_rate - 1.0) * 0.5 + 1.0 - step_support_rate) * step_time)); // 预规划从相位支撑相结束时间
            lf_leg_step.update_flight_trajectory(
                robot->lf_leg_calc->foot_pos(robot->lf_joint_pos), Vector3D(0.0, 0.0, 0.0), lf_exp_vel, ((1.0 - step_support_rate) * step_time),
                step_height);
            rf_leg_step.update_support_trajectory(
                robot->rf_leg_calc->foot_pos(robot->rf_joint_pos), rf_exp_vel,
                (std::abs(2.0 * step_support_rate - 1.0) * 0.5 + 1.0 - step_support_rate) * step_time);
            lb_leg_step.update_support_trajectory(
                robot->lb_leg_calc->foot_pos(robot->lb_joint_pos), lb_exp_vel,
                (std::abs(2.0 * step_support_rate - 1.0) * 0.5 + 1.0 - step_support_rate) * step_time);
            rb_leg_step.update_flight_trajectory(
                robot->lf_leg_calc->foot_pos(robot->lf_joint_pos), Vector3D(0.0, 0.0, 0.0), lf_exp_vel, ((1.0 - step_support_rate) * step_time),
                step_height);
            step1_support_updated = false;                                                                       // 设置足端轨迹更新状态
            step1_flight_updated  = true;
            step2_flight_updated  = false;
            step2_support_updated = true;
            step_state = 15;
        }
    }
    if (step_state == 15) {
        double cur_roll, cur_pitch, cur_yaw;
        tf2::Matrix3x3(robot->robot_rotation).getRPY(cur_roll, cur_pitch, cur_yaw);
        auto footstep_correction = [this, cur_pitch, cur_roll](const Vector3D& initial_target) -> Vector3D {
            Vector3D new_target = initial_target;
            new_target[0] += -(exp_pitch - cur_pitch) * pitch_balance_step_compen;
            new_target[1] += (exp_roll - cur_roll) * roll_balance_step_compen;
            // 限制落足点范围在边长为 0.24m 的正方形内 (原代码注释写 0.12/2 但变量名 max_offset=0.12，此处保持原逻辑)
            const double max_offset = 0.12;
            new_target[0]           = std::clamp(new_target[0], initial_target[0] - max_offset, initial_target[0] + max_offset);
            new_target[1]           = std::clamp(new_target[1], initial_target[1] - max_offset, initial_target[1] + max_offset);
            return new_target;
        };
        std::tie(lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force) = balance_force_calc(robot, cur_roll, cur_pitch);

        if ((cur_roll > 40 * 3.14 / 180 || cur_roll < -40 * 3.14 / 180 || cur_pitch > 50 * 3.14 / 180
            || cur_pitch < -50 * 3.14 / 180)) // 机器人倾倒，切入IDEL状态
            return "idel";

        std::tie(lf_exp_vel, rf_exp_vel, lb_exp_vel, rb_exp_vel) =
            calc_foot_vel(robot, Vector3D(robot->move_cmd.vx, robot->move_cmd.vy, robot->move_cmd.vz));

        auto now = robot->node_->get_clock()->now();
        // TODO:利用LegStep类的轨迹计算是否成功的判据来决定是否开启
        if (step1_flight_updated && (!step1_support_updated)) {    // 处于足端飞行相
            if (now - main_phrase_start_time > rclcpp::Duration(
                    std::chrono::duration<double>(
                        (1.0 - step_support_rate) * step_time))) { // 如果主相位飞行相已经结束，那么立即规划主相位支撑相
                step1_support_updated = true;                      // 设置足端轨迹更新状态
                step1_flight_updated  = false;
                slave_phrase_stop_time =
                    now                                            // 从相位支撑相结束时间等于主相位飞行相结束时间+T*(2*α-1)/2
                    + rclcpp::Duration(std::chrono::duration<double>(std::abs(2.0 * step_support_rate - 1.0) * step_time * 0.5));

                // TODO:根据姿态更新足端中性点位置,求在平面上的投影与基偏移量叠加作为新的足端中性点
                auto vertical_v = Vector3D(0.0, 0.0, -robot->body_height); // 足端垂直向量
                tf2::Quaternion q;
                q.setRPY(cur_roll, cur_pitch, 0.0);
                Eigen::Quaterniond e_q(q);
                Eigen::Matrix3d R_mat   = e_q.toRotationMatrix().transpose();
                Vector3D rot_pos_offset = R_mat * vertical_v;
                rot_pos_offset[2]       = 0.0;

                // lf_leg_calc->pos_offset=lf_base_offset-rot_pos_offset;      //在规划轨迹前更改足端中性点，不会引起系统冲击
                // rb_leg_calc->pos_offset=rb_base_offset-rot_pos_offset;

                lf_leg_step.update_support_trajectory(
                    robot->lf_leg_calc->foot_pos(robot->lf_joint_pos), lf_exp_vel, step_support_rate * step_time);
                // 主相对角腿也需要同步进入支撑相（右后）
                rb_leg_step.update_support_trajectory(
                    robot->rb_leg_calc->foot_pos(robot->rb_joint_pos), rb_exp_vel, step_support_rate * step_time);
                main_phrase_start_time = now;
                RCLCPP_INFO(robot->node_->get_logger(), "主相位支撑相规划");
            }
        } else if (step1_support_updated && (!step1_flight_updated)) {               // 处于足端支撑相
            if (now - main_phrase_start_time > rclcpp::Duration(
                    std::chrono::duration<double>(step_support_rate) * step_time)) { // 如果主相位飞行相已经结束，那么立即规划主相位飞行相
                step1_support_updated = false;                                       // 设置足端轨迹更新状态
                step1_flight_updated  = true;


                // 使用带回调函数的重载版本，在飞行到中点时重新规划落足点
                lf_leg_step.update_flight_trajectory(
                    robot->lf_leg_calc->foot_pos(robot->lf_joint_pos), -Vector3D(lf_exp_vel[0], lf_exp_vel[1], 0.0), lf_exp_vel,
                    step_time * (1.0 - step_support_rate), step_height, footstep_correction);
                // 主相对角腿也需要规划飞行轨迹（右后）
                rb_leg_step.update_flight_trajectory(                           
                    robot->rb_leg_calc->foot_pos(robot->rb_joint_pos), -Vector3D(rb_exp_vel[0], rb_exp_vel[1], 0.0), rb_exp_vel,
                    step_time * (1.0 - step_support_rate), step_height, footstep_correction);
                // lf_leg_step.update_flight_trajectory(
                //     robot->lf_leg_calc->foot_pos(robot->lf_joint_pos), -Vector3D(lf_exp_vel[0], lf_exp_vel[1], 0.0), lf_exp_vel,
                //     step_time * (1.0 - step_support_rate), step_height);
                // // 主相对角腿也需要规划飞行轨迹（右后）
                // rb_leg_step.update_flight_trajectory(
                //     robot->rb_leg_calc->foot_pos(robot->rb_joint_pos), -Vector3D(rb_exp_vel[0], rb_exp_vel[1], 0.0), rb_exp_vel,
                //     step_time * (1.0 - step_support_rate), step_height);
                main_phrase_start_time = now;
                RCLCPP_INFO(robot->node_->get_logger(), "主相位摆动相规划");
            }
        }


        if (step2_flight_updated && (!step2_support_updated)) {    // 如果从相位处于飞行相
            if (now - slave_phrase_start_time > rclcpp::Duration(
                    std::chrono::duration<double>(
                        (1.0 - step_support_rate) * step_time))) { // 如果主相位飞行相已经结束，那么立即规划主相位支撑相
                step2_support_updated = true;                      // 设置足端轨迹更新状态
                step2_flight_updated  = false;
                // TODO:根据姿态更新足端中性点位置

                auto vertical_v = Vector3D(0.0, 0.0, -robot->body_height); // 足端垂直向量
                tf2::Quaternion q;
                q.setRPY(cur_roll, cur_pitch, 0.0);
                Eigen::Quaterniond e_q(q);
                Eigen::Matrix3d R_mat   = e_q.toRotationMatrix().transpose();
                Vector3D rot_pos_offset = R_mat * vertical_v;
                rot_pos_offset[2]       = 0.0;

                // rf_leg_calc->pos_offset=rf_base_offset-rot_pos_offset;      //在规划轨迹前更改足端中性点，不会引起系统冲击
                // lb_leg_calc->pos_offset=lb_base_offset-rot_pos_offset;


                rf_leg_step.update_support_trajectory(
                    robot->rf_leg_calc->foot_pos(robot->rf_joint_pos), rf_exp_vel,
                    step_support_rate * step_time); // 预更新支撑相(精确结束时间由主相位确定)
                // 从相对角腿也同步进入支撑相（左后）
                lb_leg_step.update_support_trajectory(
                    robot->lb_leg_calc->foot_pos(robot->lb_joint_pos), lb_exp_vel, step_support_rate * step_time);
                slave_phrase_start_time = now;
                slave_phrase_stop_time  = now + rclcpp::Duration(std::chrono::duration<double>(step_support_rate * step_time));
                if (robot->move_cmd.step_mode == 1) {                  // 请求状态为停止，那么记录足端停下的位置，然后请求跳转到stop
                    robot->lf_leg_stop_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
                    robot->rf_leg_stop_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
                    robot->lb_leg_stop_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
                    robot->rb_leg_stop_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
                    next_state             = "stop";
                }
                RCLCPP_INFO(robot->node_->get_logger(), "从相位支撑相规划");
            }
        } else if (step2_support_updated && (!step2_flight_updated)) { // 如果从相位处于支撑相(调相位)
            if (now > slave_phrase_stop_time)                          // 如果到达了由主相位确定的从相位支撑相结束时间，那么更新从相位飞行相
            {
                step2_support_updated = false;                         // 设置足端轨迹更新状态
                step2_flight_updated  = true;

                // 使用带回调函数的重载版本，在飞行到中点时重新规划落足点
                rf_leg_step.update_flight_trajectory(
                    robot->rf_leg_calc->foot_pos(robot->rf_joint_pos), -Vector3D(rf_exp_vel[0], rf_exp_vel[1], 0.0), rf_exp_vel,
                    (1.0 - step_support_rate) * step_time, step_height, footstep_correction);
                lb_leg_step.update_flight_trajectory(
                    robot->lb_leg_calc->foot_pos(robot->lb_joint_pos), -Vector3D(lb_exp_vel[0], lb_exp_vel[1], 0.0), lb_exp_vel,
                    (1.0 - step_support_rate) * step_time, step_height, footstep_correction);

                // 从相两条腿同时进入飞行相（右前 & 左后）
                // rf_leg_step.update_flight_trajectory(
                //     robot->rf_leg_calc->foot_pos(robot->rf_joint_pos), -Vector3D(rf_exp_vel[0], rf_exp_vel[1], 0.0), rf_exp_vel,
                //     (1.0 - step_support_rate) * step_time, step_height);
                // lb_leg_step.update_flight_trajectory(
                //     robot->lb_leg_calc->foot_pos(robot->lb_joint_pos), -Vector3D(lb_exp_vel[0], lb_exp_vel[1], 0.0), lb_exp_vel,
                //     (1.0 - step_support_rate) * step_time, step_height);
                slave_phrase_start_time = now;
                RCLCPP_INFO(robot->node_->get_logger(), "从相位摆动相规划");
            }
        }
        bool success[4];
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) =
            lf_leg_step.get_target((now - main_phrase_start_time).seconds(), success[0]); // 得到狗腿当前期望
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) =
            rf_leg_step.get_target((now - slave_phrase_start_time).seconds(), success[1]);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) =
            lb_leg_step.get_target((now - slave_phrase_start_time).seconds(), success[2]);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) =
            rb_leg_step.get_target((now - main_phrase_start_time).seconds(), success[3]);

        auto lf_cart_pos   = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto lf_cart_vel   = robot->lf_leg_calc->foot_vel(robot->lf_joint_pos, robot->lf_joint_vel);
        auto lf_cart_force = robot->lf_leg_calc->foot_force(robot->lf_joint_pos, robot->lf_joint_torque, robot->lf_forward_torque);
        auto rb_cart_pos   = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
        auto rb_cart_vel   = robot->rb_leg_calc->foot_vel(robot->rb_joint_pos, robot->rb_joint_vel);
        auto rb_cart_force = robot->rb_leg_calc->foot_force(robot->rb_joint_pos, robot->rb_joint_torque, robot->rb_forward_torque);
        auto rf_cart_pos   = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto rf_cart_vel   = robot->rf_leg_calc->foot_vel(robot->rf_joint_pos, robot->rf_joint_vel);
        auto rf_cart_force = robot->rf_leg_calc->foot_force(robot->rf_joint_pos, robot->rf_joint_torque, robot->rf_forward_torque);
        auto lb_cart_pos   = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto lb_cart_vel   = robot->lb_leg_calc->foot_vel(robot->lb_joint_pos, robot->lb_joint_vel);
        auto lb_cart_force = robot->lb_leg_calc->foot_force(robot->lb_joint_pos, robot->lb_joint_torque, robot->lb_forward_torque);

        if (step1_support_updated) {                                                      // 主相位处于支撑相
            std::tie(lf_foot_exp_pos[2], lf_foot_exp_vel[2], lf_foot_exp_acc[2]) =
                robot->lf_z_vmc->targetUpdate(lf_foot_exp_pos[2], lf_cart_pos[2], lf_foot_exp_vel[2], lf_cart_vel[2], -lf_cart_force[2]);

            std::tie(rb_foot_exp_pos[2], rb_foot_exp_vel[2], rb_foot_exp_acc[2]) =
                robot->rb_z_vmc->targetUpdate(rb_foot_exp_pos[2], rb_cart_pos[2], rb_foot_exp_vel[2], rb_cart_vel[2], -rb_cart_force[2]);

            if (step2_support_updated) { // 如果从相位也需要VMC计算，说明此时四足触底，每个脚的向下的力为一倍，否则为两倍
                lf_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_lf_grivate);
                rb_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_rb_grivate);
            } else {
                lf_foot_exp_force += Vector3D(0.0, 0.0, -2.0 * robot->robot_lf_grivate);
                rb_foot_exp_force += Vector3D(0.0, 0.0, -2.0 * robot->robot_rb_grivate);
            }

            std::tie(lf_foot_exp_pos[0], lf_foot_exp_vel[0], lf_foot_exp_acc[0]) = robot->lf_x_vmc->targetUpdate(
                lf_foot_exp_pos[0], lf_cart_pos[0], lf_foot_exp_vel[0], lf_cart_vel[0],
                -lf_cart_force[0]);      // 实际这个lf_cart_force是足端本身要施加的力，不是受到的力
            std::tie(lf_foot_exp_pos[1], lf_foot_exp_vel[1], lf_foot_exp_acc[1]) =
                robot->lf_y_vmc->targetUpdate(lf_foot_exp_pos[1], lf_cart_pos[1], lf_foot_exp_vel[1], lf_cart_vel[1], -lf_cart_force[1]);
            std::tie(rb_foot_exp_pos[0], rb_foot_exp_vel[0], rb_foot_exp_acc[0]) =
                robot->rb_x_vmc->targetUpdate(rb_foot_exp_pos[0], rb_cart_pos[0], rb_foot_exp_vel[0], rb_cart_vel[0], -rb_cart_force[0]);
            std::tie(rb_foot_exp_pos[1], rb_foot_exp_vel[1], rb_foot_exp_acc[1]) =
                robot->rb_y_vmc->targetUpdate(rb_foot_exp_pos[1], rb_cart_pos[1], rb_foot_exp_vel[1], rb_cart_vel[1], -rb_cart_force[1]);
        }
        if (step2_support_updated) {     // 从相位处于支撑相

            std::tie(rf_foot_exp_pos[2], rf_foot_exp_vel[2], rf_foot_exp_acc[2]) =
                robot->rf_z_vmc->targetUpdate(rf_foot_exp_pos[2], rf_cart_pos[2], rf_foot_exp_vel[2], rf_cart_vel[2], -rf_cart_force[2]);

            std::tie(lb_foot_exp_pos[2], lb_foot_exp_vel[2], lb_foot_exp_acc[2]) =
                robot->lb_z_vmc->targetUpdate(lb_foot_exp_pos[2], lb_cart_pos[2], lb_foot_exp_vel[2], lb_cart_vel[2], -lb_cart_force[2]);

            if (step1_support_updated) {
                rf_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_rf_grivate);
                lb_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_lb_grivate);
            } else {
                rf_foot_exp_force += Vector3D(0.0, 0.0, -2.0 * robot->robot_rf_grivate);
                lb_foot_exp_force += Vector3D(0.0, 0.0, -2.0 * robot->robot_lb_grivate);
            }

            std::tie(rf_foot_exp_pos[0], rf_foot_exp_vel[0], rf_foot_exp_acc[0]) =
                robot->rf_x_vmc->targetUpdate(rf_foot_exp_pos[0], rf_cart_pos[0], rf_foot_exp_vel[0], rf_cart_vel[0], -rf_cart_force[0]);
            std::tie(rf_foot_exp_pos[1], rf_foot_exp_vel[1], rf_foot_exp_acc[1]) =
                robot->rf_y_vmc->targetUpdate(rf_foot_exp_pos[1], rf_cart_pos[1], rf_foot_exp_vel[1], rf_cart_vel[1], -rf_cart_force[1]);
            std::tie(lb_foot_exp_pos[0], lb_foot_exp_vel[0], lb_foot_exp_acc[0]) =
                robot->lb_x_vmc->targetUpdate(lb_foot_exp_pos[0], lb_cart_pos[0], lb_foot_exp_vel[0], lb_cart_vel[0], -lb_cart_force[0]);
            std::tie(lb_foot_exp_pos[1], lb_foot_exp_vel[1], lb_foot_exp_acc[1]) =
                robot->lb_y_vmc->targetUpdate(lb_foot_exp_pos[1], lb_cart_pos[1], lb_foot_exp_vel[1], lb_cart_vel[1], -lb_cart_force[1]);
        }
    }

    robot_interfaces::msg::RobotTarget joints_target;
    joints_target.legs[0] = robot->lf_leg_calc->signal_leg_calc(
        lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, &robot->lf_forward_torque);
    joints_target.legs[1] = robot->rf_leg_calc->signal_leg_calc(
        rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, &robot->rf_forward_torque);
    joints_target.legs[2] = robot->lb_leg_calc->signal_leg_calc(
        lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, &robot->lb_forward_torque);
    joints_target.legs[3] = robot->rb_leg_calc->signal_leg_calc(
        rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, &robot->rb_forward_torque);
    robot->legs_target_pub->publish(joints_target);

    return next_state;
}

Vector3D SandState::get_next_available_pos(Robot* robot, Vector3D leg_offset, Vector3D current_pos) {
    (void)leg_offset;
    (void)current_pos;
    // RCLCPP_INFO_THROTTLE(
    //     robot->node_->get_logger(),
    //     *robot->node_->get_clock(),
    //     100,
    //     "\033[31mstep_state = %d, current_pos[1] = %.2f\033[0m",
    //     step_state,
    //     current_pos[1]
    // );
    // return {0.12, current_pos[1], 0.0};
    if(sand_first)
    {
        if(step_state == 3)
        {
            return {0.2, 0.08, 0.0};
        }else if(step_state == 9)
        {
            return {0.2, -0.08, 0.0};
        }else if(step_state == 5)
        {
            return {0.2, 0.08, 0.08};
        }else if(step_state == 11)
        {
            return {0.2, -0.08, 0.08};
        }
    }
    else
    {
        if(step_state == 3)
        {
            return {0.2, 0.08, 0.08};
        }else if(step_state == 9)
        {
            return {0.2, -0.08, 0.08};
        }else if(step_state == 5)
        {
            return {0.2, 0.08, 0.08};
        }else if(step_state == 11)
        {
            return {0.2, -0.08, 0.08};
        }
    }
}



void Sand_Step::update_support_trajectory(const Vector3D& cur_pos, const Vector3D final_pos, double time) {
    traj.time = time;
    T         = time;

    for (int i = 0; i < 3; i++) {
        double p0 = cur_pos[i];
        double pT = final_pos[i];

        // ⭐ Quintic：直接保证平滑
        double v0 = 0.0;
        double vT = 0.0;
        double a0 = 0.0;
        double aT = 0.0;

        if (i == 0)
            set_quintic(traj.x, p0, v0, a0, pT, vT, aT, time);
        else if (i == 1)
            set_quintic(traj.y, p0, v0, a0, pT, vT, aT, time);
        else
            set_quintic(traj.z, p0, v0, a0, pT, vT, aT, time);
    }
}

std::tuple<Vector3D, Vector3D, Vector3D> Sand_Step::get_target(double time, bool& success) {
    Vector3D pos, vel, acc;

    if (time >= T) {
        time    = T;
        success = false;
    } else {
        success = true;
    }

    pos[0] = get_quintic_value(traj.x, time);
    vel[0] = get_quintic_dt(traj.x, time);
    acc[0] = get_quintic_dtdt(traj.x, time);

    pos[1] = get_quintic_value(traj.y, time);
    vel[1] = get_quintic_dt(traj.y, time);
    acc[1] = get_quintic_dtdt(traj.y, time);

    pos[2] = get_quintic_value(traj.z, time);
    vel[2] = get_quintic_dt(traj.z, time);
    acc[2] = get_quintic_dtdt(traj.z, time);

    return {pos, vel, acc};
}


std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d>
    SandState::calc_foot_vel(Robot* robot, Eigen::Vector3d exp_vel) {
    Vector3D v_body(exp_vel[0], exp_vel[1], 0.0);
    Vector3D omega(0.0, 0.0, exp_vel[2]);

    // LF
    Vector3D v_lf   = v_body + omega.cross(robot->lf_leg_calc->pos_offset);
    auto lf_exp_vel = Vector2D(v_lf[0], v_lf[1]);

    // RF
    Vector3D v_rf   = v_body + omega.cross(robot->rf_leg_calc->pos_offset);
    auto rf_exp_vel = Vector2D(v_rf[0], v_rf[1]);

    // LB
    Vector3D v_lb   = v_body + omega.cross(robot->lb_leg_calc->pos_offset);
    auto lb_exp_vel = Vector2D(v_lb[0], v_lb[1]);

    // RB
    Vector3D v_rb   = v_body + omega.cross(robot->rb_leg_calc->pos_offset);
    auto rb_exp_vel = Vector2D(v_rb[0], v_rb[1]);

    return {lf_exp_vel, rf_exp_vel, lb_exp_vel, rb_exp_vel};
}


std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
    SandState::balance_force_calc(Robot* robot, double cur_roll, double cur_pitch) {

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