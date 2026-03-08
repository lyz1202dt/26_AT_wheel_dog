#include "states/amble.hpp"
#include "core/robot.hpp"


AmbleState::AmbleState(Robot* robot)
    : BaseState<Robot>("amble") {
    mass = (robot->robot_lf_grivate + robot->robot_rf_grivate + robot->robot_lb_grivate + robot->robot_rb_grivate) / 9.8;
    mass_center_pos =
        Vector2D(
            robot->robot_lf_grivate * robot->lf_leg_calc->pos_offset[0] + robot->robot_rf_grivate * robot->rf_leg_calc->pos_offset[0]
                + robot->robot_lb_grivate * robot->lb_leg_calc->pos_offset[0] + robot->robot_rb_grivate * robot->rb_leg_calc->pos_offset[0],
            robot->robot_lf_grivate * robot->lf_leg_calc->pos_offset[1] + robot->robot_rf_grivate * robot->rf_leg_calc->pos_offset[1]
                + robot->robot_lb_grivate * robot->lb_leg_calc->pos_offset[1] + robot->robot_rb_grivate * robot->rb_leg_calc->pos_offset[1])
        / (mass * 9.8);
    (void)robot;
}

bool AmbleState::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    step_state = 0;
    start_time = robot->node_->get_clock()->now();

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

    return true;
}


/*缓行步态流程：
1.狗身平移到y负位置
2.左后腿向前迈退
3.左前腿向前迈进
4.狗身从y负位置平移到y正位置
5.右后腿向前迈进
6.右前腿向前迈进
*/

std::string AmbleState::update(Robot* robot) {
    auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
    auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
    auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
    auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

    // 位控站立状态
    if (step_state == 0) {
        if (std::abs(robot->move_cmd.vx) > 0.1 || std::abs(robot->move_cmd.vy) > 0.1) {
            step_state = 1;
        }
    }
    if (step_state == 1) {
        RCLCPP_INFO(robot->node_->get_logger(), "狗身向右平移");
        double x_target = -(0.5 * (last_pos_1[0] + last_pos_2[0])), y_target = step_dy;
        lf_leg_step.update_support_trajectory(lf_cart_pos, Vector3D(x_target, y_target, 0.0), 2.0);
        rf_leg_step.update_support_trajectory(rf_cart_pos, Vector3D(0.0, y_target, 0.0), 2.0);
        lb_leg_step.update_support_trajectory(lb_cart_pos, Vector3D(x_target, y_target, 0.0), 2.0);
        rb_leg_step.update_support_trajectory(rb_cart_pos, Vector3D(0.0, y_target, 0.0), 2.0);
        start_time = robot->node_->get_clock()->now();
        step_state = 2;
    }
    if (step_state == 2) { // 身体向右侧平移
        bool success;
        double t = (robot->node_->get_clock()->now() - start_time).seconds();

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);   //在简化的二维平面模型中，提供支撑力的位置
        auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
        
        // 使用Eigen求解每条腿的支撑力
        // 构建方程组: A * f = b
        // 约束条件:
        // 1. 垂直力平衡: f_lf + f_rf + f_lb + f_rb = mg
        // 2. 绕质心的力矩平衡(x方向): (lf_pos.x - mass_center_pos.x)*f_lf + ... = 0
        // 3. 绕质心的力矩平衡(y方向): (lf_pos.y - mass_center_pos.y)*f_lf + ... = 0
        Eigen::Matrix<double, 3, 4> A;
        Eigen::Vector3d b;
        
        // 第一行: 力平衡约束
        A(0, 0) = 1.0;  // lf
        A(0, 1) = 1.0;  // rf
        A(0, 2) = 1.0;  // lb
        A(0, 3) = 1.0;  // rb
        b(0) = mass * 9.8;
        
        // 第二行: 绕质心的x方向力矩平衡
        A(1, 0) = lf_pos.x() - mass_center_pos.x();
        A(1, 1) = rf_pos.x() - mass_center_pos.x();
        A(1, 2) = lb_pos.x() - mass_center_pos.x();
        A(1, 3) = rb_pos.x() - mass_center_pos.x();
        b(1) = 0.0;
        
        // 第三行: 绕质心的y方向力矩平衡
        A(2, 0) = lf_pos.y() - mass_center_pos.y();
        A(2, 1) = rf_pos.y() - mass_center_pos.y();
        A(2, 2) = lb_pos.y() - mass_center_pos.y();
        A(2, 3) = rb_pos.y() - mass_center_pos.y();
        b(2) = 0.0;
        
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
        Vector3D next_available_pos = get_next_available_pos(robot->lb_leg_calc->pos_offset, lb_cart_pos);
        last_pos_1                  = next_available_pos;
        lb_leg_step.update_flight_trajectory(lb_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 2.0, 0.12);
        start_time = robot->node_->get_clock()->now();
        step_state = 4;
    }
    if (step_state == 4) // 左后腿向前摆动
    {
        bool success;
        double t = (robot->node_->get_clock()->now() - start_time).seconds();

        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        
        // 3足支撑力计算 (lf, rf, rb支撑，lb摆动)
        auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);
        auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
        auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
        
        Eigen::Matrix3d A;
        Eigen::Vector3d b;
        
        // 力平衡约束
        A(0, 0) = 1.0;  // lf
        A(0, 1) = 1.0;  // rf
        A(0, 2) = 1.0;  // rb
        b(0) = mass * 9.8;
        
        // 绕质心的x方向力矩平衡
        A(1, 0) = lf_pos.x() - mass_center_pos.x();
        A(1, 1) = rf_pos.x() - mass_center_pos.x();
        A(1, 2) = rb_pos.x() - mass_center_pos.x();
        b(1) = 0.0;
        
        // 绕质心的y方向力矩平衡
        A(2, 0) = lf_pos.y() - mass_center_pos.y();
        A(2, 1) = rf_pos.y() - mass_center_pos.y();
        A(2, 2) = rb_pos.y() - mass_center_pos.y();
        b(2) = 0.0;
        
        // 求解3x3方程组
        Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);
        
        lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
        rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
        lb_foot_exp_force = Vector3D::Zero();  // 摆动腿无支撑力
        rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));
        
        if (!success)
            step_state = 5;
    }
    if (step_state == 5) {
        RCLCPP_INFO(robot->node_->get_logger(), "左后腿向前摆动");
        Vector3D next_available_pos = get_next_available_pos(robot->lf_leg_calc->pos_offset, lf_cart_pos);
        last_pos_2                  = next_available_pos;
        lf_leg_step.update_flight_trajectory(lf_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 2.0, 0.12);
        start_time = robot->node_->get_clock()->now();
        step_state = 6;
    }
    if (step_state == 6) // 左前腿向前摆动
    {
        bool success;
        double t = (robot->node_->get_clock()->now() - start_time).seconds();

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        
        // 3足支撑力计算 (rf, lb, rb支撑，lf摆动)
        auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
        
        Eigen::Matrix3d A;
        Eigen::Vector3d b;
        
        // 力平衡约束
        A(0, 0) = 1.0;  // rf
        A(0, 1) = 1.0;  // lb
        A(0, 2) = 1.0;  // rb
        b(0) = mass * 9.8;
        
        // 绕质心的x方向力矩平衡
        A(1, 0) = rf_pos.x() - mass_center_pos.x();
        A(1, 1) = lb_pos.x() - mass_center_pos.x();
        A(1, 2) = rb_pos.x() - mass_center_pos.x();
        b(1) = 0.0;
        
        // 绕质心的y方向力矩平衡
        A(2, 0) = rf_pos.y() - mass_center_pos.y();
        A(2, 1) = lb_pos.y() - mass_center_pos.y();
        A(2, 2) = rb_pos.y() - mass_center_pos.y();
        b(2) = 0.0;
        
        // 求解3x3方程组
        Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);
        
        lf_foot_exp_force = Vector3D::Zero();  // 摆动腿无支撑力
        rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
        lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
        rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));
        
        if (!success)
            step_state = 7;
    }
    if (step_state == 7) {
        RCLCPP_INFO(robot->node_->get_logger(), "狗身向左平移");
        double x_target = -(0.5 * (last_pos_1[0] + last_pos_2[0])), y_target = -step_dy;
        lf_leg_step.update_support_trajectory(lf_cart_pos, Vector3D(0.0, y_target, 0.0), 2.0);
        rf_leg_step.update_support_trajectory(rf_cart_pos, Vector3D(x_target, y_target, 0.0), 2.0);
        lb_leg_step.update_support_trajectory(lb_cart_pos, Vector3D(0.0, y_target, 0.0), 2.0);
        rb_leg_step.update_support_trajectory(rb_cart_pos, Vector3D(x_target, y_target, 0.0), 2.0);
        start_time = robot->node_->get_clock()->now();
        step_state = 8;
    }
    if (step_state == 8) // 整体向左前方移动
    {
        bool success;
        double t = (robot->node_->get_clock()->now() - start_time).seconds();

        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        
        // 计算每个腿要承担的垂直力
        auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);
        auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
        
        // 构建方程组: A * f = b
        // 约束条件与step_state==2相同
        Eigen::Matrix<double, 3, 4> A;
        Eigen::Vector3d b;
        
        // 第一行: 力平衡约束
        A(0, 0) = 1.0;  // lf
        A(0, 1) = 1.0;  // rf
        A(0, 2) = 1.0;  // lb
        A(0, 3) = 1.0;  // rb
        b(0) = mass * 9.8;
        
        // 第二行: 绕质心的x方向力矩平衡
        A(1, 0) = lf_pos.x() - mass_center_pos.x();
        A(1, 1) = rf_pos.x() - mass_center_pos.x();
        A(1, 2) = lb_pos.x() - mass_center_pos.x();
        A(1, 3) = rb_pos.x() - mass_center_pos.x();
        b(1) = 0.0;
        
        // 第三行: 绕质心的y方向力矩平衡
        A(2, 0) = lf_pos.y() - mass_center_pos.y();
        A(2, 1) = rf_pos.y() - mass_center_pos.y();
        A(2, 2) = lb_pos.y() - mass_center_pos.y();
        A(2, 3) = rb_pos.y() - mass_center_pos.y();
        b(2) = 0.0;
        
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
        Vector3D next_available_pos = get_next_available_pos(robot->rb_leg_calc->pos_offset, rb_cart_pos);
        last_pos_1                  = next_available_pos;
        rb_leg_step.update_flight_trajectory(rb_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 2.0, 0.12);
        start_time = robot->node_->get_clock()->now();
        step_state = 10;
    }
    if (step_state == 10) // 右后腿向前摆动
    {
        bool success;
        double t = (robot->node_->get_clock()->now() - start_time).seconds();
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
        
        // 3足支撑力计算 (lf, rf, lb支撑，rb摆动)
        auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);
        auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
        auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
        
        Eigen::Matrix3d A;
        Eigen::Vector3d b;
        
        // 力平衡约束
        A(0, 0) = 1.0;  // lf
        A(0, 1) = 1.0;  // rf
        A(0, 2) = 1.0;  // lb
        b(0) = mass * 9.8;
        
        // 绕质心的x方向力矩平衡
        A(1, 0) = lf_pos.x() - mass_center_pos.x();
        A(1, 1) = rf_pos.x() - mass_center_pos.x();
        A(1, 2) = lb_pos.x() - mass_center_pos.x();
        b(1) = 0.0;
        
        // 绕质心的y方向力矩平衡
        A(2, 0) = lf_pos.y() - mass_center_pos.y();
        A(2, 1) = rf_pos.y() - mass_center_pos.y();
        A(2, 2) = lb_pos.y() - mass_center_pos.y();
        b(2) = 0.0;
        
        // 求解3x3方程组
        Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);
        
        lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
        rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
        lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));
        rb_foot_exp_force = Vector3D::Zero();  // 摆动腿无支撑力
        
        if (!success)
            step_state = 11;
    }
    if (step_state == 11) {
        RCLCPP_INFO(robot->node_->get_logger(), "右前腿向前摆动");
        Vector3D next_available_pos = get_next_available_pos(robot->rf_leg_calc->pos_offset, rf_cart_pos);
        last_pos_2                  = next_available_pos;
        rf_leg_step.update_flight_trajectory(rf_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 2.0, 0.12);
        start_time = robot->node_->get_clock()->now();
        step_state = 12;
    }
    if (step_state == 12) // 右前腿向前摆动
    {
        bool success;
        double t = (robot->node_->get_clock()->now() - start_time).seconds();
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
        
        // 3足支撑力计算 (lf, lb, rb支撑，rf摆动)
        auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);
        auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
        auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
        
        Eigen::Matrix3d A;
        Eigen::Vector3d b;
        
        // 力平衡约束
        A(0, 0) = 1.0;  // lf
        A(0, 1) = 1.0;  // lb
        A(0, 2) = 1.0;  // rb
        b(0) = mass * 9.8;
        
        // 绕质心的x方向力矩平衡
        A(1, 0) = lf_pos.x() - mass_center_pos.x();
        A(1, 1) = lb_pos.x() - mass_center_pos.x();
        A(1, 2) = rb_pos.x() - mass_center_pos.x();
        b(1) = 0.0;
        
        // 绕质心的y方向力矩平衡
        A(2, 0) = lf_pos.y() - mass_center_pos.y();
        A(2, 1) = lb_pos.y() - mass_center_pos.y();
        A(2, 2) = rb_pos.y() - mass_center_pos.y();
        b(2) = 0.0;
        
        // 求解3x3方程组
        Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);
        
        lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
        rf_foot_exp_force = Vector3D::Zero();  // 摆动腿无支撑力
        lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
        rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));
        
        if (!success) {
            step_state = 0;
            if (robot->move_cmd.step_mode == 1)
                return "stop";
        }
    }

    robot_interfaces::msg::Robot joints_target;
    joints_target.legs[0] = robot->signal_leg_calc(
        lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, robot->lf_leg_calc, &robot->lf_forward_torque);
    joints_target.legs[1] = robot->signal_leg_calc(
        rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, robot->rf_leg_calc, &robot->rf_forward_torque);
    joints_target.legs[2] = robot->signal_leg_calc(
        lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, robot->lb_leg_calc, &robot->lb_forward_torque);
    joints_target.legs[3] = robot->signal_leg_calc(
        rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, robot->rb_leg_calc, &robot->rb_forward_torque);
    robot->legs_target_pub->publish(joints_target);

    return "amble";
}

Vector3D AmbleState::get_next_available_pos(Vector3D leg_offset, Vector3D current_pos) {
    (void)leg_offset;
    (void)current_pos;
    return {0.12, current_pos[1], 0.0};
}
