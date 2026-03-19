#include "states/climb_steps.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>

ClimbStepstate::ClimbStepstate(Robot* robot)
    : BaseState<Robot>("climb_steps") {
    (void)robot;

    robot->node_->declare_parameter("climb_step_finished_idel_time", 0.5);
    robot->node_->declare_parameter("climb_step_exp_pitch",0.0);

    robot->add_param_cb([this](const rclcpp::Parameter& param) {
        if (param.get_name() == "climb_step_finished_idel_time") {
            climb_step_finished_idel_time = param.as_double();
        }
        else if (param.get_name() == "climb_step_exp_pitch") {
            exp_pitch = param.as_double();
        }
        return true;
    });

    mass = (robot->robot_lf_grivate + robot->robot_rf_grivate + robot->robot_lb_grivate + robot->robot_rb_grivate) / 9.8;
    mass_center_pos =
        Vector2D(
            robot->robot_lf_grivate * robot->lf_leg_calc->pos_offset[0] + robot->robot_rf_grivate * robot->rf_leg_calc->pos_offset[0]
                + robot->robot_lb_grivate * robot->lb_leg_calc->pos_offset[0] + robot->robot_rb_grivate * robot->rb_leg_calc->pos_offset[0],
            robot->robot_lf_grivate * robot->lf_leg_calc->pos_offset[1] + robot->robot_rf_grivate * robot->rf_leg_calc->pos_offset[1]
                + robot->robot_lb_grivate * robot->lb_leg_calc->pos_offset[1] + robot->robot_rb_grivate * robot->rb_leg_calc->pos_offset[1])
        / (mass * 9.8);
}

bool ClimbStepstate::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    RCLCPP_INFO(robot->node_->get_logger(), "进入滑行/上台阶模式");
    req_state     = 0;
    current_state = 0;

    // Initialize time variables with the node's clock to ensure consistent time sources
    auto now                    = robot->node_->get_clock()->now();
    foot_climbing_time          = now;
    last_foot_climbing_end_time = now;
    main_phrase_start_time      = now;
    slave_phrase_start_time     = now;
    slave_phrase_stop_time      = now;

    rb_force_filter = lb_force_filter = rf_force_filter = lf_force_filter = Eigen::Vector3d::Zero();
    first_step                                                            = true;
    foot_climbing_step                                                    = 0;
    action_foot_num                                                       = 0;
    action_step                                                           = 0;
    flight_foot_num                                                       = 0;
    last_pos_1                                                            = Vector3D::Zero();
    last_pos_2                                                            = Vector3D::Zero();

    last_lf_pos = Vector3D(0.0, 0.0, 0.0);
    last_rf_pos = Vector3D(0.0, 0.0, 0.0);
    last_lb_pos = Vector3D(0.0, 0.0, 0.0);
    last_rb_pos = Vector3D(0.0, 0.0, 0.0);

    robot->node_->get_parameter("climb_step_finished_idel_time", climb_step_finished_idel_time);

    return true;
}

std::string ClimbStepstate::update(Robot* robot) {
    // 估计机器人本体速度
    // 如果某个足端处于摆动相(由foot_climbing_step指定), 则不应将该足端对应的轮子速度纳入速度估计
    double vel_sum = 0.0;
    int vel_count  = 0;
    // 左前
    if (foot_climbing_step != 1) {
        vel_sum += robot->lf_wheel_omega;
        vel_count++;
    }
    // 右前 (符号与原公式一致为负)
    if (foot_climbing_step != 2) {
        vel_sum -= robot->rf_wheel_omega;
        vel_count++;
    }
    // 左后
    if (foot_climbing_step != 3) {
        vel_sum += robot->lb_wheel_omega;
        vel_count++;
    }
    // 右后
    if (foot_climbing_step != 4) {
        vel_sum -= robot->rb_wheel_omega;
        vel_count++;
    }
    if (vel_count > 0) {
        current_body_vel = (vel_sum / static_cast<double>(vel_count)) * robot->WHEEL_RADIUS;
    } else {
        // 所有足端都在摆动相, 无法估计, 置零
        current_body_vel = 0.0;
    }

    double acc                    = exp_vel_kp * (robot->move_cmd.vx - current_body_vel);
    current_exp_vel               = current_exp_vel + acc * 0.002;
    double current_exp_foot_force = robot->robot_mass * acc * 0.25;

    double lf_wheel_vel = current_exp_vel, rf_wheel_vel = -current_exp_vel, lb_wheel_vel = current_exp_vel, rb_wheel_vel = -current_exp_vel;
    double lf_wheel_force = current_exp_foot_force, rf_wheel_force = -current_exp_foot_force, lb_wheel_force = current_exp_foot_force,
           rb_wheel_force = -current_exp_foot_force;


    Eigen::Vector3d lf_foot_exp_pos = Eigen::Vector3d::Zero(), rf_foot_exp_pos = Eigen::Vector3d::Zero(),
                    lb_foot_exp_pos = Eigen::Vector3d::Zero(), rb_foot_exp_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d lf_foot_exp_force = Eigen::Vector3d::Zero(), rf_foot_exp_force = Eigen::Vector3d::Zero(),
                    lb_foot_exp_force = Eigen::Vector3d::Zero(), rb_foot_exp_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d lf_foot_exp_vel = Eigen::Vector3d::Zero(), rf_foot_exp_vel = Eigen::Vector3d::Zero(),
                    lb_foot_exp_vel = Eigen::Vector3d::Zero(), rb_foot_exp_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d lf_foot_exp_acc = Eigen::Vector3d::Zero(), rf_foot_exp_acc = Eigen::Vector3d::Zero(),
                    lb_foot_exp_acc = Eigen::Vector3d::Zero(), rb_foot_exp_acc = Eigen::Vector3d::Zero();

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

    lf_force_filter = 0.9 * lf_force_filter + 0.1 * lf_cart_force;
    rf_force_filter = 0.9 * rf_force_filter + 0.1 * rf_cart_force;
    lb_force_filter = 0.9 * lb_force_filter + 0.1 * lb_cart_force;
    rb_force_filter = 0.9 * rb_force_filter + 0.1 * rb_cart_force;


    double cur_roll, cur_pitch, cur_yaw;
    tf2::Matrix3x3(robot->robot_rotation).getRPY(cur_roll, cur_pitch, cur_yaw);
    std::tie(lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force) = balance_force_calc(robot, cur_roll, cur_pitch,0.0,exp_pitch);

    if ((cur_roll > 40 * 3.14 / 180 || cur_roll < -40 * 3.14 / 180 || cur_pitch > 50 * 3.14 / 180
         || cur_pitch < -50 * 3.14 / 180))                 // 机器人倾倒，切入IDEL状态
        return "idel";

    lf_foot_exp_pos = robot->lf_leg_stop_pos;
    rf_foot_exp_pos = robot->rf_leg_stop_pos;
    lb_foot_exp_pos = robot->lb_leg_stop_pos;
    rb_foot_exp_pos = robot->rb_leg_stop_pos;


    robot_interfaces::msg::RobotTarget joints_target;


    Vector2D lf_exp_vel, rf_exp_vel, lb_exp_vel, rb_exp_vel;
    if (std::abs(robot->move_cmd.vz) > vz_dead_range || std::abs(robot->move_cmd.vy) > vy_dead_range) {
        Vector3D v_body(0.0, robot->move_cmd.vy, 0.0);
        Vector3D omega(0.0, 0.0, robot->move_cmd.vz);
        Vector3D v_lf = v_body + omega.cross(robot->lf_leg_calc->pos_offset);
        lf_exp_vel    = Vector2D(v_lf[0], v_lf[1]);
        Vector3D v_rf = v_body + omega.cross(robot->rf_leg_calc->pos_offset);
        rf_exp_vel    = Vector2D(v_rf[0], v_rf[1]);
        Vector3D v_lb = v_body + omega.cross(robot->lb_leg_calc->pos_offset);
        lb_exp_vel    = Vector2D(v_lb[0], v_lb[1]);
        Vector3D v_rb = v_body + omega.cross(robot->rb_leg_calc->pos_offset);
        rb_exp_vel    = Vector2D(v_rb[0], v_rb[1]);

        if (!(current_state == 2 || current_state == 3)) { // 当前不是在走步态的话，请求进入走步态调整方向
            last_state = req_state;
            req_state  = 2;
        }
        RCLCPP_INFO(robot->node_->get_logger(), "请求走步态调整方向,state=%d", current_state);
    } else {
        req_state = last_state;
    }

    if (current_state == 0 || current_state == 1)          // 纯滑行态或准备上台阶态
    {
        // 只有在没有请求走步态时，才根据step_mode设置状态
        if (req_state != 2) {
            if (robot->move_cmd.step_mode == 3)
                req_state = 0;
            else if (robot->move_cmd.step_mode == 4)
                req_state = 1;
        }

        current_state = req_state;

        if (current_state == 1) { // 需要进行自动上台阶
            // 测量足端受到的意外力，判定足端碰到台阶，需要抬腿上台阶
            // 只有在一次抬腿完成后至少间隔1s，才允许检查并触发下一次抬腿
            auto now = robot->node_->get_clock()->now();


            // 找到最大的足端力
            int trgged_leg   = 1;
            double max_force = lf_cart_force[0];
            if (rf_cart_force[0] > max_force) {
                max_force  = rf_cart_force[0];
                trgged_leg = 2;
            }
            if (lb_cart_force[0] > max_force) {
                max_force  = lb_cart_force[0];
                trgged_leg = 3;
            }
            if (rb_cart_force[0] > max_force) {
                max_force  = rb_cart_force[0];
                trgged_leg = 4;
            }

            if (max_force < foot_obstruct_gate)
                trgged_leg = 0;


            // 观测足端力，选择触发抬腿
            //  RCLCPP_INFO(robot->node_->get_logger(), "当前最大足端受力:%lf", max_force);
            if ((trgged_leg == 1 && foot_climbing_step == 0) && first_step) {

                climb_step_finished_idel_time = 0.5;
                RCLCPP_INFO(robot->node_->get_logger(), "触发左前抬腿");
                foot_climbing_step = 1;
                action_foot_num    = 1;
                action_step        = 0;
                first_step         = true;
            } else if ((trgged_leg == 2 && foot_climbing_step == 0) && first_step) {
                climb_step_finished_idel_time = 0.5;
                RCLCPP_INFO(robot->node_->get_logger(), "触发右前抬腿");
                foot_climbing_step = 2;
                action_foot_num    = 2;
                action_step        = 0;
                first_step         = true;
            } else if ((trgged_leg == 3 && foot_climbing_step == 0) && first_step) {
                RCLCPP_INFO(robot->node_->get_logger(), "触发左后抬腿");
                foot_climbing_step = 3;
                action_foot_num    = 3;
                action_step        = 0;
                first_step         = true;
            } else if ((trgged_leg == 4 && foot_climbing_step == 0) && first_step) {
                RCLCPP_INFO(robot->node_->get_logger(), "触发右后抬腿");
                foot_climbing_step = 4;
                action_foot_num    = 4;
                action_step        = 0;
                first_step         = true;
            }
        }

        lf_foot_exp_force += Vector3D(current_exp_foot_force, 0.0, -robot->robot_lf_grivate);
        rf_foot_exp_force += Vector3D(current_exp_foot_force, 0.0, -robot->robot_rf_grivate);
        lb_foot_exp_force += Vector3D(current_exp_foot_force, 0.0, -robot->robot_lb_grivate);
        rb_foot_exp_force += Vector3D(current_exp_foot_force, 0.0, -robot->robot_rb_grivate);

        // TODO:根据足端位置选择向左还是右平移
        if (action_foot_num)                                                              // 此时需要抬腿跨越台阶
        {
            lf_wheel_vel   = 0.0;
            rf_wheel_vel   = 0.0;
            lb_wheel_vel   = 0.0;
            rb_wheel_vel   = 0.0;
            lf_wheel_force = 0.0;
            rf_wheel_force = 0.0;
            lb_wheel_force = 0.0;
            rb_wheel_force = 0.0;

            current_exp_vel = 0.0;
            if (action_step == 0)                                                         // 第一步，平移狗身，使得质心进入支撑三角形内
            {
                if (action_foot_num == 1 || action_foot_num == 3)                         // 左侧需要抬腿，平移身体到右边
                {
                    RCLCPP_INFO(robot->node_->get_logger(), "左侧抬腿，狗身向右平移");
                    double x_target = -(0.5 * (last_pos_1[0] + last_pos_2[0])), y_target = step_dy;
                    lf_leg_step.update_support_trajectory(lf_cart_pos, Vector3D(x_target, y_target, lf_cart_pos[2]), 6.0);
                    rf_leg_step.update_support_trajectory(rf_cart_pos, Vector3D(0.0, y_target, rf_cart_pos[2]), 6.0);
                    lb_leg_step.update_support_trajectory(lb_cart_pos, Vector3D(x_target, y_target, lb_cart_pos[2]), 6.0);
                    rb_leg_step.update_support_trajectory(rb_cart_pos, Vector3D(0.0, y_target, rb_cart_pos[2]), 6.0);
                    action_start_time = robot->node_->get_clock()->now();
                    action_step       = 1;
                }
                if (action_foot_num == 2 || action_foot_num == 4)                         // 左侧需要抬腿，平移身体到右边
                {
                    RCLCPP_INFO(robot->node_->get_logger(), "右侧抬腿，狗身向左平移");
                    double x_target = -(0.5 * (last_pos_1[0] + last_pos_2[0])), y_target = -step_dy;
                    lf_leg_step.update_support_trajectory(lf_cart_pos, Vector3D(0.0, y_target, lf_cart_pos[2]), 6.0);
                    rf_leg_step.update_support_trajectory(rf_cart_pos, Vector3D(x_target, y_target, rf_cart_pos[2]), 6.0);
                    lb_leg_step.update_support_trajectory(lb_cart_pos, Vector3D(0.0, y_target, lb_cart_pos[2]), 6.0);
                    rb_leg_step.update_support_trajectory(rb_cart_pos, Vector3D(x_target, y_target, rb_cart_pos[2]), 6.0);
                    action_start_time = robot->node_->get_clock()->now();
                    action_step       = 1;
                }
            }
            if (action_step == 1) {                                                       // 身体平移
                bool success;
                double t = (robot->node_->get_clock()->now() - action_start_time).seconds();

                std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
                std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
                std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
                std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
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
                    last_lf_pos = lf_foot_exp_pos;
                    last_rf_pos = rf_foot_exp_pos;
                    last_lb_pos = lb_foot_exp_pos;
                    last_rb_pos = rb_foot_exp_pos;

                    action_step = 2;
                }
            }
            if (action_step == 2) // 开始抬腿
            {
                if (action_foot_num == 1 || action_foot_num == 3) {
                    RCLCPP_INFO(robot->node_->get_logger(), "左后腿向前摆动");
                    double z                    = action_foot_num == 3 ? (std::clamp(lb_cart_pos[2], -0.05, 0.05) + 0.12) : 0.0;
                    Vector3D next_available_pos = lb_cart_pos + Vector3D(0.1, 0.0, z);
                    last_pos_1                  = next_available_pos;
                    flight_foot_num             = 3;
                    lb_leg_step.update_flight_trajectory(
                        lb_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 8.0, 0.12);
                    action_start_time = robot->node_->get_clock()->now();
                    action_step       = 3;
                } else {
                    RCLCPP_INFO(robot->node_->get_logger(), "右后腿向前摆动");
                    double z                    = action_foot_num == 4 ? (std::clamp(rb_cart_pos[2], -0.05, 0.05) + 0.12) : 0.0;
                    Vector3D next_available_pos = rb_cart_pos + Vector3D(0.1, 0.0, z);
                    last_pos_1                  = next_available_pos;
                    flight_foot_num             = 4;
                    rb_leg_step.update_flight_trajectory(
                        rb_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 8.0, 0.12);
                    action_start_time = robot->node_->get_clock()->now();
                    action_step       = 3;
                }
            }
            if (action_step == 3) {
                bool success;
                double t = (robot->node_->get_clock()->now() - action_start_time).seconds();

                if (flight_foot_num == 3) {
                    lf_foot_exp_pos = last_lf_pos;
                    rf_foot_exp_pos = last_rf_pos;
                    rb_foot_exp_pos = last_rb_pos;
                    // 上面的success一定失败，只关心摆动的腿执行是否成功
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
                } else {
                    lf_foot_exp_pos                                             = last_lf_pos;
                    rf_foot_exp_pos                                             = last_rf_pos;
                    lb_foot_exp_pos                                             = last_lb_pos;
                    std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);

                    // 3足支撑力计算 (lf, rf, rb支撑，lb摆动)
                    auto lf_pos = (lf_foot_exp_pos + robot->lf_leg_calc->pos_offset).head(2);
                    auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
                    auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);

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
                }

                if (!success) {
                    last_lf_pos = lf_foot_exp_pos;
                    last_rf_pos = rf_foot_exp_pos;
                    last_lb_pos = lb_foot_exp_pos;
                    last_rb_pos = rb_foot_exp_pos;

                    action_step = 4;
                }
            }
            if (action_step == 4) {
                if (action_foot_num == 1 || action_foot_num == 3) {
                    RCLCPP_INFO(robot->node_->get_logger(), "左前腿向前摆动");
                    double z                    = action_foot_num == 1 ? (std::clamp(lf_cart_pos[2], -0.05, 0.05) + 0.12) : 0.0;
                    Vector3D next_available_pos = lf_cart_pos + Vector3D(0.1, 0.0, z);
                    last_pos_2                  = next_available_pos;
                    flight_foot_num             = 1;
                    lf_leg_step.update_flight_trajectory(
                        lf_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 8.0, 0.12);
                    action_start_time = robot->node_->get_clock()->now();
                    action_step       = 5;
                } else {
                    RCLCPP_INFO(robot->node_->get_logger(), "右前腿向前摆动");
                    double z                    = action_foot_num == 2 ? (std::clamp(rf_cart_pos[2], -0.05, 0.05) + 0.12) : 0.0;
                    Vector3D next_available_pos = rf_cart_pos + Vector3D(0.1, 0.0, z);
                    last_pos_2                  = next_available_pos;
                    flight_foot_num             = 2;
                    rf_leg_step.update_flight_trajectory(
                        rf_cart_pos, Vector3D(0.0, 0.0, 0.0), next_available_pos, Vector2D(0.0, 0.0), 8.0, 0.12);
                    action_start_time = robot->node_->get_clock()->now();
                    action_step       = 5;
                }
            }
            if (action_step == 5)                         // 还有选择哪个足端施力的错误没修正
            {
                bool success;
                double t = (robot->node_->get_clock()->now() - action_start_time).seconds();

                if (flight_foot_num == 1) {
                    std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);

                    rf_foot_exp_pos = last_rf_pos;
                    lb_foot_exp_pos = last_lb_pos;
                    rb_foot_exp_pos = last_rb_pos;
                    // 3足支撑力计算 (lf, rf, rb支撑，lb摆动)

                    auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
                    auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
                    auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

                    Eigen::Matrix3d A;
                    Eigen::Vector3d b;

                    // 力平衡约束
                    A(0, 0) = 1.0; // lf
                    A(0, 1) = 1.0; // rf
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



                    lf_foot_exp_force = Eigen::Vector3d::Zero();
                    rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
                    lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
                    rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));
                } else {
                    std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);

                    lf_foot_exp_pos = last_lf_pos;
                    lb_foot_exp_pos = last_lb_pos;
                    rb_foot_exp_pos = last_rb_pos;

                    auto lf_pos = (lf_foot_exp_pos + robot->lf_leg_calc->pos_offset).head(2);
                    auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
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
                    rf_foot_exp_force = Eigen::Vector3d::Zero();
                    lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
                    rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));
                }

                if (!success)
                    action_step = 6;
            }
            if (action_step == 6) {
                RCLCPP_INFO(robot->node_->get_logger(), "狗身移回正确位置");
                lf_leg_step.update_support_trajectory(lf_cart_pos, Vector3D(lf_cart_pos[0], 0.0, lf_cart_pos[2]), 2.0);
                rf_leg_step.update_support_trajectory(rf_cart_pos, Vector3D(rf_cart_pos[0], 0.0, rf_cart_pos[2]), 2.0);
                lb_leg_step.update_support_trajectory(lb_cart_pos, Vector3D(rf_cart_pos[0], 0.0, lb_cart_pos[2]), 2.0);
                rb_leg_step.update_support_trajectory(rb_cart_pos, Vector3D(rf_cart_pos[0], 0.0, rb_cart_pos[2]), 2.0);
                action_start_time = robot->node_->get_clock()->now();
                action_step       = 7;
            }
            if (action_step == 7) {
                bool success;
                double t = (robot->node_->get_clock()->now() - action_start_time).seconds();

                std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_leg_step.get_target(t, success);
                std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_leg_step.get_target(t, success);
                std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_leg_step.get_target(t, success);
                std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_leg_step.get_target(t, success);
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

                    last_foot_climbing_end_time = robot->node_->get_clock()->now();

                    last_lf_pos = lf_foot_exp_pos;
                    last_rf_pos = rf_foot_exp_pos;
                    last_lb_pos = lb_foot_exp_pos;
                    last_rb_pos = rb_foot_exp_pos;


                    if (first_step) {                      // 第一次迈腿
                        RCLCPP_INFO(robot->node_->get_logger(), "触发第二次抬腿");
                        first_step = false;
                        if (action_foot_num == 1)
                            action_foot_num = 2;
                        else if (action_foot_num == 2)
                            action_foot_num = 1;
                        else if (action_foot_num == 3)
                            action_foot_num = 4;
                        else if (action_foot_num == 4)
                            action_foot_num = 3;
                        RCLCPP_INFO(
                            robot->node_->get_logger(), "第二次抬腿将在下一次update从action_step=0继续, action_foot_num=%d", action_foot_num);
                        action_step = 0;
                    } else {
                        first_step      = true;
                        foot_climbing_step = 0;
                        flight_foot_num = 0;
                        action_step     = 0;
                        action_foot_num = 0;
                    }
                }
            }
        } else {
            lf_foot_exp_pos = last_lf_pos;
            rf_foot_exp_pos = last_rf_pos;
            lb_foot_exp_pos = last_lb_pos;
            rb_foot_exp_pos = last_rb_pos;

            std::tie(lf_foot_exp_pos[2], lf_foot_exp_vel[2], lf_foot_exp_acc[2]) =
                robot->lf_z_vmc->targetUpdate(lf_foot_exp_pos[2], lf_cart_pos[2], lf_foot_exp_vel[2], lf_cart_vel[2], -lf_cart_force[2]);

            std::tie(rf_foot_exp_pos[2], rf_foot_exp_vel[2], rf_foot_exp_acc[2]) =
                robot->rf_z_vmc->targetUpdate(rf_foot_exp_pos[2], rf_cart_pos[2], rf_foot_exp_vel[2], rf_cart_vel[2], -rf_cart_force[2]);

            std::tie(lb_foot_exp_pos[2], lb_foot_exp_vel[2], lb_foot_exp_acc[2]) =
                robot->lb_z_vmc->targetUpdate(lb_foot_exp_pos[2], lb_cart_pos[2], lb_foot_exp_vel[2], lb_cart_vel[2], -lb_cart_force[2]);

            std::tie(rb_foot_exp_pos[2], rb_foot_exp_vel[2], rb_foot_exp_acc[2]) =
                robot->rb_z_vmc->targetUpdate(rb_foot_exp_pos[2], rb_cart_pos[2], rb_foot_exp_vel[2], rb_cart_vel[2], -rb_cart_force[2]);
        }


        joints_target.legs[0] = robot->lf_leg_calc->signal_leg_calc(
            lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, &robot->lf_forward_torque, lf_wheel_vel, lf_wheel_force);
        joints_target.legs[1] = robot->rf_leg_calc->signal_leg_calc(
            rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, &robot->rf_forward_torque, rf_wheel_vel, rf_wheel_force);
        joints_target.legs[2] = robot->lb_leg_calc->signal_leg_calc(
            lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, &robot->lb_forward_torque, lb_wheel_vel, lb_wheel_force);
        joints_target.legs[3] = robot->rb_leg_calc->signal_leg_calc(
            rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, &robot->rb_forward_torque, rb_wheel_vel, rb_wheel_force);

    } else if (current_state == 2 || current_state == 3) { // 抬腿调整方向
        if (current_state == 2) {
            auto now                = robot->node_->get_clock()->now();
            main_phrase_start_time  = now;
            slave_phrase_start_time = now;
            slave_phrase_stop_time =
                now                                        // 预规划从相位支撑相结束时间
                + rclcpp::Duration(
                    std::chrono::duration<double>((std::abs(2.0 * step_support_rate - 1.0) * 0.5 + 1.0 - step_support_rate) * step_time));
            lf_leg_step.update_flight_trajectory(
                robot->lf_leg_calc->foot_pos(robot->lf_joint_pos), Vector3D(0.0, 0.0, 0.0), lf_exp_vel,
                ((1.0 - step_support_rate) * step_time), step_height);
            rf_leg_step.update_support_trajectory(
                robot->rf_leg_calc->foot_pos(robot->rf_joint_pos), rf_exp_vel,
                (std::abs(2.0 * step_support_rate - 1.0) * 0.5 + 1.0 - step_support_rate) * step_time);
            lb_leg_step.update_support_trajectory(
                robot->lb_leg_calc->foot_pos(robot->lb_joint_pos), lb_exp_vel,
                (std::abs(2.0 * step_support_rate - 1.0) * 0.5 + 1.0 - step_support_rate) * step_time);
            rb_leg_step.update_flight_trajectory(
                robot->lf_leg_calc->foot_pos(robot->lf_joint_pos), Vector3D(0.0, 0.0, 0.0), lf_exp_vel,
                ((1.0 - step_support_rate) * step_time), step_height);
            step1_support_updated = false;                 // 设置足端轨迹更新状态
            step1_flight_updated  = true;
            step2_flight_updated  = false;
            step2_support_updated = true;

            current_state = 3;
        }
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
                lf_leg_step.update_flight_trajectory(
                    robot->lf_leg_calc->foot_pos(robot->lf_joint_pos), -Vector3D(lf_exp_vel[0], lf_exp_vel[1], 0.0), lf_exp_vel,
                    step_time * (1.0 - step_support_rate), step_height);
                // 主相对角腿也需要规划飞行轨迹（右后）
                rb_leg_step.update_flight_trajectory(
                    robot->rb_leg_calc->foot_pos(robot->rb_joint_pos), -Vector3D(rb_exp_vel[0], rb_exp_vel[1], 0.0), rb_exp_vel,
                    step_time * (1.0 - step_support_rate), step_height);
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
                if (req_state == last_state) {                         // 请求状态为停止，那么记录足端停下的位置，然后请求跳转到stop
                    robot->lf_leg_stop_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
                    robot->rf_leg_stop_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
                    robot->lb_leg_stop_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
                    robot->rb_leg_stop_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

                    current_state = last_state;
                }
                RCLCPP_INFO(robot->node_->get_logger(), "从相位支撑相规划");
            }
        } else if (step2_support_updated && (!step2_flight_updated)) { // 如果从相位处于支撑相(调相位)
            if (now > slave_phrase_stop_time)                          // 如果到达了由主相位确定的从相位支撑相结束时间，那么更新从相位飞行相
            {
                step2_support_updated = false;                         // 设置足端轨迹更新状态
                step2_flight_updated  = true;
                // 从相两条腿同时进入飞行相（右前 & 左后）
                rf_leg_step.update_flight_trajectory(
                    robot->rf_leg_calc->foot_pos(robot->rf_joint_pos), -Vector3D(rf_exp_vel[0], rf_exp_vel[1], 0.0), rf_exp_vel,
                    (1.0 - step_support_rate) * step_time, step_height);
                lb_leg_step.update_flight_trajectory(
                    robot->lb_leg_calc->foot_pos(robot->lb_joint_pos), -Vector3D(lb_exp_vel[0], lb_exp_vel[1], 0.0), lb_exp_vel,
                    (1.0 - step_support_rate) * step_time, step_height);
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



        if (step1_support_updated) {                                                      // 主相位需要VMC计算
            std::tie(lf_foot_exp_pos[2], lf_foot_exp_vel[2], lf_foot_exp_acc[2]) =
                robot->lf_z_vmc->targetUpdate(lf_foot_exp_pos[2], lf_cart_pos[2], lf_foot_exp_vel[2], lf_cart_vel[2], -lf_cart_force[2]);

            std::tie(rb_foot_exp_pos[2], rb_foot_exp_vel[2], rb_foot_exp_acc[2]) =
                robot->rb_z_vmc->targetUpdate(rb_foot_exp_pos[2], rb_cart_pos[2], rb_foot_exp_vel[2], rb_cart_vel[2], -rb_cart_force[2]);

            if (step2_support_updated) {                   // 如果从相位也需要VMC计算，说明此时四足触底，每个脚的向下的力为一倍，否则为两倍
                lf_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_lf_grivate);
                rb_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_rb_grivate);
            } else {
                lf_foot_exp_force += Vector3D(0.0, 0.0, -2.0 * robot->robot_lf_grivate);
                rb_foot_exp_force += Vector3D(0.0, 0.0, -2.0 * robot->robot_rb_grivate);
                joints_target.legs[1].wheel.torque = 0.0f;
                joints_target.legs[2].wheel.torque = 0.0f;
            }



            std::tie(lf_foot_exp_pos[0], lf_foot_exp_vel[0], lf_foot_exp_acc[0]) = robot->lf_x_vmc->targetUpdate(
                lf_foot_exp_pos[0], lf_cart_pos[0], lf_foot_exp_vel[0], lf_cart_vel[0],
                -lf_cart_force[0]);                        // 实际这个lf_cart_force是足端本身要施加的力，不是受到的力
            std::tie(lf_foot_exp_pos[1], lf_foot_exp_vel[1], lf_foot_exp_acc[1]) =
                robot->lf_y_vmc->targetUpdate(lf_foot_exp_pos[1], lf_cart_pos[1], lf_foot_exp_vel[1], lf_cart_vel[1], -lf_cart_force[1]);
            std::tie(rb_foot_exp_pos[0], rb_foot_exp_vel[0], rb_foot_exp_acc[0]) =
                robot->rb_x_vmc->targetUpdate(rb_foot_exp_pos[0], rb_cart_pos[0], rb_foot_exp_vel[0], rb_cart_vel[0], -rb_cart_force[0]);
            std::tie(rb_foot_exp_pos[1], rb_foot_exp_vel[1], rb_foot_exp_acc[1]) =
                robot->rb_y_vmc->targetUpdate(rb_foot_exp_pos[1], rb_cart_pos[1], rb_foot_exp_vel[1], rb_cart_vel[1], -rb_cart_force[1]);
        }
        if (step2_support_updated) {                       // 从相位需要VMC计算

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
                joints_target.legs[0].wheel.torque = 0.0f; // 另外两个轮子腾空，取消驱动
                joints_target.legs[3].wheel.torque = 0.0f;
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


        // 根据支撑相/飞行相设置轮子控制：飞行相时速度保持但力矩为0
        double lf_wheel_force = step1_support_updated ? current_exp_foot_force : 0.0;
        double rb_wheel_force = step1_support_updated ? -current_exp_foot_force : 0.0;
        double rf_wheel_force = step2_support_updated ? -current_exp_foot_force : 0.0;
        double lb_wheel_force = step2_support_updated ? current_exp_foot_force : 0.0;

        joints_target.legs[0] = robot->lf_leg_calc->signal_leg_calc(
            lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, &robot->lf_forward_torque, current_exp_vel,
            lf_wheel_force);
        joints_target.legs[1] = robot->rf_leg_calc->signal_leg_calc(
            rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, &robot->rf_forward_torque, -current_exp_vel,
            rf_wheel_force);
        joints_target.legs[2] = robot->lb_leg_calc->signal_leg_calc(
            lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, &robot->lb_forward_torque, current_exp_vel,
            lb_wheel_force);
        joints_target.legs[3] = robot->rb_leg_calc->signal_leg_calc(
            rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, &robot->rb_forward_torque, -current_exp_vel,
            rb_wheel_force);
    }

    // RCLCPP_INFO(robot->node_->get_logger(),"(%lf,%lf)",joints_target.legs[0].wheel.omega,joints_target.legs[0].wheel.torque);
    robot->legs_target_pub->publish(joints_target);

    if (current_state == 0 || current_state == 1) // 只有处于非抬腿状态，才能切状态
    {
        if (robot->move_cmd.step_mode == 1)
            return "stop";
    }
    return "climb_steps";
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
    ClimbStepstate::balance_force_calc(Robot* robot, double cur_roll, double cur_pitch,double exp_roll,double exp_pitch) {

    double roll_offset_virtual_torque  = robot->roll_vmc->update(cur_roll-exp_roll, robot->robot_velocity.angular.x, 0.0);
    double pitch_offset_virtual_torque = robot->pitch_vmc->update(cur_pitch-exp_pitch, robot->robot_velocity.angular.y, 0.0);

    // TODO:计算四个足端的期望的平衡虚拟力(pitch)
    Eigen::Vector3d lf_force = Eigen::Vector3d::Zero(), rf_force = Eigen::Vector3d::Zero(), lb_force = Eigen::Vector3d::Zero(),
                    rb_force = Eigen::Vector3d::Zero();
    lf_force[2] += pitch_offset_virtual_torque * robot->lf_leg_calc->pos_offset[0];
    rf_force[2] += pitch_offset_virtual_torque * robot->rf_leg_calc->pos_offset[0];
    lb_force[2] += pitch_offset_virtual_torque * robot->lb_leg_calc->pos_offset[0];
    rb_force[2] += pitch_offset_virtual_torque * robot->rb_leg_calc->pos_offset[0];

    // TODO:计算四个足端的期望的平衡虚拟力(roll)
    lf_force[2] += roll_offset_virtual_torque * robot->lf_leg_calc->pos_offset[1];
    rf_force[2] += roll_offset_virtual_torque * robot->rf_leg_calc->pos_offset[1];
    lb_force[2] += roll_offset_virtual_torque * robot->lb_leg_calc->pos_offset[1];
    rb_force[2] += roll_offset_virtual_torque * robot->rb_leg_calc->pos_offset[1];


    return {lf_force, rf_force, lb_force, rb_force};
}


Vector3D ClimbStepstate::get_next_available_pos(Vector3D leg_offset, Vector3D current_pos) {
    (void)leg_offset;
    (void)current_pos;
    return {0.12, current_pos[1], 0.0};
}
