#include "states/forcewalk.hpp"
#include "core/robot.hpp"
#include <tf2/LinearMath/Quaternion.h>


ForcewalkState::ForcewalkState(Robot* robot)
    : BaseState<Robot>("forcewalk")
    , roll_vmc(20.0, 10.0)
    , pitch_vmc(30.0, 15.0)
    , lf_leg_vmc_z(200, 3.0)
    , rf_leg_vmc_z(200, 3.0)
    , lb_leg_vmc_z(200, 3.0)
    , rb_leg_vmc_z(200, 3.0)
    , lf_leg_vmc_x(80.0, 2.0)
    , rf_leg_vmc_x(80.0, 2.0)
    , lb_leg_vmc_x(80.0, 2.0)
    , rb_leg_vmc_x(80.0, 2.0)
    , lf_leg_vmc_y(80.0, 2.0)
    , rf_leg_vmc_y(80.0, 2.0)
    , lb_leg_vmc_y(80.0, 2.0)
    , rb_leg_vmc_y(80.0, 2.0) {

    mass = (robot->robot_lf_grivate + robot->robot_rf_grivate + robot->robot_lb_grivate + robot->robot_rb_grivate) / 9.8;
    mass_center_pos =
        Vector2D(
            robot->robot_lf_grivate * robot->lf_leg_calc->pos_offset[0] + robot->robot_rf_grivate * robot->rf_leg_calc->pos_offset[0]
                + robot->robot_lb_grivate * robot->lb_leg_calc->pos_offset[0] + robot->robot_rb_grivate * robot->rb_leg_calc->pos_offset[0],
            robot->robot_lf_grivate * robot->lf_leg_calc->pos_offset[1] + robot->robot_rf_grivate * robot->rf_leg_calc->pos_offset[1]
                + robot->robot_lb_grivate * robot->lb_leg_calc->pos_offset[1] + robot->robot_rb_grivate * robot->rb_leg_calc->pos_offset[1])
        / (mass * 9.8);

    // 声明z方向VMC的PD参数
    robot->node_->declare_parameter("force_walk_vmc_z_kp", 200.0);
    robot->node_->declare_parameter("force_walk_vmc_z_kd", 7.0);

    // 声明x、y方向VMC的PD参数
    robot->node_->declare_parameter("force_walk_vmc_xy_kp", 100.0);
    robot->node_->declare_parameter("force_walk_vmc_xy_kd", 5.0);

    // 声明roll和pitch轴的VMC参数
    robot->node_->declare_parameter("force_walk_vmc_roll_kp", -4.0);
    robot->node_->declare_parameter("force_walk_vmc_roll_kd", -0.4);
    robot->node_->declare_parameter("force_walk_vmc_pitch_kp", 5);
    robot->node_->declare_parameter("force_walk_vmc_pitch_kd", 0.6);

    // 添加参数变化回调函数
    robot->add_param_cb([this](const rclcpp::Parameter& param) {
        auto name = param.get_name();
        if (name == "force_walk_vmc_z_kp") {
            double kp       = param.as_double();
            lf_leg_vmc_z.kp = kp;
            rf_leg_vmc_z.kp = kp;
            lb_leg_vmc_z.kp = kp;
            rb_leg_vmc_z.kp = kp;
        } else if (name == "force_walk_vmc_z_kd") {
            double kd       = param.as_double();
            lf_leg_vmc_z.kd = kd;
            rf_leg_vmc_z.kd = kd;
            lb_leg_vmc_z.kd = kd;
            rb_leg_vmc_z.kd = kd;
        } else if (name == "force_walk_vmc_xy_kp") {
            double kp       = param.as_double();
            lf_leg_vmc_x.kp = kp;
            rf_leg_vmc_x.kp = kp;
            lb_leg_vmc_x.kp = kp;
            rb_leg_vmc_x.kp = kp;
            lf_leg_vmc_y.kp = kp;
            rf_leg_vmc_y.kp = kp;
            lb_leg_vmc_y.kp = kp;
            rb_leg_vmc_y.kp = kp;
        } else if (name == "force_walk_vmc_xy_kd") {
            double kd       = param.as_double();
            lf_leg_vmc_x.kd = kd;
            rf_leg_vmc_x.kd = kd;
            lb_leg_vmc_x.kd = kd;
            rb_leg_vmc_x.kd = kd;
            lf_leg_vmc_y.kd = kd;
            rf_leg_vmc_y.kd = kd;
            lb_leg_vmc_y.kd = kd;
            rb_leg_vmc_y.kd = kd;
        } else if (name == "force_walk_vmc_roll_kp")
            roll_vmc.kp = param.as_double();
        else if (name == "force_walk_vmc_roll_kd")
            roll_vmc.kd = param.as_double();
        else if (name == "force_walk_vmc_pitch_kp")
            pitch_vmc.kp = param.as_double();
        else if (name == "force_walk_vmc_pitch_kd")
            pitch_vmc.kd = param.as_double();
        RCLCPP_INFO(rclcpp::get_logger("logger"), "参数更新");
        return true;
    });


    robot->node_->declare_parameter("forcewalk_step_time", 0.5);
    robot->node_->declare_parameter("forcewalk_step_height", 0.08);
    robot->node_->declare_parameter("forcewalk_step_support_rate", 0.6);
    robot->node_->declare_parameter("forcewalk_roll_balance_step_compen", 0.3);
    robot->node_->declare_parameter("forcewalk_pitch_balance_step_compen", 0.3);
    robot->node_->declare_parameter("forcewalk_exp_roll", 0.0);
    robot->node_->declare_parameter("forcewalk_exp_pitch", 0.0);

    robot->add_param_cb([this](const rclcpp::Parameter& param) {
        auto name = param.get_name();
        if (name == "forcewalk_step_time")
            step_time = param.as_double();
        else if (name == "forcewalk_step_height")
            step_height = param.as_double();
        else if (name == "forcewalk_step_support_rate")
            step_support_rate = param.as_double();
        else if (name == "forcewalk_roll_balance_step_compen")
            roll_balance_step_compen = param.as_double();
        else if (name == "forcewalk_pitch_balance_step_compen")
            pitch_balance_step_compen = param.as_double();
        else if (name == "forcewalk_exp_roll")
            exp_roll = param.as_double();
        else if (name == "forcewalk_exp_pitch")
            exp_pitch = param.as_double();
        return true;
    });
}

bool ForcewalkState::enter(Robot* robot, const std::string& last_status) {
    (void)last_status;
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
    return true;
}

std::string ForcewalkState::update(Robot* robot) {
    std::string next_state("forcewalk");
    Vector3D lf_foot_exp_pos = Vector3D::Zero(), rf_foot_exp_pos = Vector3D::Zero(), lb_foot_exp_pos = Vector3D::Zero(),
             rb_foot_exp_pos   = Vector3D::Zero();
    Vector3D lf_foot_exp_force = Vector3D::Zero(), rf_foot_exp_force = Vector3D::Zero(), lb_foot_exp_force = Vector3D::Zero(),
             rb_foot_exp_force = Vector3D::Zero();
    Vector3D lf_foot_exp_vel = Vector3D::Zero(), rf_foot_exp_vel = Vector3D::Zero(), lb_foot_exp_vel = Vector3D::Zero(),
             rb_foot_exp_vel = Vector3D::Zero();
    Vector3D lf_foot_exp_acc = Vector3D::Zero(), rf_foot_exp_acc = Vector3D::Zero(), lb_foot_exp_acc = Vector3D::Zero(),
             rb_foot_exp_acc = Vector3D::Zero();

    auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
    auto lf_cart_vel = robot->lf_leg_calc->foot_vel(robot->lf_joint_pos, robot->lf_joint_vel);
    auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
    auto rb_cart_vel = robot->rb_leg_calc->foot_vel(robot->rb_joint_pos, robot->rb_joint_vel);
    auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
    auto rf_cart_vel = robot->rf_leg_calc->foot_vel(robot->rf_joint_pos, robot->rf_joint_vel);
    auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
    auto lb_cart_vel = robot->lb_leg_calc->foot_vel(robot->lb_joint_pos, robot->lb_joint_vel);

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

            lf_leg_step.update_support_trajectory(lf_cart_pos, lf_exp_vel, step_support_rate * step_time);
            // 主相对角腿也需要同步进入支撑相（右后）
            rb_leg_step.update_support_trajectory(rb_cart_pos, rb_exp_vel, step_support_rate * step_time);
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
                lf_cart_pos, -Vector3D(lf_exp_vel[0], lf_exp_vel[1], 0.0), lf_exp_vel, step_time * (1.0 - step_support_rate), step_height,
                footstep_correction);
            // 主相对角腿也需要规划飞行轨迹（右后）
            rb_leg_step.update_flight_trajectory(
                rb_cart_pos, -Vector3D(rb_exp_vel[0], rb_exp_vel[1], 0.0), rb_exp_vel, step_time * (1.0 - step_support_rate), step_height,
                footstep_correction);
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

            rf_leg_step.update_support_trajectory(
                rf_cart_pos, rf_exp_vel,
                step_support_rate * step_time);                        // 预更新支撑相(精确结束时间由主相位确定)
            // 从相对角腿也同步进入支撑相（左后）
            lb_leg_step.update_support_trajectory(lb_cart_pos, lb_exp_vel, step_support_rate * step_time);
            slave_phrase_start_time = now;
            slave_phrase_stop_time  = now + rclcpp::Duration(std::chrono::duration<double>(step_support_rate * step_time));
            if (robot->move_cmd.step_mode == 1) {                  // 请求状态为停止，那么记录足端停下的位置，然后请求跳转到stop
                robot->lf_leg_stop_pos = lf_cart_pos;
                robot->rf_leg_stop_pos = rf_cart_pos;
                robot->lb_leg_stop_pos = lb_cart_pos;
                robot->rb_leg_stop_pos = rb_cart_pos;
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
                rf_cart_pos, -Vector3D(rf_exp_vel[0], rf_exp_vel[1], 0.0), rf_exp_vel, (1.0 - step_support_rate) * step_time, step_height,
                footstep_correction);
            lb_leg_step.update_flight_trajectory(
                lb_cart_pos, -Vector3D(lb_exp_vel[0], lb_exp_vel[1], 0.0), lb_exp_vel, (1.0 - step_support_rate) * step_time, step_height,
                footstep_correction);

            slave_phrase_start_time = now;
            RCLCPP_INFO(robot->node_->get_logger(), "从相位摆动相规划");
        }
    }

    bool success[4]; // 得到狗腿当前期望
    std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) =
        lf_leg_step.get_target((now - main_phrase_start_time).seconds(), success[0]);
    std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) =
        rf_leg_step.get_target((now - slave_phrase_start_time).seconds(), success[1]);
    std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) =
        lb_leg_step.get_target((now - slave_phrase_start_time).seconds(), success[2]);
    std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) =
        rb_leg_step.get_target((now - main_phrase_start_time).seconds(), success[3]);

    robot_interfaces::msg::RobotTarget joints_target;
    if (step1_flight_updated && step2_support_updated) {
        // 摆动相位置控制
        joints_target.legs[0] = robot->rf_leg_calc->signal_leg_calc(
            rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, Eigen::Vector3d::Zero(), &robot->rf_forward_torque);
        joints_target.legs[3] = robot->lb_leg_calc->signal_leg_calc(
            lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, Eigen::Vector3d::Zero(), &robot->lb_forward_torque);

        // 支撑相力控
        rf_foot_exp_force[2] = rf_leg_vmc_z.update(rf_cart_pos[2], rf_cart_vel[2],lf_foot_exp_pos[2],lf_foot_exp_vel[2]);
        rf_foot_exp_force[0] = rf_leg_vmc_x.update(rf_cart_pos[0], rf_cart_vel[0],lf_foot_exp_pos[0],lf_foot_exp_vel[0]);
        rf_foot_exp_force[1] = rf_leg_vmc_y.update(rf_cart_pos[1], rf_cart_vel[1],lf_foot_exp_pos[1],lf_foot_exp_vel[1]);

        lb_foot_exp_force[2] = lb_leg_vmc_z.update(lb_cart_pos[2], lb_cart_vel[2],lb_foot_exp_pos[2],lb_foot_exp_vel[2]);
        lb_foot_exp_force[0] = lb_leg_vmc_x.update(lb_cart_pos[0], lb_cart_vel[0],lb_foot_exp_pos[0],lb_foot_exp_vel[0]);
        lb_foot_exp_force[1] = lb_leg_vmc_y.update(lb_cart_pos[1], lb_cart_vel[1],lb_foot_exp_pos[1],lb_foot_exp_vel[1]);

        joints_target.legs[1] =
            robot->rf_leg_calc->signal_leg_calc(rf_cart_pos, rf_cart_vel, rf_foot_exp_acc, rf_foot_exp_force, &robot->rf_forward_torque);
        joints_target.legs[2] =
            robot->lb_leg_calc->signal_leg_calc(lb_cart_pos, lb_cart_vel, lb_foot_exp_acc, lb_foot_exp_force, &robot->lb_forward_torque);
    } else if (step1_support_updated && step2_flight_updated) {
        // 摆动相位置控制
        joints_target.legs[1] = robot->rf_leg_calc->signal_leg_calc(
            rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, Eigen::Vector3d::Zero(), &robot->rf_forward_torque);
        joints_target.legs[2] = robot->lb_leg_calc->signal_leg_calc(
            lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, Eigen::Vector3d::Zero(), &robot->lb_forward_torque);

        // 支撑相力控
        lf_foot_exp_force[2] = lf_leg_vmc_z.update(lf_cart_pos[2], lf_cart_vel[2],lf_foot_exp_pos[2],lf_foot_exp_vel[2]);
        lf_foot_exp_force[0] = lf_leg_vmc_x.update(lf_cart_pos[0], lf_cart_vel[0],lf_foot_exp_pos[0],lf_foot_exp_vel[0]);
        lf_foot_exp_force[1] = lf_leg_vmc_y.update(lf_cart_pos[1], lf_cart_vel[1],lf_foot_exp_pos[1],lf_foot_exp_vel[1]);

        rb_foot_exp_force[2] = rb_leg_vmc_z.update(rb_cart_pos[2], rb_cart_vel[2],rb_foot_exp_pos[2],rb_foot_exp_vel[2]);
        rb_foot_exp_force[0] = rb_leg_vmc_x.update(rb_cart_pos[0], rb_cart_vel[0],rb_foot_exp_pos[0],rb_foot_exp_vel[0]);
        rb_foot_exp_force[1] = rb_leg_vmc_y.update(rb_cart_pos[1], rb_cart_vel[1],rb_foot_exp_pos[1],rb_foot_exp_vel[1]);

        joints_target.legs[0] =
            robot->lf_leg_calc->signal_leg_calc(lf_cart_pos, lf_cart_vel, lf_foot_exp_acc, lf_foot_exp_force, &robot->lf_forward_torque);
        joints_target.legs[3] =
            robot->rb_leg_calc->signal_leg_calc(rb_cart_pos, rb_cart_vel, rb_foot_exp_acc, rb_foot_exp_force, &robot->rb_forward_torque);
    } else {
        // 四个腿全部处于支撑相，都进行力控
        lf_foot_exp_force[2] = lf_leg_vmc_z.update(lf_cart_pos[2], lf_cart_vel[2],lf_foot_exp_pos[2],lf_foot_exp_vel[2]);
        lf_foot_exp_force[0] = lf_leg_vmc_x.update(lf_cart_pos[0], lf_cart_vel[0],lf_foot_exp_pos[0],lf_foot_exp_vel[0]);
        lf_foot_exp_force[1] = lf_leg_vmc_y.update(lf_cart_pos[1], lf_cart_vel[1],lf_foot_exp_pos[1],lf_foot_exp_vel[1]);

        rf_foot_exp_force[2] = rf_leg_vmc_z.update(rf_cart_pos[2], rf_cart_vel[2],lf_foot_exp_pos[2],lf_foot_exp_vel[2]);
        rf_foot_exp_force[0] = rf_leg_vmc_x.update(rf_cart_pos[0], rf_cart_vel[0],lf_foot_exp_pos[0],lf_foot_exp_vel[0]);
        rf_foot_exp_force[1] = rf_leg_vmc_y.update(rf_cart_pos[1], rf_cart_vel[1],lf_foot_exp_pos[1],lf_foot_exp_vel[1]);

        lb_foot_exp_force[2] = lb_leg_vmc_z.update(lb_cart_pos[2], lb_cart_vel[2],lb_foot_exp_pos[2],lb_foot_exp_vel[2]);
        lb_foot_exp_force[0] = lb_leg_vmc_x.update(lb_cart_pos[0], lb_cart_vel[0],lb_foot_exp_pos[0],lb_foot_exp_vel[0]);
        lb_foot_exp_force[1] = lb_leg_vmc_y.update(lb_cart_pos[1], lb_cart_vel[1],lb_foot_exp_pos[1],lb_foot_exp_vel[1]);

        rb_foot_exp_force[2] = rb_leg_vmc_z.update(rb_cart_pos[2], rb_cart_vel[2],rb_foot_exp_pos[2],rb_foot_exp_vel[2]);
        rb_foot_exp_force[0] = rb_leg_vmc_x.update(rb_cart_pos[0], rb_cart_vel[0],rb_foot_exp_pos[0],rb_foot_exp_vel[0]);
        rb_foot_exp_force[1] = rb_leg_vmc_y.update(rb_cart_pos[1], rb_cart_vel[1],rb_foot_exp_pos[1],rb_foot_exp_vel[1]);

        joints_target.legs[0] =
            robot->lf_leg_calc->signal_leg_calc(lf_cart_pos, lf_cart_vel, lf_foot_exp_acc, lf_foot_exp_force, &robot->lf_forward_torque);
        joints_target.legs[1] =
            robot->rf_leg_calc->signal_leg_calc(rf_cart_pos, rf_cart_vel, rf_foot_exp_acc, rf_foot_exp_force, &robot->rf_forward_torque);
        joints_target.legs[2] =
            robot->lb_leg_calc->signal_leg_calc(lb_cart_pos, lb_cart_vel, lb_foot_exp_acc, lb_foot_exp_force, &robot->lb_forward_torque);
        joints_target.legs[3] =
            robot->rb_leg_calc->signal_leg_calc(rb_cart_pos, rb_cart_vel, rb_foot_exp_acc, rb_foot_exp_force, &robot->rb_forward_torque);
    }

    // joints_target.legs[0] =
    //         robot->lf_leg_calc->signal_leg_calc(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, &robot->lf_forward_torque);
    //     joints_target.legs[1] =
    //         robot->rf_leg_calc->signal_leg_calc(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, &robot->rf_forward_torque);
    //     joints_target.legs[2] =
    //         robot->lb_leg_calc->signal_leg_calc(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, &robot->lb_forward_torque);
    //     joints_target.legs[3] =
    //         robot->rb_leg_calc->signal_leg_calc(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, &robot->rb_forward_torque);
    

    RCLCPP_INFO(robot->node_->get_logger(),"lf_foot_exp_pos:(%lf,%lf,%lf)",lf_foot_exp_pos[0],lf_foot_exp_pos[1],lf_foot_exp_pos[2]);

    robot->legs_target_pub->publish(joints_target);

    return next_state;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
    ForcewalkState::balance_force_calc(Robot* robot, double cur_roll, double cur_pitch) {

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

    // TODO:计算四个足端的期望的平衡虚拟力(roll)
    lf_force[2] += roll_offset_virtual_torque * robot->lf_leg_calc->pos_offset[1];
    rf_force[2] += roll_offset_virtual_torque * robot->rf_leg_calc->pos_offset[1];
    lb_force[2] += roll_offset_virtual_torque * robot->lb_leg_calc->pos_offset[1];
    rb_force[2] += roll_offset_virtual_torque * robot->rb_leg_calc->pos_offset[1];

    return {lf_force, rf_force, lb_force, rb_force};
}

std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d>
    ForcewalkState::calc_foot_vel(Robot* robot, Eigen::Vector3d exp_vel) {
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