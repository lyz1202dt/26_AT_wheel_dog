#include "states/walk.hpp"
#include "core/robot.hpp"


WalkState::WalkState(Robot* robot)
    : BaseState<Robot>("walk") {
        robot->node_->declare_parameter("step_time",0.5);
        robot->node_->declare_parameter("step_height",0.08);
        robot->node_->declare_parameter("step_support_rate",0.6);

        robot->add_param_cb([this](const rclcpp::Parameter& param){
            auto name=param.get_name();
            if(name=="step_time")
                step_time=param.as_double();
            else if(name=="step_height")
                step_height=param.as_double();
            else if(name=="step_support_rate")
                step_support_rate=param.as_double();
            return true;
        });
    }

bool WalkState::enter(Robot* robot, const std::string& last_status) {
    calc_foot_vel(robot, Vector3D(robot->move_cmd.vx, robot->move_cmd.vy, robot->move_cmd.vz));

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
}

std::string WalkState::update(Robot* robot) {
    std::string next_state("walk");
    Vector3D lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos;
    Vector3D lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force;
    Vector3D lf_foot_exp_vel, rf_foot_exp_vel, lb_foot_exp_vel, rb_foot_exp_vel;
    Vector3D lf_foot_exp_acc, rf_foot_exp_acc, lb_foot_exp_acc, rb_foot_exp_acc;

    double cur_roll, cur_pitch, cur_yaw;
    tf2::Matrix3x3(robot->robot_rotation).getRPY(cur_roll, cur_pitch, cur_yaw);
    std::tie(lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force) = balance_force_calc(robot, cur_roll, cur_pitch);

    if ((cur_roll > 40 * 3.14 / 180 || cur_roll < -40 * 3.14 / 180 || cur_pitch > 50 * 3.14 / 180
         || cur_pitch < -50 * 3.14 / 180))                                                               // 机器人倾倒，切入IDEL状态
        return "idel";

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
            if (robot->move_cmd.step_mode == 1) {                  // 请求状态为停止，那么记录足端停下的位置，然后请求跳转到stop
                robot->lf_leg_stop_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
                robot->rf_leg_stop_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
                robot->lb_leg_stop_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
                robot->rb_leg_stop_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);


                next_state = "stop";
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

    if (step1_support_updated) {                                                      // 主相位需要VMC计算
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
    if (step2_support_updated) {     // 从相位需要VMC计算

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

    return next_state;
}

std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d> calc_foot_vel(Robot* robot, Eigen::Vector3d exp_vel) {
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
    WalkState::balance_force_calc(Robot* robot, double cur_roll, double cur_pitch) {

    double sin_pitch = std::sin(cur_pitch);
    double sin_roll  = std::sin(cur_roll);

    double roll_offset_virtual_torque  = robot->roll_vmc->update(cur_roll, robot->robot_velocity.angular.x, 0.0);
    double pitch_offset_virtual_torque = robot->pitch_vmc->update(cur_pitch, robot->robot_velocity.angular.y, 0.0);

    // TODO:计算四个足端的期望的平衡虚拟力(pitch)
    Eigen::Vector3d lf_force, rf_force, lb_force, rb_force;
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
