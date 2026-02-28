#include "states/climb_steps.hpp"

ClimbStepstate::ClimbStepstate(Robot* robot)
    : BaseState<Robot>("climb_steps") {
    (void)robot;
}

bool ClimbStepstate::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    return true;
}

std::string ClimbStepstate::update(Robot* robot) {
    current_body_vel = 0.25 * (robot->lf_wheel_omega + robot->rf_wheel_omega + robot->lb_wheel_omega + robot->rb_wheel_omega)
                     * robot->WHEEL_RADIUS;       // 估计机器人本体速度
    double acc                     = exp_vel_kp * (robot->move_cmd.vx - current_body_vel);
    current_exp_vel                = current_exp_vel + acc * 0.002;
    double current_exp_foot_force  = robot->robot_mass * acc * 0.25;
    double current_exp_foot_torque = current_exp_foot_force * robot->WHEEL_RADIUS;
    double current_exp_foot_omega  = current_exp_vel / robot->WHEEL_RADIUS;

    Eigen::Vector3d lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos;
    Eigen::Vector3d lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force;
    Eigen::Vector3d lf_foot_exp_vel, rf_foot_exp_vel, lb_foot_exp_vel, rb_foot_exp_vel;
    Eigen::Vector3d lf_foot_exp_acc, rf_foot_exp_acc, lb_foot_exp_acc, rb_foot_exp_acc;

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



    double cur_roll, cur_pitch, cur_yaw;
    tf2::Matrix3x3(robot->robot_rotation).getRPY(cur_roll, cur_pitch, cur_yaw);
    std::tie(lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force) = balance_force_calc(robot, cur_roll, cur_pitch);

    if ((cur_roll > 40 * 3.14 / 180 || cur_roll < -40 * 3.14 / 180 || cur_pitch > 50 * 3.14 / 180
         || cur_pitch < -50 * 3.14 / 180))        // 机器人倾倒，切入IDEL状态
        return "idel";

    lf_foot_exp_pos = robot->lf_leg_stop_pos;
    rf_foot_exp_pos = robot->rf_leg_stop_pos;
    lb_foot_exp_pos = robot->lb_leg_stop_pos;
    rb_foot_exp_pos = robot->rb_leg_stop_pos;


    robot_interfaces::msg::Robot joints_target;
    joints_target.legs[0].wheel.omega  = (float)current_exp_foot_omega;
    joints_target.legs[0].wheel.torque = (float)current_exp_foot_torque;
    joints_target.legs[2].wheel        = joints_target.legs[0].wheel;

    joints_target.legs[1].wheel.omega  = -(float)current_exp_foot_omega;
    joints_target.legs[1].wheel.torque = -(float)current_exp_foot_torque;
    joints_target.legs[3].wheel        = joints_target.legs[1].wheel;


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

        last_state = req_state;
        req_state  = 2;
    } else {
        req_state = last_state;
    }

    if (current_state == 0 || current_state == 1) // 不抬腿，轮子纯走
    {

        current_state = req_state;

        if (current_state == 1) {                 // 判定为需要进行自动上台阶
            // 测量足端受到的意外力，判定足端碰到台阶，需要抬腿上台阶
            if ((lf_cart_force[0] < foot_obstruct_gate&&foot_climbing_step == 0) || foot_climbing_step == 1) {
                if (!foot_trajectory_updated) {
                    foot_trajectory_updated = true;
                    foot_climbing_step      = 1;
                    foot_climbing_time      = robot->node_->get_clock()->now();
                    lf_leg_step.update_flight_trajectory(
                        robot->lf_leg_calc->foot_pos(robot->lf_joint_pos), Vector3D(0.0, 0.0, 0.0), Vector3D(0.0, 0.0, 0.1),Vector2D(0.0,0.0), 0.8, 1.3);
                }
                bool success = false;
                std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target((robot->node_->get_clock()->now() - foot_climbing_time).seconds(), success);
                if (!success) {
                    foot_climbing_step = 0;
                    foot_trajectory_updated = false;
                }
            } else if ((rf_cart_force[0] < foot_obstruct_gate&&foot_climbing_step == 0)|| foot_climbing_step == 2) {
                if (!foot_trajectory_updated) {
                    foot_trajectory_updated = true;
                    foot_climbing_step      = 2;
                    foot_climbing_time      = robot->node_->get_clock()->now();
                    rf_leg_step.update_flight_trajectory(
                        robot->rf_leg_calc->foot_pos(robot->rf_joint_pos), Vector3D(0.0, 0.0, 0.0), Vector3D(0.0, 0.0, 0.1),Vector2D(0.0,0.0), 0.8, 1.3);
                }
                bool success = false;
                std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_leg_step.get_target((robot->node_->get_clock()->now() - foot_climbing_time).seconds(), success);
                if (!success) {
                    foot_climbing_step = 0;
                    foot_trajectory_updated = false;
                }

            } else if ((lb_cart_force[0] < foot_obstruct_gate&&foot_climbing_step == 0)|| foot_climbing_step == 3) {
                if (!foot_trajectory_updated) {
                    foot_trajectory_updated = true;
                    foot_climbing_step      = 3;
                    foot_climbing_time      = robot->node_->get_clock()->now();
                    lb_leg_step.update_flight_trajectory(
                        robot->lb_leg_calc->foot_pos(robot->lb_joint_pos), Vector3D(0.0, 0.0, 0.0), Vector3D(0.0, 0.0, 0.1),Vector2D(0.0,0.0), 0.8, 1.3);
                }
                bool success = false;
                std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target((robot->node_->get_clock()->now() - foot_climbing_time).seconds(), success);
                if (!success) {
                    foot_climbing_step = 0;
                    foot_trajectory_updated = false;
                }

            } else if ((rb_cart_force[0] < foot_obstruct_gate&&foot_climbing_step == 0)|| foot_climbing_step == 4) {
                if (!foot_trajectory_updated) {
                    foot_trajectory_updated = true;
                    foot_climbing_step      = 4;
                    foot_climbing_time      = robot->node_->get_clock()->now();
                    rb_leg_step.update_flight_trajectory(
                        robot->rb_leg_calc->foot_pos(robot->rb_joint_pos), Vector3D(0.0, 0.0, 0.0), Vector3D(0.0, 0.0, 0.1),Vector2D(0.0,0.0), 0.8, 1.3);
                }
                bool success = false;
                std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target((robot->node_->get_clock()->now() - foot_climbing_time).seconds(), success);
                if (!success) {
                    foot_climbing_step = 0;
                    foot_trajectory_updated = false;
                }
            }
        }
        
        std::tie(lf_foot_exp_pos[2], lf_foot_exp_vel[2], lf_foot_exp_acc[2]) =
            robot->lf_z_vmc->targetUpdate(lf_foot_exp_pos[2], lf_cart_pos[2], lf_foot_exp_vel[2], lf_cart_vel[2], -lf_cart_force[2]);
        lf_foot_exp_force += Vector3D(current_exp_foot_force, 0.0, -robot->robot_lf_grivate);
       
        std::tie(rf_foot_exp_pos[2], rf_foot_exp_vel[2], rf_foot_exp_acc[2]) =
            robot->rf_z_vmc->targetUpdate(rf_foot_exp_pos[2], rf_cart_pos[2], rf_foot_exp_vel[2], rf_cart_vel[2], -rf_cart_force[2]);
        rf_foot_exp_force += Vector3D(current_exp_foot_force, 0.0, -robot->robot_rf_grivate);
       
        std::tie(lb_foot_exp_pos[2], lb_foot_exp_vel[2], lb_foot_exp_acc[2]) =
            robot->lb_z_vmc->targetUpdate(lb_foot_exp_pos[2], lb_cart_pos[2], lb_foot_exp_vel[2], lb_cart_vel[2], -lb_cart_force[2]);
        lb_foot_exp_force += Vector3D(current_exp_foot_force, 0.0, -robot->robot_lb_grivate);

        std::tie(rb_foot_exp_pos[2], rb_foot_exp_vel[2], rb_foot_exp_acc[2]) =
            robot->rb_z_vmc->targetUpdate(rb_foot_exp_pos[2], rb_cart_pos[2], rb_foot_exp_vel[2], rb_cart_vel[2], -rb_cart_force[2]);
        rb_foot_exp_force += Vector3D(current_exp_foot_force, 0.0, -robot->robot_rb_grivate);


        robot_interfaces::msg::Robot joints_target;
        joints_target.legs[0] = robot->signal_leg_calc(
            lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, robot->lf_leg_calc,
            &robot->lf_forward_torque);
        joints_target.legs[1] = robot->signal_leg_calc(
            rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, robot->rf_leg_calc,
            &robot->rf_forward_torque);
        joints_target.legs[2] = robot->signal_leg_calc(
            lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, robot->lb_leg_calc,
            &robot->lb_forward_torque);
        joints_target.legs[3] = robot->signal_leg_calc(
            rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, robot->rb_leg_calc,
            &robot->rb_forward_torque);

    } else if (current_state == 2) {
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
                if (req_state == 0) {                                  // 请求状态为停止，那么记录足端停下的位置，然后请求跳转到stop
                    robot->lf_leg_stop_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
                    robot->rf_leg_stop_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
                    robot->lb_leg_stop_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
                    robot->rb_leg_stop_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

                    current_state = 0;
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


        joints_target.legs[0] = robot->signal_leg_calc(
            lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, robot->lf_leg_calc, &robot->lf_forward_torque);
        joints_target.legs[1] = robot->signal_leg_calc(
            rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, robot->rf_leg_calc, &robot->rf_forward_torque);
        joints_target.legs[2] = robot->signal_leg_calc(
            lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, robot->lb_leg_calc, &robot->lb_forward_torque);
        joints_target.legs[3] = robot->signal_leg_calc(
            rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, robot->rb_leg_calc, &robot->rb_forward_torque);
        robot->legs_target_pub->publish(joints_target);
    }


    robot->legs_target_pub->publish(joints_target);

    return "climb_steps";
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
    ClimbStepstate::balance_force_calc(Robot* robot, double cur_roll, double cur_pitch) {

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
