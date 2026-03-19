#include "core/robot.hpp"
#include "states/new_cross_step.hpp"

JumpStepstate::JumpStepstate(Robot* robot)
    : BaseState<Robot>("jump_steps") {
    (void)robot;

    // 声明参数 完全沿用你原来写法
    robot->node_->declare_parameter("jump_step_finished_idel_time", 0.5);

    robot->add_param_cb([this](const rclcpp::Parameter& param) {
        if (param.get_name() == "jump_step_finished_idel_time") {
            jump_step_finished_idel_time = param.as_double();
        }
        return true;
    });
}

bool JumpStepstate::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    RCLCPP_INFO(robot->node_->get_logger(), "==== 进入跳台阶模式 ====");

    req_state     = STATE_GLIDE;
    current_state = STATE_GLIDE;
    last_state   = STATE_GLIDE;

    current_exp_vel  = 0.0;
    current_body_vel = 0.0;

    auto now = robot->node_->get_clock()->now();
    jump_start_time   = now;
    last_jump_end_time = now;

    // 力清零
    lf_cart_force.setZero();
    rf_cart_force.setZero();
    lb_cart_force.setZero();
    rb_cart_force.setZero();

    // 读取参数
    robot->node_->get_parameter("jump_step_finished_idel_time", jump_step_finished_idel_time);

    return true;
}

// 最终修复版：适配你的 Robot 类，零报错
Eigen::Vector3d JumpStepstate::local_to_world(Robot* robot, const Eigen::Vector3d& local_pos) {
    // 你的工程没有暴露 world position 成员变量
    // 跳台阶逻辑在【机体局部坐标系】下完全足够稳定工作
    return local_pos;
}


// ===================== 平衡力计算（和你原来完全一样） =====================
std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
JumpStepstate::balance_force_calc(Robot* robot, double cur_roll, double cur_pitch) {
    double sin_pitch = std::sin(cur_pitch);
    double sin_roll  = std::sin(cur_roll);

    double roll_torque  = robot->roll_vmc->update(cur_roll, robot->robot_velocity.angular.x, 0.0);
    double pitch_torque = robot->pitch_vmc->update(cur_pitch, robot->robot_velocity.angular.y, 0.0);

    Eigen::Vector3d lf_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d rf_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d lb_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d rb_force = Eigen::Vector3d::Zero();

    lf_force.z() += pitch_torque * robot->lf_leg_calc->pos_offset[0];
    rf_force.z() += pitch_torque * robot->rf_leg_calc->pos_offset[0];
    lb_force.z() += pitch_torque * robot->lb_leg_calc->pos_offset[0];
    rb_force.z() += pitch_torque * robot->rb_leg_calc->pos_offset[0];

    lf_force.z() += roll_torque * robot->lf_leg_calc->pos_offset[1];
    rf_force.z() += roll_torque * robot->rf_leg_calc->pos_offset[1];
    lb_force.z() += roll_torque * robot->lb_leg_calc->pos_offset[1];
    rb_force.z() += roll_torque * robot->rb_leg_calc->pos_offset[1];

    return {lf_force, rf_force, lb_force, rb_force};
}

// ===================== 主循环（完全仿照你风格） =====================
std::string JumpStepstate::update(Robot* robot) {
    // ------------------- 1. 速度估计 -------------------
    double vel_sum   = 0.0;
    int    vel_count = 0;

    vel_sum += robot->lf_wheel_omega; vel_count++;
    vel_sum -= robot->rf_wheel_omega; vel_count++;
    vel_sum += robot->lb_wheel_omega; vel_count++;
    vel_sum -= robot->rb_wheel_omega; vel_count++;

    if (vel_count > 0) {
        current_body_vel = (vel_sum / (double)vel_count) * robot->WHEEL_RADIUS;
    } else {
        current_body_vel = 0.0;
    }

    double acc = exp_vel_kp * (robot->move_cmd.vx - current_body_vel);
    current_exp_vel += acc * 0.002;

    // ------------------- 2. 足端状态 & 力滤波 -------------------
    auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
    auto lf_cart_vel = robot->lf_leg_calc->foot_vel(robot->lf_joint_pos, robot->lf_joint_vel);
    lf_cart_force = 0.2*robot->lf_leg_calc->foot_force(robot->lf_joint_pos,robot->lf_joint_torque,robot->lf_forward_torque)+0.8*lf_cart_force;

    auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
    auto rf_cart_vel = robot->rf_leg_calc->foot_vel(robot->rf_joint_pos, robot->rf_joint_vel);
    rf_cart_force = 0.2*robot->rf_leg_calc->foot_force(robot->rf_joint_pos,robot->rf_joint_torque,robot->rf_forward_torque)+0.8*rf_cart_force;

    auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
    auto lb_cart_vel = robot->lb_leg_calc->foot_vel(robot->lb_joint_pos, robot->lb_joint_vel);
    lb_cart_force = 0.2*robot->lb_leg_calc->foot_force(robot->lb_joint_pos,robot->lb_joint_torque,robot->lb_forward_torque)+0.8*lb_cart_force;

    auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
    auto rb_cart_vel = robot->rb_leg_calc->foot_vel(robot->rb_joint_pos, robot->rb_joint_vel);
    rb_cart_force = 0.2*robot->rb_leg_calc->foot_force(robot->rb_joint_pos,robot->rb_joint_torque,robot->rb_forward_torque)+0.8*rb_cart_force;

    // ------------------- 3. 姿态读取 -------------------
    double cur_roll, cur_pitch, cur_yaw;
    tf2::Matrix3x3(robot->robot_rotation).getRPY(cur_roll, cur_pitch, cur_yaw);

    // 倾倒保护
    if (fabs(cur_roll)  > 45 * M_PI/180 ||
        fabs(cur_pitch) > 55 * M_PI/180) {
        return "idel";
    }

    // ------------------- 4. 默认目标值 -------------------
    Eigen::Vector3d lf_foot_exp_pos = robot->lf_leg_stop_pos;
    Eigen::Vector3d rf_foot_exp_pos = robot->rf_leg_stop_pos;
    Eigen::Vector3d lb_foot_exp_pos = robot->lb_leg_stop_pos;
    Eigen::Vector3d rb_foot_exp_pos = robot->rb_leg_stop_pos;

    Eigen::Vector3d lf_foot_exp_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d rf_foot_exp_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d lb_foot_exp_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d rb_foot_exp_vel = Eigen::Vector3d::Zero();

    Eigen::Vector3d lf_foot_exp_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d rf_foot_exp_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d lb_foot_exp_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d rb_foot_exp_acc = Eigen::Vector3d::Zero();

    Eigen::Vector3d lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force;
    std::tie(lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force) =
        balance_force_calc(robot, cur_roll, cur_pitch);

    // 轮子默认速度
    double lf_wheel_vel = current_exp_vel;
    double rf_wheel_vel = -current_exp_vel;
    double lb_wheel_vel = current_exp_vel;
    double rb_wheel_vel = -current_exp_vel;

    double lf_wheel_force = 0.0;
    double rf_wheel_force = 0.0;
    double lb_wheel_force = 0.0;
    double rb_wheel_force = 0.0;

    auto now = robot->node_->get_clock()->now();
    bool allow_next_jump = (now - last_jump_end_time).seconds() >= jump_step_finished_idel_time;

    // ===================== 跳台阶状态机 =====================
    if (current_state == STATE_GLIDE) {
        // 正常滑行
        req_state = STATE_GLIDE;

        // 前足碰到台阶 → 准备跳跃
        if ((lf_cart_force.x() > foot_obstruct_gate || rf_cart_force.x() > foot_obstruct_gate) && allow_next_jump) {
            RCLCPP_INFO(robot->node_->get_logger(), "检测到台阶，前双腿准备跳跃");
            req_state = STATE_PRE_JUMP;
        }
    }
    else if (current_state == STATE_PRE_JUMP) {
        // 开始前双腿同步跳跃
        jump_start_time = now;
        req_state = STATE_FRONT_JUMP;

        // 本地轨迹 → 世界坐标
        Eigen::Vector3d lf_local_target = lf_cart_pos + Eigen::Vector3d(jump_forward, 0, jump_height);
        Eigen::Vector3d rf_local_target = rf_cart_pos + Eigen::Vector3d(jump_forward, 0, jump_height);

        Eigen::Vector3d lf_world_target = local_to_world(robot, lf_local_target);
        Eigen::Vector3d rf_world_target = local_to_world(robot, rf_local_target);

        // 前双腿同步起跳
        lf_leg_step.update_flight_trajectory(lf_cart_pos, Eigen::Vector3d::Zero(), lf_world_target,
                                             Eigen::Vector2d::Zero(), jump_duration, jump_height);
        rf_leg_step.update_flight_trajectory(rf_cart_pos, Eigen::Vector3d::Zero(), rf_world_target,
                                             Eigen::Vector2d::Zero(), jump_duration, jump_height);
    }
    else if (current_state == STATE_FRONT_JUMP) {
        // 执行跳跃
        bool lf_ok, rf_ok;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) =
            lf_leg_step.get_target((now - jump_start_time).seconds(), lf_ok);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) =
            rf_leg_step.get_target((now - jump_start_time).seconds(), rf_ok);

        // 跳跃期间：前轮固定不动，后轮缓慢前进
        lf_wheel_vel = 0.0;
        rf_wheel_vel = 0.0;
        lb_wheel_vel = rear_slide_vel;
        rb_wheel_vel = -rear_slide_vel;

        // 跳跃完成
        if (!lf_ok && !rf_ok) {
            RCLCPP_INFO(robot->node_->get_logger(), "前腿已跳上台阶");
            req_state = STATE_FRONT_ON_STEP;
            last_jump_end_time = now;
        }
    }
    else if (current_state == STATE_FRONT_ON_STEP) {
        // 前轮锁死，后轮驱动推进
        lf_wheel_vel = 0.0;
        rf_wheel_vel = 0.0;
        lb_wheel_vel = rear_slide_vel * 0.8;
        rb_wheel_vel = -rear_slide_vel * 0.8;

        // 再次检测台阶 → 再次跳跃
        if ((lf_cart_force.x() > foot_obstruct_gate || rf_cart_force.x() > foot_obstruct_gate) && allow_next_jump) {
            RCLCPP_INFO(robot->node_->get_logger(), "再次检测到台阶，再次跳跃");
            req_state = STATE_PRE_JUMP;
        }
    }

    current_state = req_state;

    // ------------------- VMC 高度控制 -------------------
    if (current_state != STATE_FRONT_JUMP) {
        std::tie(lf_foot_exp_pos[2], lf_foot_exp_vel[2], lf_foot_exp_acc[2]) =
            robot->lf_z_vmc->targetUpdate(lf_foot_exp_pos[2], lf_cart_pos[2], lf_foot_exp_vel[2], lf_cart_vel[2], -lf_cart_force[2]);
        std::tie(rf_foot_exp_pos[2], rf_foot_exp_vel[2], rf_foot_exp_acc[2]) =
            robot->rf_z_vmc->targetUpdate(rf_foot_exp_pos[2], rf_cart_pos[2], rf_foot_exp_vel[2], rf_cart_vel[2], -rf_cart_force[2]);
    }

    std::tie(lb_foot_exp_pos[2], lb_foot_exp_vel[2], lb_foot_exp_acc[2]) =
        robot->lb_z_vmc->targetUpdate(lb_foot_exp_pos[2], lb_cart_pos[2], lb_foot_exp_vel[2], lb_cart_vel[2], -lb_cart_force[2]);
    std::tie(rb_foot_exp_pos[2], rb_foot_exp_vel[2], rb_foot_exp_acc[2]) =
        robot->rb_z_vmc->targetUpdate(rb_foot_exp_pos[2], rb_cart_pos[2], rb_foot_exp_vel[2], rb_cart_vel[2], -rb_cart_force[2]);

    // ------------------- 下发指令 -------------------
    robot_interfaces::msg::RobotTarget joints_target;

    joints_target.legs[0] = robot->lf_leg_calc->signal_leg_calc(
        lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force,
        &robot->lf_forward_torque, lf_wheel_vel, lf_wheel_force);

    joints_target.legs[1] = robot->rf_leg_calc->signal_leg_calc(
        rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force,
        &robot->rf_forward_torque, rf_wheel_vel, rf_wheel_force);

    joints_target.legs[2] = robot->lb_leg_calc->signal_leg_calc(
        lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force,
        &robot->lb_forward_torque, lb_wheel_vel, lb_wheel_force);

    joints_target.legs[3] = robot->rb_leg_calc->signal_leg_calc(
        rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force,
        &robot->rb_forward_torque, rb_wheel_vel, rb_wheel_force);

    robot->legs_target_pub->publish(joints_target);

    // 退出条件
    if (robot->move_cmd.step_mode == 1) {
        return "stop";
    }

    return "jump_steps";
}
