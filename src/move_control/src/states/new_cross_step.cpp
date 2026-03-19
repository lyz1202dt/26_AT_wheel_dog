#include "states/new_cross_step.hpp"
#include "core/robot.hpp"

JumpStepState::JumpStepState(Robot* robot)
    : BaseState<Robot>("jump_step") {

    robot->node_->declare_parameter("jump_height", 0.12);
    robot->node_->declare_parameter("jump_forward", 0.05);
    robot->node_->declare_parameter("jump_duration", 0.25);
    robot->node_->declare_parameter("rear_slide_vel", 0.10);
    robot->node_->declare_parameter("foot_obstacle_threshold", 18.0);
    robot->node_->declare_parameter("jump_cooldown_time", 0.5);
    robot->node_->declare_parameter("exp_vel_kp", 2.0);

    robot->add_param_cb([this](const rclcpp::Parameter& param) {
        auto name = param.get_name();
        if (name == "jump_height")
            jump_height = param.as_double();
        else if (name == "jump_forward")
            jump_forward = param.as_double();
        else if (name == "jump_duration")
            jump_duration = param.as_double();
        else if (name == "rear_slide_vel")
            rear_slide_vel = param.as_double();
        else if (name == "foot_obstacle_threshold")
            foot_obstacle_threshold = param.as_double();
        else if (name == "jump_cooldown_time")
            jump_cooldown_time = param.as_double();
        else if (name == "exp_vel_kp")
            exp_vel_kp = param.as_double();
        return true;
    });
}

bool JumpStepState::enter(Robot* robot, const std::string& last_status) {
    (void)last_status;
    RCLCPP_INFO(robot->node_->get_logger(), "==== 进入 跳台阶 模式 ====");

    // 初始化状态
    req_state = STATE_GLIDE;
    current_state = STATE_GLIDE;

    // 时间
    auto now = robot->node_->get_clock()->now();
    jump_start_time = now;
    last_jump_finish_time = now;

    // 力清零
    lf_cart_force.setZero();
    rf_cart_force.setZero();
    lb_cart_force.setZero();
    rb_cart_force.setZero();

    current_body_vel = 0.0;
    return true;
}

// ===================== 局部坐标转世界坐标（姿态参与计算） =====================
Vector3D JumpStepState::local_to_world(Robot* robot, const Vector3D& local_pos) {
    double cur_roll, cur_pitch, cur_yaw;
    tf2::Matrix3x3(robot->robot_rotation).getRPY(cur_roll, cur_pitch, cur_yaw);

    tf2::Quaternion q;
    q.setRPY(cur_roll, cur_pitch, cur_yaw);
    Eigen::Quaterniond eigen_q(q);
    Eigen::Matrix3d rot_mat = eigen_q.toRotationMatrix();

    // 机身姿态旋转 → 得到世界坐标系下足端位置
    Vector3D world_pos = rot_mat * local_pos;
    return world_pos;
}

// ===================== 平衡计算（和walk完全同风格） =====================
std::tuple<Vector3D, Vector3D, Vector3D, Vector3D>
JumpStepState::balance_force_calc(Robot* robot, double cur_roll, double cur_pitch) {
    double sin_pitch = std::sin(cur_pitch);
    double sin_roll = std::sin(cur_roll);

    double roll_torque = robot->roll_vmc->update(cur_roll, robot->robot_velocity.angular.x, 0.0);
    double pitch_torque = robot->pitch_vmc->update(cur_pitch, robot->robot_velocity.angular.y, 0.0);

    Vector3D lf_force = Vector3D::Zero();
    Vector3D rf_force = Vector3D::Zero();
    Vector3D lb_force = Vector3D::Zero();
    Vector3D rb_force = Vector3D::Zero();

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

// ===================== 主循环 update（核心逻辑） =====================
std::string JumpStepState::update(Robot* robot) {
    std::string next_state = "jump_step";

    // 期望足端变量
    Vector3D lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos;
    Vector3D lf_foot_exp_vel, rf_foot_exp_vel, lb_foot_exp_vel, rb_foot_exp_vel;
    Vector3D lf_foot_exp_acc, rf_foot_exp_acc, lb_foot_exp_acc, rb_foot_exp_acc;
    Vector3D lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force;

    // 读取姿态
    double cur_roll, cur_pitch, cur_yaw;
    tf2::Matrix3x3(robot->robot_rotation).getRPY(cur_roll, cur_pitch, cur_yaw);

    // 倾倒保护
    if (fabs(cur_roll) > 45 * M_PI / 180 || fabs(cur_pitch) > 55 * M_PI / 180) {
        return "idel";
    }

    // 平衡力
    std::tie(lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force) =
        balance_force_calc(robot, cur_roll, cur_pitch);

    // ------------------- 足端实时状态 -------------------
    auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
    auto lf_cart_vel = robot->lf_leg_calc->foot_vel(robot->lf_joint_pos, robot->lf_joint_vel);
    lf_cart_force = 0.2 * robot->lf_leg_calc->foot_force(robot->lf_joint_pos, robot->lf_joint_torque, robot->lf_forward_torque)
                  + 0.8 * lf_cart_force;

    auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
    auto rf_cart_vel = robot->rf_leg_calc->foot_vel(robot->rf_joint_pos, robot->rf_joint_vel);
    rf_cart_force = 0.2 * robot->rf_leg_calc->foot_force(robot->rf_joint_pos, robot->rf_joint_torque, robot->rf_forward_torque)
                  + 0.8 * rf_cart_force;

    auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
    auto lb_cart_vel = robot->lb_leg_calc->foot_vel(robot->lb_joint_pos, robot->lb_joint_vel);
    lb_cart_force = 0.2 * robot->lb_leg_calc->foot_force(robot->lb_joint_pos, robot->lb_joint_torque, robot->lb_forward_torque)
                  + 0.8 * lb_cart_force;

    auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
    auto rb_cart_vel = robot->rb_leg_calc->foot_vel(robot->rb_joint_pos, robot->rb_joint_vel);
    rb_cart_force = 0.2 * robot->rb_leg_calc->foot_force(robot->rb_joint_pos, robot->rb_joint_torque, robot->rb_forward_torque)
                  + 0.8 * rb_cart_force;

    // ------------------- 默认：后轮保持默认位置 -------------------
    lb_foot_exp_pos = robot->lb_leg_stop_pos;
    rb_foot_exp_pos = robot->rb_leg_stop_pos;
    lb_foot_exp_vel = Vector3D::Zero();
    rb_foot_exp_vel = Vector3D::Zero();

    // ------------------- 默认轮速 -------------------
    double lf_wheel_vel = robot->move_cmd.vx;
    double rf_wheel_vel = -robot->move_cmd.vx;
    double lb_wheel_vel = robot->move_cmd.vx;
    double rb_wheel_vel = -robot->move_cmd.vx;

    double lf_wheel_force = 0.0;
    double rf_wheel_force = 0.0;
    double lb_wheel_force = 0.0;
    double rb_wheel_force = 0.0;

    auto now = robot->node_->get_clock()->now();
    bool can_jump_again = (now - last_jump_finish_time).seconds() >= jump_cooldown_time;

    // ===================== 状态机 =====================
    if (current_state == STATE_GLIDE) {
        // 正常滑行
        req_state = STATE_GLIDE;

        // 前轮碰到台阶 → 准备跳
        if ((lf_cart_force.x() > foot_obstacle_threshold ||
             rf_cart_force.x() > foot_obstacle_threshold) && can_jump_again)
        {
            RCLCPP_INFO(robot->node_->get_logger(), "检测到台阶，准备前腿跳跃");
            req_state = STATE_PRE_JUMP;
        }
    }
    else if (current_state == STATE_PRE_JUMP) {
        // 开始规划前腿跳跃轨迹
        jump_start_time = now;
        req_state = STATE_FRONT_LEGS_JUMP;

        // 局部目标点
        Vector3D lf_local = lf_cart_pos + Vector3D(jump_forward, 0, jump_height);
        Vector3D rf_local = rf_cart_pos + Vector3D(jump_forward, 0, jump_height);

        // 转世界坐标系
        Vector3D lf_world = local_to_world(robot, lf_local);
        Vector3D rf_world = local_to_world(robot, rf_local);

        // 前腿轨迹
        lf_leg_step.update_flight_trajectory(lf_cart_pos, Vector3D::Zero(), lf_world,
                                             Vector2D::Zero(), jump_duration, jump_height);
        rf_leg_step.update_flight_trajectory(rf_cart_pos, Vector3D::Zero(), rf_world,
                                             Vector2D::Zero(), jump_duration, jump_height);
    }
    else if (current_state == STATE_FRONT_LEGS_JUMP) {
        // 执行跳跃
        bool lf_ok, rf_ok;
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) =
            lf_leg_step.get_target((now - jump_start_time).seconds(), lf_ok);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) =
            rf_leg_step.get_target((now - jump_start_time).seconds(), rf_ok);

        // 跳跃时：前轮锁死，后轮缓慢滑动
        lf_wheel_vel = 0.0;
        rf_wheel_vel = 0.0;
        lb_wheel_vel = rear_slide_vel;
        rb_wheel_vel = -rear_slide_vel;

        // 跳跃完成
        if (!lf_ok && !rf_ok) {
            RCLCPP_INFO(robot->node_->get_logger(), "前腿已跳上台阶");
            req_state = STATE_FRONT_ON_STEP;
            last_jump_finish_time = now;
        }
    }
    else if (current_state == STATE_FRONT_ON_STEP) {
        // 前轮已上台：前轮不动，后轮驱动前进
        lf_wheel_vel = 0.0;
        rf_wheel_vel = 0.0;
        lb_wheel_vel = rear_slide_vel * 0.8;
        rb_wheel_vel = -rear_slide_vel * 0.8;

        // 再次碰到台阶 → 再次跳跃
        if ((lf_cart_force.x() > foot_obstacle_threshold ||
             rf_cart_force.x() > foot_obstacle_threshold) && can_jump_again)
        {
            RCLCPP_INFO(robot->node_->get_logger(), "再次检测到台阶，再次跳跃");
            req_state = STATE_PRE_JUMP;
        }
    }

    current_state = req_state;

    // ------------------- VMC 高度控制（同walk风格） -------------------
    if (current_state != STATE_FRONT_LEGS_JUMP) {
        std::tie(lf_foot_exp_pos[2], lf_foot_exp_vel[2], lf_foot_exp_acc[2]) =
            robot->lf_z_vmc->targetUpdate(lf_foot_exp_pos[2], lf_cart_pos[2], lf_foot_exp_vel[2], lf_cart_vel[2], -lf_cart_force[2]);
        std::tie(rf_foot_exp_pos[2], rf_foot_exp_vel[2], rf_foot_exp_acc[2]) =
            robot->rf_z_vmc->targetUpdate(rf_foot_exp_pos[2], rf_cart_pos[2], rf_foot_exp_vel[2], rf_cart_vel[2], -rf_cart_force[2]);
    }

    std::tie(lb_foot_exp_pos[2], lb_foot_exp_vel[2], lb_foot_exp_acc[2]) =
        robot->lb_z_vmc->targetUpdate(lb_foot_exp_pos[2], lb_cart_pos[2], lb_foot_exp_vel[2], lb_cart_vel[2], -lb_cart_force[2]);
    std::tie(rb_foot_exp_pos[2], rb_foot_exp_vel[2], rb_foot_exp_acc[2]) =
        robot->rb_z_vmc->targetUpdate(rb_foot_exp_pos[2], rb_cart_pos[2], rb_foot_exp_vel[2], rb_cart_vel[2], -rb_cart_force[2]);

    // ------------------- 下发指令（完全同walk） -------------------
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

    // 停止条件
    if (robot->move_cmd.step_mode == 1) {
        return "stop";
    }

    return next_state;
}
