#include "states/force.hpp"
#include "core/robot.hpp"
#include <tf2/LinearMath/Quaternion.h>


ForceState::ForceState(Robot* robot)
    : BaseState<Robot>("force")
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
    robot->node_->declare_parameter("force_vmc_z_kp", 200.0);
    robot->node_->declare_parameter("force_vmc_z_kd", 5.0);

    // 声明x、y方向VMC的PD参数
    robot->node_->declare_parameter("force_vmc_xy_kp", 100.0);
    robot->node_->declare_parameter("force_vmc_xy_kd", 2.0);

    // 声明roll和pitch轴的VMC参数
    robot->node_->declare_parameter("force_vmc_roll_kp", 0.0);
    robot->node_->declare_parameter("force_vmc_roll_kd", 0.0);
    robot->node_->declare_parameter("force_vmc_pitch_kp", 0.0);
    robot->node_->declare_parameter("force_vmc_pitch_kd", 0.0);

    // 添加参数变化回调函数
    robot->add_param_cb([this](const rclcpp::Parameter& param) {
        auto name = param.get_name();
        if (name == "force_vmc_z_kp") {
            double kp       = param.as_double();
            lf_leg_vmc_z.kp = kp;
            rf_leg_vmc_z.kp = kp;
            lb_leg_vmc_z.kp = kp;
            rb_leg_vmc_z.kp = kp;
        } else if (name == "force_vmc_z_kd") {
            double kd       = param.as_double();
            lf_leg_vmc_z.kd = kd;
            rf_leg_vmc_z.kd = kd;
            lb_leg_vmc_z.kd = kd;
            rb_leg_vmc_z.kd = kd;
        } else if (name == "force_vmc_xy_kp") {
            double kp       = param.as_double();
            lf_leg_vmc_x.kp = kp;
            rf_leg_vmc_x.kp = kp;
            lb_leg_vmc_x.kp = kp;
            rb_leg_vmc_x.kp = kp;
            lf_leg_vmc_y.kp = kp;
            rf_leg_vmc_y.kp = kp;
            lb_leg_vmc_y.kp = kp;
            rb_leg_vmc_y.kp = kp;
        } else if (name == "force_vmc_xy_kd") {
            double kd       = param.as_double();
            lf_leg_vmc_x.kd = kd;
            rf_leg_vmc_x.kd = kd;
            lb_leg_vmc_x.kd = kd;
            rb_leg_vmc_x.kd = kd;
            lf_leg_vmc_y.kd = kd;
            rf_leg_vmc_y.kd = kd;
            lb_leg_vmc_y.kd = kd;
            rb_leg_vmc_y.kd = kd;
        } else if (name == "force_vmc_roll_kp")
            roll_vmc.kp = param.as_double();
        else if (name == "force_vmc_roll_kd")
            roll_vmc.kd = param.as_double();
        else if (name == "force_vmc_pitch_kp")
            pitch_vmc.kp = param.as_double();
        else if (name == "force_vmc_pitch_kd")
            pitch_vmc.kd = param.as_double();
        RCLCPP_INFO(rclcpp::get_logger("logger"), "参数更新");
        return true;
    });
}

bool ForceState::enter(Robot* robot, const std::string& last_status) {
    (void)last_status;
    double z_kp, z_kd, xy_kp, xy_kd;
    robot->node_->get_parameter("force_vmc_z_kp", z_kp);
    robot->node_->get_parameter("force_vmc_z_kd", z_kd);
    robot->node_->get_parameter("force_vmc_xy_kp", xy_kp);
    robot->node_->get_parameter("force_vmc_xy_kd", xy_kd);
    robot->node_->get_parameter("force_vmc_roll_kp", roll_vmc.kp);
    robot->node_->get_parameter("force_vmc_roll_kd", roll_vmc.kd);
    robot->node_->get_parameter("force_vmc_pitch_kp", pitch_vmc.kp);
    robot->node_->get_parameter("force_vmc_pitch_kd", pitch_vmc.kd);

    lf_leg_vmc_z.kp = z_kp;
    rf_leg_vmc_z.kp = z_kp;
    lb_leg_vmc_z.kp = z_kp;
    rb_leg_vmc_z.kp = z_kp;
    lf_leg_vmc_z.kd = z_kd;
    rf_leg_vmc_z.kd = z_kd;
    lb_leg_vmc_z.kd = z_kd;
    rb_leg_vmc_z.kd = z_kd;

    lf_leg_vmc_x.kp = xy_kp;
    rf_leg_vmc_x.kp = xy_kp;
    lb_leg_vmc_x.kp = xy_kp;
    rb_leg_vmc_x.kp = xy_kp;
    lf_leg_vmc_y.kp = xy_kp;
    rf_leg_vmc_y.kp = xy_kp;
    lb_leg_vmc_y.kp = xy_kp;
    rb_leg_vmc_y.kp = xy_kp;

    lf_leg_vmc_x.kd = xy_kd;
    rf_leg_vmc_x.kd = xy_kd;
    lb_leg_vmc_x.kd = xy_kd;
    rb_leg_vmc_x.kd = xy_kd;
    lf_leg_vmc_y.kd = xy_kd;
    rf_leg_vmc_y.kd = xy_kd;
    lb_leg_vmc_y.kd = xy_kd;
    rb_leg_vmc_y.kd = xy_kd;

    return true;
}

std::string ForceState::update(Robot* robot) {
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

    lf_cart_vel=filter_gate*lf_cart_vel+(1.0-filter_gate)*lf_last_vel;
    rf_cart_vel=filter_gate*rf_cart_vel+(1.0-filter_gate)*rf_last_vel;
    lb_cart_vel=filter_gate*lb_cart_vel+(1.0-filter_gate)*lb_last_vel;
    rb_cart_vel=filter_gate*rb_cart_vel+(1.0-filter_gate)*rb_last_vel;


    // 单腿VMC计算
    lf_foot_exp_force[2] = lf_leg_vmc_z.update(lf_cart_pos[2], lf_cart_vel[2]);
    lf_foot_exp_force[0] = lf_leg_vmc_x.update(lf_cart_pos[0], lf_cart_vel[0]);
    lf_foot_exp_force[1] = lf_leg_vmc_y.update(lf_cart_pos[1], lf_cart_vel[1]);

    rf_foot_exp_force[2] = rf_leg_vmc_z.update(rf_cart_pos[2], rf_cart_vel[2]);
    rf_foot_exp_force[0] = rf_leg_vmc_x.update(rf_cart_pos[0], rf_cart_vel[0]);
    rf_foot_exp_force[1] = rf_leg_vmc_y.update(rf_cart_pos[1], rf_cart_vel[1]);

    lb_foot_exp_force[2] = lb_leg_vmc_z.update(lb_cart_pos[2], lb_cart_vel[2]);
    lb_foot_exp_force[0] = lb_leg_vmc_x.update(lb_cart_pos[0], lb_cart_vel[0]);
    lb_foot_exp_force[1] = lb_leg_vmc_y.update(lb_cart_pos[1], lb_cart_vel[1]);

    rb_foot_exp_force[2] = rb_leg_vmc_z.update(rb_cart_pos[2], rb_cart_vel[2]);
    rb_foot_exp_force[0] = rb_leg_vmc_x.update(rb_cart_pos[0], rb_cart_vel[0]);
    rb_foot_exp_force[1] = rb_leg_vmc_y.update(rb_cart_pos[1], rb_cart_vel[1]);


    // 重力补偿
     lf_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_lf_grivate);
     rf_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_rf_grivate);
     lb_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_lb_grivate);
     rb_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_rb_grivate);


    // 狗身平衡
    double cur_roll, cur_pitch, cur_yaw;
    auto robot_rot_mat = tf2::Matrix3x3(robot->robot_rotation);
    robot_rot_mat.getRPY(cur_roll, cur_pitch, cur_yaw);
    double pitch_torque = pitch_vmc.update(cur_pitch, robot->robot_velocity.angular.y);
    double roll_torque  = roll_vmc.update(cur_roll, robot->robot_velocity.angular.x);

    // tf2::Quaternion q;
    // robot_rot_mat.getRotation(q);
    // Eigen::Matrix3d rot_mat_eigen = Eigen::Quaterniond(q.x(), q.y(), q.z(), q.w()).toRotationMatrix();

    // auto lf_foot_pos_body  = Vector3D(0.0, 0.0, -robot->body_height) + lf_cart_pos;
    // auto lf_foot_pos_world = rot_mat_eigen * lf_foot_pos_body;
    // auto lf_pos_2d         = lf_foot_pos_world.head(2);

    // auto rf_foot_pos_body  = Vector3D(0.0, 0.0, -robot->body_height) + rf_cart_pos;
    // auto rf_foot_pos_world = rot_mat_eigen * rf_foot_pos_body;
    // auto rf_pos_2d         = rf_foot_pos_world.head(2);

    // auto lb_foot_pos_body  = Vector3D(0.0, 0.0, -robot->body_height) + lb_cart_pos;
    // auto lb_foot_pos_world = rot_mat_eigen * lb_foot_pos_body;
    // auto lb_pos_2d         = lb_foot_pos_world.head(2);

    // auto rb_foot_pos_body  = Vector3D(0.0, 0.0, -robot->body_height) + rb_cart_pos;
    // auto rb_foot_pos_world = rot_mat_eigen * rb_foot_pos_body;
    // auto rb_pos_2d         = rb_foot_pos_world.head(2);


    // Eigen::Matrix<double, 3, 4> A;
    // Eigen::Vector3d b;

    // // 第一行: 力平衡约束
    // A(0, 0) = 1.0; // lf
    // A(0, 1) = 1.0; // rf
    // A(0, 2) = 1.0; // lb
    // A(0, 3) = 1.0; // rb
    // b(0)    = mass * 9.8;

    // // 第二行: 绕质心的x方向力矩平衡
    // A(1, 0) = lf_pos_2d.y();
    // A(1, 1) = rf_pos_2d.y();
    // A(1, 2) = lb_pos_2d.y();
    // A(1, 3) = rb_pos_2d.y();

    // // τy = -x * Fz
    // A(2, 0) = -(lf_pos_2d.x());
    // A(2, 1) = -(rf_pos_2d.x());
    // A(2, 2) = -(lb_pos_2d.x());
    // A(2, 3) = -(rb_pos_2d.x());
    // b(2)    = 0.0;

    // // 使用最小二乘法求解超定方程组
    // Eigen::Vector4d forces = A.completeOrthogonalDecomposition().solve(b);

    // // 将求解的垂直力存入各腿的期望力向量
    // lf_foot_exp_force += rot_mat_eigen.transpose() * Vector3D(0.0, 0.0, -forces(0));
    // rf_foot_exp_force += rot_mat_eigen.transpose() * Vector3D(0.0, 0.0, -forces(1));
    // lb_foot_exp_force += rot_mat_eigen.transpose() * Vector3D(0.0, 0.0, -forces(2));
    // rb_foot_exp_force += rot_mat_eigen.transpose() * Vector3D(0.0, 0.0, -forces(3));

    double lf_force = pitch_torque / (robot->lf_leg_calc->pos_offset[0] +lf_cart_pos[0])/4;
    double rf_force = pitch_torque / (robot->rf_leg_calc->pos_offset[0] +rf_cart_pos[0])/4;
    double lb_force = pitch_torque / (robot->lb_leg_calc->pos_offset[0] +lb_cart_pos[0])/4;
    double rb_force = pitch_torque / (robot->rb_leg_calc->pos_offset[0] +rb_cart_pos[0])/4;

    lf_force += roll_torque / (robot->lf_leg_calc->pos_offset[1] +lf_cart_pos[1])/4;
    rf_force += roll_torque / (robot->rf_leg_calc->pos_offset[1] +rf_cart_pos[1])/4;
    lb_force += roll_torque / (robot->lb_leg_calc->pos_offset[1] +lb_cart_pos[1])/4;
    rb_force += roll_torque / (robot->rb_leg_calc->pos_offset[1] +rb_cart_pos[1])/4;

    lf_foot_exp_force[2] += lf_force;
    rf_foot_exp_force[2] += rf_force;
    lb_foot_exp_force[2] += lb_force;
    rb_foot_exp_force[2] += rb_force;


    // RCLCPP_INFO(robot->node_->get_logger(), "roll_angular=%lf" ,robot->robot_velocity.angular.x);


    auto lf_torque = signal_leg_torque_calc(robot->lf_leg_calc, robot->lf_joint_pos, lf_foot_exp_force, lf_foot_exp_vel, lf_foot_exp_acc);
    auto rf_torque = signal_leg_torque_calc(robot->rf_leg_calc, robot->rf_joint_pos, rf_foot_exp_force, rf_foot_exp_vel, rf_foot_exp_acc);
    auto lb_torque = signal_leg_torque_calc(robot->lb_leg_calc, robot->lb_joint_pos, lb_foot_exp_force, lb_foot_exp_vel, lb_foot_exp_acc);
    auto rb_torque = signal_leg_torque_calc(robot->rb_leg_calc, robot->rb_joint_pos, rb_foot_exp_force, rb_foot_exp_vel, rb_foot_exp_acc);

    if (lf_torque.hasNaN() || rf_torque.hasNaN() || lb_torque.hasNaN() || rb_torque.hasNaN()) {
        RCLCPP_ERROR(robot->node_->get_logger(), "计算出现NAN");
        return "force";
    }

    robot_interfaces::msg::RobotTarget joints_target;
    for (int i = 0; i < 3; i++) {
        joints_target.legs[0].joints[i].torque = static_cast<float>(lf_torque[i]);
        joints_target.legs[1].joints[i].torque = static_cast<float>(rf_torque[i]);
        joints_target.legs[2].joints[i].torque = static_cast<float>(lb_torque[i]);
        joints_target.legs[3].joints[i].torque = static_cast<float>(rb_torque[i]);

        joints_target.legs[0].joints[i].kp = 0.0f;
        joints_target.legs[1].joints[i].kp = 0.0f;
        joints_target.legs[2].joints[i].kp = 0.0f;
        joints_target.legs[3].joints[i].kp = 0.0f;

        joints_target.legs[0].joints[i].kd = 0.0f;
        joints_target.legs[1].joints[i].kd = 0.0f;
        joints_target.legs[2].joints[i].kd = 0.0f;
        joints_target.legs[3].joints[i].kd = 0.0f;
    }
    robot->legs_target_pub->publish(joints_target);
    if (robot->move_cmd.step_mode == 20)
        return "idel";
    return "force";
}

Vector3D ForceState::signal_leg_torque_calc(
    std::shared_ptr<LegCalc> leg_calc, const Vector3D& cur_joint_pos, const Vector3D& exp_foot_force, const Vector3D& foot_vel,
    const Vector3D& foot_acc) {
    Vector3D joint_torque;
    joint_torque = leg_calc->joint_torque_foot_force(cur_joint_pos, exp_foot_force);
    joint_torque += leg_calc->joint_torque_dynamic(cur_joint_pos, leg_calc->joint_vel(cur_joint_pos, foot_vel), foot_acc);
    return joint_torque;
}


std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
    ForceState::balance_force_calc(Robot* robot, double cur_roll, double cur_pitch) {

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
