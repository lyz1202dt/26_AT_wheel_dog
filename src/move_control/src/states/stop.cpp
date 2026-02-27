#include "states/stop.hpp"
#include "core/robot.hpp"
#include <Eigen/Dense>

StopState::StopState(Robot* robot)
    : BaseState<Robot>("stop") {
        (void)robot;
    }

bool StopState::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    return true;
}

std::string StopState::update(Robot* robot) {
    Eigen::Vector3d lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos;
    Eigen::Vector3d lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force;
    Eigen::Vector3d lf_foot_exp_vel, rf_foot_exp_vel, lb_foot_exp_vel, rb_foot_exp_vel;
    Eigen::Vector3d lf_foot_exp_acc, rf_foot_exp_acc, lb_foot_exp_acc, rb_foot_exp_acc;

    double cur_roll, cur_pitch, cur_yaw;
    tf2::Matrix3x3(robot->robot_rotation).getRPY(cur_roll, cur_pitch, cur_yaw);
    std::tie(lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force) = balance_force_calc(robot, cur_roll, cur_pitch);

    if ((cur_roll > 40 * 3.14 / 180 || cur_roll < -40 * 3.14 / 180 || cur_pitch > 50 * 3.14 / 180
         || cur_pitch < -50 * 3.14 / 180)) // 机器人倾倒，切入IDEL状态
        return "idel";

    lf_foot_exp_pos = robot->lf_leg_stop_pos;
    rf_foot_exp_pos = robot->rf_leg_stop_pos;
    lb_foot_exp_pos = robot->lb_leg_stop_pos;
    rb_foot_exp_pos = robot->rb_leg_stop_pos;

    auto lf_cart_pos   = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
    auto lf_cart_vel   = robot->lf_leg_calc->foot_vel(robot->lf_joint_pos, robot->lf_joint_vel);
    auto lf_cart_force = robot->lf_leg_calc->foot_force(robot->lf_joint_pos, robot->lf_joint_torque, robot->lf_forward_torque);
    std::tie(lf_foot_exp_pos[2], lf_foot_exp_vel[2], lf_foot_exp_acc[2]) =
        robot->lf_z_vmc->targetUpdate(0.0, lf_cart_pos[2], 0.0, lf_cart_vel[2], -lf_cart_force[2]);
    lf_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_lf_grivate);

    auto rf_cart_pos   = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
    auto rf_cart_vel   = robot->rf_leg_calc->foot_vel(robot->rf_joint_pos, robot->rf_joint_vel);
    auto rf_cart_force = robot->rf_leg_calc->foot_force(robot->rf_joint_pos, robot->rf_joint_torque, robot->rf_forward_torque);
    std::tie(rf_foot_exp_pos[2], rf_foot_exp_vel[2], rf_foot_exp_acc[2]) =
        robot->rf_z_vmc->targetUpdate(0.0, rf_cart_pos[2], 0.0, rf_cart_vel[2], -rf_cart_force[2]);
    rf_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_rf_grivate);

    auto lb_cart_pos   = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
    auto lb_cart_vel   = robot->lb_leg_calc->foot_vel(robot->lb_joint_pos, robot->lb_joint_vel);
    auto lb_cart_force = robot->lb_leg_calc->foot_force(robot->lb_joint_pos, robot->lb_joint_torque, robot->lb_forward_torque);
    std::tie(lb_foot_exp_pos[2], lb_foot_exp_vel[2], lb_foot_exp_acc[2]) =
        robot->lb_z_vmc->targetUpdate(0.0, lb_cart_pos[2], 0.0, lb_cart_vel[2], -lb_cart_force[2]);
    lb_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_lb_grivate);

    auto rb_cart_pos   = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
    auto rb_cart_vel   = robot->rb_leg_calc->foot_vel(robot->rb_joint_pos, robot->rb_joint_vel);
    auto rb_cart_force = robot->rb_leg_calc->foot_force(robot->rb_joint_pos, robot->rb_joint_torque, robot->rb_forward_torque);
    std::tie(rb_foot_exp_pos[2], rb_foot_exp_vel[2], rb_foot_exp_acc[2]) =
        robot->rb_z_vmc->targetUpdate(0.0, rb_cart_pos[2], 0.0, rb_cart_vel[2], -rb_cart_force[2]);
    rb_foot_exp_force += Vector3D(0.0, 0.0, -robot->robot_rb_grivate);


    robot_interfaces::msg::Robot joints_target;
    joints_target.legs[0] = robot->signal_leg_calc(
        lf_foot_exp_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), lf_foot_exp_force, robot->lf_leg_calc,
        &robot->lf_forward_torque);
    joints_target.legs[1] = robot->signal_leg_calc(
        rf_foot_exp_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), rf_foot_exp_force, robot->rf_leg_calc,
        &robot->rf_forward_torque);
    joints_target.legs[2] = robot->signal_leg_calc(
        lb_foot_exp_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), lb_foot_exp_force, robot->lb_leg_calc,
        &robot->lb_forward_torque);
    joints_target.legs[3] = robot->signal_leg_calc(
        rb_foot_exp_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), rb_foot_exp_force, robot->rb_leg_calc,
        &robot->rb_forward_torque);
    robot->legs_target_pub->publish(joints_target);

    return "stop";
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
    StopState::balance_force_calc(Robot* robot, double cur_roll, double cur_pitch) {

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
