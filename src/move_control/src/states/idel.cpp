#include "states/idel.hpp"
#include "core/robot.hpp"

IdelState::IdelState(Robot* robot)
    : BaseState<Robot>("idel") {
    (void)robot;
}

bool IdelState::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    debug_cnt = 0;
    return true;
}

std::string IdelState::update(Robot* robot) {
    // TODO:更新状态
    auto lf_foot_exp_pos = robot->lf_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);
    auto rf_foot_exp_pos = robot->rf_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);
    auto lb_foot_exp_pos = robot->lb_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);
    auto rb_foot_exp_pos = robot->rb_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);

    int result;
    robot_interfaces::msg::RobotTarget joints_target;
    auto lf_joint_target = robot->lf_leg_calc->joint_pos(Vector3D(0.0, 0.0, 0.0), &result);
    auto rf_joint_target = robot->rf_leg_calc->joint_pos(Vector3D(0.0, 0.0, 0.0), &result);
    auto lb_joint_target = robot->lb_leg_calc->joint_pos(Vector3D(0.0, 0.0, 0.0), &result);
    auto rb_joint_target = robot->rb_leg_calc->joint_pos(Vector3D(0.0, 0.0, 0.0), &result);
    for (int i = 0; i < 3; i++) {
        joints_target.legs[0].joints[i].rad = static_cast<float>(lf_joint_target[i]);
        joints_target.legs[1].joints[i].rad = static_cast<float>(rf_joint_target[i]);
        joints_target.legs[2].joints[i].rad = static_cast<float>(lb_joint_target[i]);
        joints_target.legs[3].joints[i].rad = static_cast<float>(rb_joint_target[i]);

        joints_target.legs[0].joints[i].omega = 0.0f;
        joints_target.legs[1].joints[i].omega = 0.0f;
        joints_target.legs[2].joints[i].omega = 0.0f;
        joints_target.legs[3].joints[i].omega = 0.0f;

        joints_target.legs[0].joints[i].torque = 0.0f;
        joints_target.legs[1].joints[i].torque = 0.0f;
        joints_target.legs[2].joints[i].torque = 0.0f;
        joints_target.legs[3].joints[i].torque = 0.0f;
    }
    for (int i = 0; i < 4; i++) {
        joints_target.legs[i].wheel.omega  = 0.0f;
        joints_target.legs[i].wheel.torque = 0.0f;
        for (int j = 0; j < 3; j++) {
            joints_target.legs[i].joints[j].kp = robot->lf_leg_calc->kp[j];
            joints_target.legs[i].joints[j].kd = robot->lf_leg_calc->kd[j];
        }
        joints_target.legs[i].wheel.kd = robot->lf_leg_calc->wheel_kd;
    }

    auto lf_current_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
    auto rf_current_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
    auto lb_current_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
    auto rb_current_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
    static int cnt = 0;
    cnt++;
    if(cnt>=100)
    {
        cnt = 0;
        RCLCPP_ERROR(robot->node_->get_logger(),"\033[lf_current_pos = (%.2f, %.2f, %.2f) rf_current_pos = (%.2f, %.2f, %.2f) lb_current_pos = (%.2f, %.2f, %.2f) rb_current_pos = (%.2f, %.2f, %.2f)\033[0m", 
        lf_current_pos[0], lf_current_pos[1], lf_current_pos[2], 
        rf_current_pos[0], rf_current_pos[1], rf_current_pos[2], 
        lb_current_pos[0], lb_current_pos[1], lb_current_pos[2], 
        rb_current_pos[0], rb_current_pos[1], rb_current_pos[2]);
    }
    
    robot->legs_target_pub->publish(joints_target);
    if (robot->move_cmd.step_mode == 1)  // 如果希望跳转到STOP状态（VMC站立），那么跳转
        return "stop";
    if (robot->move_cmd.step_mode == 10) // 切到测试纯VMC状态
        return "force";
    return "idel";
}
