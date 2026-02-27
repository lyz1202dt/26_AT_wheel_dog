#include "states/idel.hpp"
#include "core/robot.hpp"

IdelState::IdelState(Robot* robot)
    : BaseState<Robot>("idel") {
    (void)robot;
}

bool IdelState::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    debug_cnt=0;
    return true;
}

std::string IdelState::update(Robot* robot) {
    // TODO:更新状态
    auto lf_foot_exp_pos = robot->lf_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);
    auto rf_foot_exp_pos = robot->rf_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);
    auto lb_foot_exp_pos = robot->lb_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);
    auto rb_foot_exp_pos = robot->rb_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);

    int result;
    // robot_interfaces::msg::Robot joints_target;
    // auto lf_joint_target = robot->lf_leg_calc->joint_pos(Vector3D(0.0, 0.0, 0.0), &result);
    // auto rf_joint_target = robot->rf_leg_calc->joint_pos(Vector3D(0.0, 0.0, 0.0), &result);
    // auto lb_joint_target = robot->lb_leg_calc->joint_pos(Vector3D(0.0, 0.0, 0.0), &result);
    // auto rb_joint_target = robot->rb_leg_calc->joint_pos(Vector3D(0.0, 0.0, 0.0), &result);
    // for (int i = 0; i < 3; i++) {
    //     joints_target.legs[0].joints[i].rad = static_cast<float>(lf_joint_target[i]);
    //     joints_target.legs[1].joints[i].rad = static_cast<float>(rf_joint_target[i]);
    //     joints_target.legs[2].joints[i].rad = static_cast<float>(lb_joint_target[i]);
    //     joints_target.legs[3].joints[i].rad = static_cast<float>(rb_joint_target[i]);
    // }
    // robot->legs_target_pub->publish(joints_target);
    auto cur_pos=robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
    // debug_cnt++;
    // if(debug_cnt==10)
    // {
    //     debug_cnt=0;
    //     RCLCPP_INFO(robot->node_->get_logger(),"result=%d,offset=(%lf,%lf,%lf),lf_leg=(%lf,%lf,%lf)",result,robot->lf_leg_calc->pos_offset[0],robot->lf_leg_calc->pos_offset[1],robot->lf_leg_calc->pos_offset[2],cur_pos[0],cur_pos[1],cur_pos[2]);
    // }
    if (robot->move_cmd.step_mode == 1) // 如果希望跳转到STOP状态（VMC站立），那么跳转
        return "stop";
    return "idel";
}
