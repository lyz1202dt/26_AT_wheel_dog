#include "states/idel.hpp"
#include "core/robot.hpp"

IdelState::IdelState(Robot* robot)
    : BaseState<Robot>("idel") {}

bool IdelState::enter(Robot* robot, const std::string& last_status) { return true; }

std::string IdelState::update(Robot* robot) {
    // TODO:更新状态
    auto lf_foot_exp_pos = robot->lf_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);
    auto rf_foot_exp_pos = robot->rf_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);
    auto lb_foot_exp_pos = robot->lb_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);
    auto rb_foot_exp_pos = robot->rb_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);

    int result;
    robot_interfaces::msg::Robot joints_target;
    auto lf_joint_target=robot->lf_leg_calc->joint_pos(lf_foot_exp_pos, &result);
    auto rf_joint_target=robot->rf_leg_calc->joint_pos(rf_foot_exp_pos, &result);
    auto lb_joint_target=robot->lb_leg_calc->joint_pos(lb_foot_exp_pos, &result);
    auto rb_joint_target=robot->rb_leg_calc->joint_pos(rb_foot_exp_pos, &result);
    for(int i=0;i<3;i++)
    {
        joints_target.legs[0].joints[i].rad=static_cast<float>(lf_joint_target[i]);
        joints_target.legs[1].joints[i].rad=static_cast<float>(rf_joint_target[i]);
        joints_target.legs[2].joints[i].rad=static_cast<float>(lb_joint_target[i]);
        joints_target.legs[3].joints[i].rad=static_cast<float>(rb_joint_target[i]);
    }
    robot->legs_target_pub->publish(joints_target);

    if (robot->move_cmd.step_mode == 1) // 如果希望跳转到STOP状态（VMC站立），那么跳转
        return "stop";
    return "idel";
}
