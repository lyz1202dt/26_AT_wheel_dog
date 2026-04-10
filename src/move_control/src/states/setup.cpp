#include "states/setup.hpp"
#include "core/robot.hpp"
#include <rclcpp/logging.hpp>

SetupState::SetupState(Robot* robot)
    : BaseState<Robot>("setup") {
    (void)robot;
}

bool SetupState::enter(Robot* robot, const std::string& last_status) {
    (void)last_status;
    if(robot->arm_enable) {
        leg_exp_pos_arm_OC = 0.2;
        setup_stage = -1;
    }else {
        leg_exp_pos_arm_OC = 0.0;
        setup_stage = 0;
    }
    
    return true;
}

std::string SetupState::update(Robot* robot) {
    // TODO:更新状态
    if ((setup_stage == -1) && (!trajectory_calced)) {
        RCLCPP_INFO(robot->node_->get_logger(), "开始执行上电序列0");
        RCLCPP_INFO(robot->node_->get_logger(), "arm_enable = %d", robot->arm_enable);
        RCLCPP_INFO(robot->node_->get_logger(), "leg_exp_pos_arm_OC = %f", leg_exp_pos_arm_OC);
        trajectory_calced = true;
        setup_time        = robot->node_->get_clock()->now();
        lf_leg_step.update_support_trajectory(robot->lf_joint_pos, Vector3D(-leg_exp_pos_arm_OC, robot->lf_joint_pos[1], robot->lf_joint_pos[2]), 4.0);
        rf_leg_step.update_support_trajectory(robot->rf_joint_pos, Vector3D(leg_exp_pos_arm_OC, robot->rf_joint_pos[1], robot->rf_joint_pos[2]), 4.0);
        lb_leg_step.update_support_trajectory(robot->lb_joint_pos, Vector3D(robot->lb_joint_pos[0], robot->lb_joint_pos[1], robot->lb_joint_pos[2]), 4.0);
        rb_leg_step.update_support_trajectory(robot->rb_joint_pos, Vector3D(robot->rb_joint_pos[0], robot->rb_joint_pos[1], robot->rb_joint_pos[2]), 4.0);
    } // 需要规划轨迹1（戳地站起来)
    else if ((setup_stage == 0) && (!trajectory_calced)) {
        RCLCPP_INFO(robot->node_->get_logger(), "开始执行上电序列1");
        RCLCPP_INFO(robot->node_->get_logger(), "arm_enable = %d", robot->arm_enable);
        RCLCPP_INFO(robot->node_->get_logger(), "leg_exp_pos_arm_OC = %f", leg_exp_pos_arm_OC);
        trajectory_calced = true;
        setup_time        = robot->node_->get_clock()->now();
        lf_leg_step.update_support_trajectory(robot->lf_joint_pos, Vector3D(-leg_exp_pos_arm_OC, 3.14, robot->lf_joint_pos[2]), 4.0);
        rf_leg_step.update_support_trajectory(robot->rf_joint_pos, Vector3D(leg_exp_pos_arm_OC, -3.14, robot->rf_joint_pos[2]), 4.0);
        lb_leg_step.update_support_trajectory(robot->lb_joint_pos, Vector3D(0.0, 3.14, robot->lb_joint_pos[2]), 4.0);
        rb_leg_step.update_support_trajectory(robot->rb_joint_pos, Vector3D(0.0, -3.14, robot->rb_joint_pos[2]), 4.0);
    } // 需要规划轨迹1（戳地站起来）
    else if ((setup_stage == 1) && (!trajectory_calced)) {
        RCLCPP_INFO(robot->node_->get_logger(), "开始执行上电序列2");
        RCLCPP_INFO(robot->node_->get_logger(), "arm_enable = %d", robot->arm_enable);
        RCLCPP_INFO(robot->node_->get_logger(), "leg_exp_pos_arm_OC = %f", leg_exp_pos_arm_OC);
        trajectory_calced = true;
        setup_time        = robot->node_->get_clock()->now();
        lf_leg_step.update_support_trajectory(robot->lf_joint_pos, Vector3D(-0.2, 0.785, 0.0), 4.0);
        rf_leg_step.update_support_trajectory(robot->rf_joint_pos, Vector3D(0.2, -0.785, 0.0), 4.0);
        lb_leg_step.update_support_trajectory(robot->lb_joint_pos, Vector3D(0.0, 0.785, 0.0), 4.0);
        rb_leg_step.update_support_trajectory(robot->rb_joint_pos, Vector3D(0.0, -0.785, 0.0), 4.0);
    }
    else if ((setup_stage == 2) && (!trajectory_calced)) {
        RCLCPP_INFO(robot->node_->get_logger(), "开始执行上电序列2");
        RCLCPP_INFO(robot->node_->get_logger(), "arm_enable = %d", robot->arm_enable);
        RCLCPP_INFO(robot->node_->get_logger(), "leg_exp_pos_arm_OC = %f", leg_exp_pos_arm_OC);
        trajectory_calced = true;
        setup_time        = robot->node_->get_clock()->now();
        lf_leg_step.update_support_trajectory(robot->lf_joint_pos, Vector3D(0.0, 0.785, 0.0), 4.0);
        rf_leg_step.update_support_trajectory(robot->rf_joint_pos, Vector3D(0.0, -0.785, 0.0), 4.0);
        lb_leg_step.update_support_trajectory(robot->lb_joint_pos, Vector3D(0.0, 0.785, 0.0), 4.0);
        rb_leg_step.update_support_trajectory(robot->rb_joint_pos, Vector3D(0.0, -0.785, 0.0), 4.0);
    }

    // 从轨迹解析当前目标值
    bool success;
    auto now       = robot->node_->get_clock()->now();
    auto lf_target = lf_leg_step.get_target((now - setup_time).seconds(), success);
    auto rf_target = rf_leg_step.get_target((now - setup_time).seconds(), success);
    auto lb_target = lb_leg_step.get_target((now - setup_time).seconds(), success);
    auto rb_target = rb_leg_step.get_target((now - setup_time).seconds(), success);

    robot_interfaces::msg::RobotTarget joints_target;
    for (int i = 0; i < 3; i++) {
        joints_target.legs[0].joints[i].rad = static_cast<float>(std::get<0>(lf_target)[i]);
        joints_target.legs[1].joints[i].rad = static_cast<float>(std::get<0>(rf_target)[i]);
        joints_target.legs[2].joints[i].rad = static_cast<float>(std::get<0>(lb_target)[i]);
        joints_target.legs[3].joints[i].rad = static_cast<float>(std::get<0>(rb_target)[i]);

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
    robot->legs_target_pub->publish(joints_target);

    if ((now - setup_time).seconds() > 4.0) {
        trajectory_calced = false;
        if (setup_stage == 2) {
            RCLCPP_INFO(robot->node_->get_logger(), "上电完成,进入IDEL模式");
            return "idel";
        }
        setup_stage++;
    }
    return "setup";
}
