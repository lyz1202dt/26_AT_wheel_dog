#include "states/idle_inward.hpp"
#include "core/robot.hpp"
#include <Eigen/Dense>

idle_inward::idle_inward(Robot* robot)
    : BaseState<Robot>("idle_inward") {
    (void)robot;
}

bool idle_inward::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    return true;
}


std::string idle_inward::update(Robot* robot) {
    // TODO:更新状态
    auto lf_foot_exp_pos = robot->lf_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);
    auto rf_foot_exp_pos = robot->rf_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);
    auto lb_foot_exp_pos = robot->lb_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);
    auto rb_foot_exp_pos = robot->rb_leg_stop_pos = Vector3D(0.0, 0.0, 0.0);

    robot_interfaces::msg::RobotTarget joints_target;
    if ((!trajectory_calced)) {
        if(robot->driver_or_sim == driver)
            robot->node_->set_parameters({
                rclcpp::Parameter("lf_grivate", 34.0),
                rclcpp::Parameter("rf_grivate", 34.0),
                rclcpp::Parameter("lb_grivate", 33.0),
                rclcpp::Parameter("rb_grivate", 33.0)
            });
        else 
            robot->node_->set_parameters({
                    rclcpp::Parameter("lf_grivate", 25.0),
                    rclcpp::Parameter("rf_grivate", 25.0),
                    rclcpp::Parameter("lb_grivate", 24.0),
                    rclcpp::Parameter("rb_grivate", 24.0)
        
                });

        RCLCPP_INFO(robot->node_->get_logger(), "开始执行内八脚运动");

        trajectory_calced = true;
        setup_time        = robot->node_->get_clock()->now();
        lf_leg_step.update_support_trajectory(robot->lf_joint_pos, robot->lf_joint_pos, 3.0);
        rf_leg_step.update_support_trajectory(robot->rf_joint_pos, robot->rf_joint_pos, 3.0);
        lb_leg_step.update_support_trajectory(robot->lb_joint_pos, Vector3D(robot->lb_joint_pos(0), -0.79, 3.28), 3.0);
        rb_leg_step.update_support_trajectory(robot->rb_joint_pos, Vector3D(robot->rb_joint_pos(0), 0.79, -3.28), 3.0);
    }

    bool success;
    auto now       = robot->node_->get_clock()->now();
    
    lf_joint_target = lf_leg_step.get_target((now - setup_time).seconds(), success);
    rf_joint_target = rf_leg_step.get_target((now - setup_time).seconds(), success);
    lb_joint_target = lb_leg_step.get_target((now - setup_time).seconds(), success);
    rb_joint_target = rb_leg_step.get_target((now - setup_time).seconds(), success);

    static int cnt = 0;
        cnt++;
        if(cnt>=10)
        {
            cnt = 0;
            RCLCPP_ERROR(robot->node_->get_logger(),
                "\033[31mlf_joint_target = (%.2f, %.2f, %.2f),\n rf_joint_target = (%.2f, %.2f, %.2f),\n lb_joint_target = (%.2f, %.2f, %.2f),\n rb_joint_target = (%.2f, %.2f, %.2f)\033[0m",
                        std::get<0>(lf_joint_target)(0), std::get<0>(lf_joint_target)(1),std::get<0>(lf_joint_target)(2),
                    std::get<0>(rf_joint_target)(0), std::get<0>(rf_joint_target)(1),std::get<0>(rf_joint_target)(2),
                std::get<0>(lb_joint_target)(0), std::get<0>(lb_joint_target)(1),std::get<0>(lb_joint_target)(2),
            std::get<0>(rb_joint_target)(0), std::get<0>(rb_joint_target)(1),std::get<0>(rb_joint_target)(2));
        }

    for (int i = 0; i < 3; i++) {
        joints_target.legs[0].joints[i].rad = static_cast<float>(std::get<0>(lf_joint_target)[i]);
        joints_target.legs[1].joints[i].rad = static_cast<float>(std::get<0>(rf_joint_target)[i]);
        joints_target.legs[2].joints[i].rad = static_cast<float>(std::get<0>(lb_joint_target)[i]);
        joints_target.legs[3].joints[i].rad = static_cast<float>(std::get<0>(rb_joint_target)[i]);

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
    if ((now - setup_time).seconds() >= 3.01)
    {
        trajectory_calced = false;
        robot->lf_leg_calc->set_init_joint_pos(robot->lf_joint_pos);
        robot->rf_leg_calc->set_init_joint_pos(robot->rf_joint_pos);
        robot->lb_leg_calc->set_init_joint_pos(robot->lb_joint_pos);
        robot->rb_leg_calc->set_init_joint_pos(robot->rb_joint_pos);
        return "stop";
    }    
    return "idle_inward";
}