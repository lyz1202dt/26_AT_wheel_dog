#ifndef JUMP_STEPS_HPP
#define JUMP_STEPS_HPP

#include "core/robot.hpp"
#include <Eigen/Dense>
#include "fsm/base_state.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class JumpStepstate : public BaseState<Robot> {
public:
    explicit JumpStepstate(Robot* robot);
    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;

private:
    // 状态机 完全沿用你原来的风格
    enum {
        STATE_GLIDE = 0,          // 正常滑行
        STATE_PRE_JUMP = 1,       // 准备跳跃
        STATE_FRONT_JUMP = 2,     // 前双腿跳跃
        STATE_FRONT_ON_STEP = 3,  // 前轮已上台阶，后轮推进
        STATE_RESET = 4           // 复位
    };

    // 平衡力计算（和你原来完全一样）
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
    balance_force_calc(Robot* robot, double cur_roll, double cur_pitch);

    // 机身坐标 → 世界坐标（你要求的核心功能）
    Eigen::Vector3d local_to_world(Robot* robot, const Eigen::Vector3d& local_pos);

    // 参数
    double jump_step_finished_idel_time;
    int req_state;
    int current_state;
    int last_state;

    // 时间
    rclcpp::Time jump_start_time;
    rclcpp::Time last_jump_end_time;

    // 腿轨迹
    LegStep lf_leg_step;
    LegStep rf_leg_step;

    // 滤波力
    Eigen::Vector3d lf_cart_force;
    Eigen::Vector3d rf_cart_force;
    Eigen::Vector3d lb_cart_force;
    Eigen::Vector3d rb_cart_force;

    // 速度控制
    double current_body_vel;
    double current_exp_vel;
    const double exp_vel_kp = 2.0;

    // 触发阈值
    const double foot_obstruct_gate = 18.0;

    // 跳台阶参数
    const double jump_height    = 0.12;   // 跳跃高度
    const double jump_forward   = 0.04;   // 跳跃前向距离
    const double jump_duration  = 0.25;   // 跳跃时间
    const double rear_slide_vel = 0.12;   // 后轮缓慢滑动速度
};

#endif
