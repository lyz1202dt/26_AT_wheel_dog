#ifndef JUMP_STEP_HPP
#define JUMP_STEP_HPP

#include "core/robot.hpp"
#include <Eigen/Dense>
#include "fsm/base_state.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tuple>

using Vector3D = Eigen::Vector3d;
using Vector2D = Eigen::Vector2d;

class JumpStepState : public BaseState<Robot> {
public:
    explicit JumpStepState(Robot* robot);
    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;

private:
    // 跳台阶状态机（完全按你需求）
    enum {
        STATE_GLIDE = 0,           // 正常滑行
        STATE_PRE_JUMP = 1,        // 检测到台阶，准备跳跃
        STATE_FRONT_LEGS_JUMP = 2, // 前双腿跳跃
        STATE_FRONT_ON_STEP = 3,   // 前轮已上台阶，后轮驱动滑行
        STATE_RESET = 4
    };

    // 平衡力矩计算（和walk风格完全一致）
    std::tuple<Vector3D, Vector3D, Vector3D, Vector3D>
    balance_force_calc(Robot* robot, double cur_roll, double cur_pitch);

    // 足端 局部坐标系 → 世界坐标系
    Vector3D local_to_world(Robot* robot, const Vector3D& local_pos);

    // 参数服务器
    double jump_height;
    double jump_forward;
    double jump_duration;
    double rear_slide_vel;
    double foot_obstacle_threshold;
    double jump_cooldown_time;

    // 状态
    int req_state;
    int current_state;

    // 时间
    rclcpp::Time jump_start_time;
    rclcpp::Time last_jump_finish_time;

    // 轨迹规划
    LegStep lf_leg_step;
    LegStep rf_leg_step;

    // 足端力滤波
    Vector3D lf_cart_force;
    Vector3D rf_cart_force;
    Vector3D lb_cart_force;
    Vector3D rb_cart_force;

    // 速度
    double current_body_vel;
    double exp_vel_kp;
};

#endif
