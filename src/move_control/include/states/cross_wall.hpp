#pragma once
#include <fstream>
#include <sstream>
#include <iomanip>
#include "fsm/base_state.hpp"
#include "leg/step.hpp"
#include <Eigen/Dense>
#include <leg/leg_calc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tuple>

class Robot;

class Cross_WallState : public BaseState<Robot> {
public:
    Cross_WallState(Robot* robot);
    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;
    ~Cross_WallState();

private:
    int cross_wall_stage{-1};
    rclcpp::Time cross_wall_stage_time;
    /**************************使用笛卡尔坐标系***********************/
    Vector3D wall_lf_foot_pos{0,0,0}, wall_rf_foot_pos{0,0,0}, wall_lb_foot_pos{0,0,0}, wall_rb_foot_pos{0,0,0};
    Vector3D lf_foot_exp_pos{0,0,0}, rf_foot_exp_pos{0,0,0}, lb_foot_exp_pos{0,0,0}, rb_foot_exp_pos{0,0,0};
    Vector3D lf_foot_exp_force{0,0,0}, rf_foot_exp_force{0,0,0}, lb_foot_exp_force{0,0,0}, rb_foot_exp_force{0,0,0};
    Vector3D lf_foot_exp_vel{0,0,0}, rf_foot_exp_vel{0,0,0}, lb_foot_exp_vel{0,0,0}, rb_foot_exp_vel{0,0,0};
    Vector3D lf_foot_exp_acc{0,0,0}, rf_foot_exp_acc{0,0,0}, lb_foot_exp_acc{0,0,0}, rb_foot_exp_acc{0,0,0};
    Vector3D lf_forward_torque{0,0,0}, rf_forward_torque{0,0,0}, lb_forward_torque{0,0,0}, rb_forward_torque{0,0,0};

    /********************使用关节角度**************************/
    //执行期望位置
    Vector3D lf_joint_exp_pos_{0,0,0},rf_joint_exp_pos_{0,0,0},
             lb_joint_exp_pos_{0,0,0},rb_joint_exp_pos_{0,0,0};
    //执行期望速度
    Vector3D lf_joint_exp_vel_{0,0,0},rf_joint_exp_vel_{0,0,0},
             lb_joint_exp_vel_{0,0,0},rb_joint_exp_vel_{0,0,0};
    //执行期望加速度
    Vector3D lf_joint_exp_acc_{0,0,0},rf_joint_exp_acc_{0,0,0},
             lb_joint_exp_acc_{0,0,0},rb_joint_exp_acc_{0,0,0};
    //规划轨迹的最终位置
    Vector3D lf_joint_target_pos{0,0,0},rf_joint_target_pos{0,0,0},
             lb_joint_target_pos{0,0,0},rb_joint_target_pos{0,0,0};
    //规划轨迹的初始位置
    Vector3D lf_joint_init_pos{0,0,0},rf_joint_init_pos{0,0,0},
             lb_joint_init_pos{0,0,0},rb_joint_init_pos{0,0,0};


    double lf_wheel_vel{0.0},rf_wheel_vel{0.0},lb_wheel_vel{0.0},rb_wheel_vel{0.0};
    double lf_wheel_force{0.0},rf_wheel_force{0.0},lb_wheel_force{0.0},rb_wheel_force{0.0};
    bool enable_posture_safe{true};
    
    LegStep lf_leg_step, rf_leg_step, lb_leg_step, rb_leg_step;

    double cross_x_lf{0.0},cross_y_lf{0.0},cross_z_lf{0.0};
    double cross_x_rf{0.0},cross_y_rf{0.0},cross_z_rf{0.0};
    double cross_x_lb{0.0},cross_y_lb{0.0},cross_z_lb{0.0};
    double cross_x_rb{0.0},cross_y_rb{0.0},cross_z_rb{0.0};
    double time_s{1.0};
    bool change_flag{false};
    bool allow_vel{true};

    float k_F{1.0f};










    double y_target{0.08f};
    double mass;
    Vector2D mass_center_pos;

    Vector3D rf_init_pos;
    Vector3D lb_init_pos;
    Vector3D rb_init_pos;

    bool foot_init = false;





//     // CSV 记录相关
//     std::ofstream csv_file_;
//     std::mutex csv_mutex_;                // 保护文件写入（如果多线程）
//     bool csv_enabled_ = true;              // 可配置是否启用
//     std::vector<double> csv_buffer_;       // 可选：用于批量写入
//     static constexpr size_t CSV_BUFFER_SIZE = 100; // 每攒够100行写入一次
// // 在类定义中添加
// std::vector<std::string> csv_lines_;
// static constexpr size_t CSV_FLUSH_THRESHOLD = 100;



};


#include <vector>
#include <cmath>

namespace trajectory {

template<typename Vec>
class Bezier {
public:
    Bezier() = default;

    Bezier(const std::vector<Vec>& control_points)
        : control_points_(control_points) {}

    void setControlPoints(const std::vector<Vec>& control_points) {
        control_points_ = control_points;
    }

    const std::vector<Vec>& controlPoints() const {
        return control_points_;
    }

    Vec evaluate(double t) const {
        std::vector<Vec> tmp = control_points_;

        int n = tmp.size();

        for (int k = 1; k < n; ++k) {
            for (int i = 0; i < n - k; ++i) {
                tmp[i] = tmp[i] * (1 - t) + tmp[i + 1] * t;
            }
        }

        return tmp[0];
    }

    Vec velocity(double t) const {
        int n = control_points_.size() - 1;

        std::vector<Vec> d_points;

        for (int i = 0; i < n; ++i) {
            d_points.push_back(
                (control_points_[i + 1] - control_points_[i]) * n
            );
        }

        Bezier<Vec> d_curve(d_points);
        return d_curve.evaluate(t);
    }

    Vec acceleration(double t) const {
        int n = control_points_.size() - 1;

        std::vector<Vec> dd_points;

        for (int i = 0; i < n - 1; ++i) {
            dd_points.push_back(
                (control_points_[i + 2]
                 - control_points_[i + 1] * 2
                 + control_points_[i]) * (n * (n - 1))
            );
        }

        Bezier<Vec> dd_curve(dd_points);
        return dd_curve.evaluate(t);
    }

private:
    std::vector<Vec> control_points_;
};

}