#pragma once

#include "fsm/base_state.hpp"
#include "leg/step.hpp"
#include <Eigen/Dense>
#include <leg/leg_calc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tuple>

class Robot;


    typedef struct {
        double a;
        double b;
        double c;
        double d;
        double e;
        double f;
    } QuinticLineParam_t;

    typedef struct {
        QuinticLineParam_t x;
        QuinticLineParam_t y;
        QuinticLineParam_t z;
        double time;
    } StepTrajectory_t;

class Cross_Step {

public:

    void update_support_trajectory(const Vector3D& cur_pos, const Vector3D final_pos, double time);
    std::tuple<Vector3D, Vector3D, Vector3D> get_target(double time,bool &success);

private:
    StepTrajectory_t traj;
    double T;
    static void set_quintic(QuinticLineParam_t& seg,
                        double p0, double v0, double a0,
                        double pT, double vT, double aT,
                        double T)
    {
        double T2 = T * T;
        double T3 = T2 * T;
        double T4 = T3 * T;
        double T5 = T4 * T;

        seg.a = p0;
        seg.b = v0;
        seg.c = 0.5 * a0;

        seg.d = (10 * (pT - p0) - (6 * v0 + 4 * vT) * T - (1.5 * a0 - 0.5 * aT) * T2) / T3;
        seg.e = (-15 * (pT - p0) + (8 * v0 + 7 * vT) * T + (1.5 * a0 - aT) * T2) / T4;
        seg.f = (6 * (pT - p0) - (3 * v0 + 3 * vT) * T - (0.5 * a0 - 0.5 * aT) * T2) / T5;
    }

    static inline double get_quintic_value(const QuinticLineParam_t& line, double t)
    {
        return line.a
            + line.b * t
            + line.c * t * t
            + line.d * t * t * t
            + line.e * t * t * t * t
            + line.f * t * t * t * t * t;
    }

    static inline double get_quintic_dt(const QuinticLineParam_t& line, double t)
    {
        return line.b
            + 2.0 * line.c * t
            + 3.0 * line.d * t * t
            + 4.0 * line.e * t * t * t
            + 5.0 * line.f * t * t * t * t;
    }

    static inline double get_quintic_dtdt(const QuinticLineParam_t& line, double t)
    {
        return 2.0 * line.c
            + 6.0 * line.d * t
            + 12.0 * line.e * t * t
            + 20.0 * line.f * t * t * t;
    }

};


class Cross_WallState : public BaseState<Robot> {
public:
    Cross_WallState(Robot* robot);
    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;


private:
    //使用笛卡尔坐标系
    int cross_wall_stage{-1};
    rclcpp::Time cross_wall_stage_time;
    Vector3D wall_lf_foot_pos{0,0,0}, wall_rf_foot_pos{0,0,0}, wall_lb_foot_pos{0,0,0}, wall_rb_foot_pos{0,0,0};
    Vector3D lf_foot_exp_pos{0,0,0}, rf_foot_exp_pos{0,0,0}, lb_foot_exp_pos{0,0,0}, rb_foot_exp_pos{0,0,0};
    Vector3D lf_foot_exp_force{0,0,0}, rf_foot_exp_force{0,0,0}, lb_foot_exp_force{0,0,0}, rb_foot_exp_force{0,0,0};
    Vector3D lf_foot_exp_vel{0,0,0}, rf_foot_exp_vel{0,0,0}, lb_foot_exp_vel{0,0,0}, rb_foot_exp_vel{0,0,0};
    Vector3D lf_foot_exp_acc{0,0,0}, rf_foot_exp_acc{0,0,0}, lb_foot_exp_acc{0,0,0}, rb_foot_exp_acc{0,0,0};
    Vector3D lf_forward_torque{0,0,0}, rf_forward_torque{0,0,0}, lb_forward_torque{0,0,0}, rb_forward_torque{0,0,0};

    //使用关节角度
    Vector3D lf_joint_exp_pos_{0,0,0},rf_joint_exp_pos_{0,0,0},
             lb_joint_exp_pos_{0,0,0},rb_joint_exp_pos_{0,0,0};
    Vector3D lf_joint_omega{0,0,0}, rf_joint_omega{0,0,0},
             lb_joint_omega{0,0,0},rb_joint_omega{0,0,0};
    Vector3D lf_joint_torque{0,0,0},rf_joint_torque{0,0,0},
             lb_joint_torque{0,0,0},rb_joint_torque{0,0,0};

    double lf_wheel_vel{0.0},rf_wheel_vel{0.0},lb_wheel_vel{0.0},rb_wheel_vel{0.0};
    double lf_wheel_force{0.0},rf_wheel_force{0.0},lb_wheel_force{0.0},rb_wheel_force{0.0};

    bool stopping = false;
    double stop_t = 0.0;
    double stop_T = 0.3;   // 建议 0.3~0.6

    double lf_vel_start, rf_vel_start, lb_vel_start, rb_vel_start;
    double lf_force_start, rf_force_start, lb_force_start, rb_force_start;

    bool enable_posture_safe{true};
    
    LegStep lf_leg_step, rf_leg_step, lb_leg_step, rb_leg_step;
    Cross_Step lf_step, rf_step, lb_step, rb_step;
    double cross_x_lf{0.0},cross_y_lf{0.0},cross_z_lf{0.0};
    double cross_x_rf{0.0},cross_y_rf{0.0},cross_z_rf{0.0};
    double cross_x_lb{0.0},cross_y_lb{0.0},cross_z_lb{0.0};
    double cross_x_rb{0.0},cross_y_rb{0.0},cross_z_rb{0.0};
    double time_s{1.0};
    bool change_flag{true};
    bool allow_vel{true};

    float k_F{1.0f};

    //力变量
    Vector2D mass_center_pos;
    double mass;

};





