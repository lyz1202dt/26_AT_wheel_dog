#pragma once

#include "fsm/base_state.hpp"
#include "leg/step.hpp"
#include <rclcpp/rclcpp.hpp>
#include "states/cross_wall.hpp"


// 前向声明
class Robot;

    // typedef struct {
    //     double a;
    //     double b;
    //     double c;
    //     double d;
    //     double e;
    //     double f;
    // } QuinticLineParam_t;

    // typedef struct {
    //     QuinticLineParam_t x;
    //     QuinticLineParam_t y;
    //     QuinticLineParam_t z;
    //     double time;
    // } StepTrajectory_t;

class  Amble_Step{

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

class AmbleState : public BaseState<Robot> {
public:
    AmbleState(Robot* robot);

    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;

private:
    Vector3D get_next_available_pos(Robot*robot, Vector3D leg_offset,Vector3D current_pos);

    Vector2D mass_center_pos;
    double mass;
    Vector3D lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos;
    Vector3D lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force;
    Vector3D lf_foot_exp_vel, rf_foot_exp_vel, lb_foot_exp_vel, rb_foot_exp_vel;
    Vector3D lf_foot_exp_acc, rf_foot_exp_acc, lb_foot_exp_acc, rb_foot_exp_acc;
    double lf_wheel_vel, rf_wheel_vel, lb_wheel_vel, rb_wheel_vel;
    double lf_wheel_force, rf_wheel_force, lb_wheel_force, rb_wheel_force;
    Vector3D last_pos_1, last_pos_2;
    int step_state;
    double step_time{2.0};
    rclcpp::Time start_time;
    LegStep lf_leg_step, rf_leg_step, lb_leg_step, rb_leg_step;
    Amble_Step lf_step, rf_step, lb_step, rb_step;
    bool lf_leg_trajectory_updated{false}, rf_leg_trajectory_updated{false}, lb_leg_trajectory_updated{false},
        rb_leg_trajectory_updated{false};
    double  step_dy{0.08};
};
