#ifndef NEW_CROSS_STEP_HPP
#define NEW_CROSS_STEP_HPP

#include "core/robot.hpp"
#include "leg/step.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

typedef struct {
    Vector3D lf_force;
    Vector3D rf_force;
    Vector3D lb_force;
    Vector3D rb_force;
} foot_f;
  
class JumpStepState : public BaseState<Robot>
{
public:
    explicit JumpStepState(Robot* robot);
    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;


private:
       robot_interfaces::msg::RobotTarget joints_target;

    foot_f F;
    int jump_stage = 0;
    rclcpp::Time jump_stage_time;

    LegStep lf_leg_step;
    LegStep rf_leg_step;
    LegStep lb_leg_step;
    LegStep rb_leg_step;

    Vector2D mass_center_pos = Vector2D::Zero();
    double mass = 0;

    double lf_wheel_vel    = 0.0;
    double rf_wheel_vel    = 0.0;
    double lb_wheel_vel    = 0.0;
    double rb_wheel_vel    = 0.0;

    double lf_wheel_force  = 0.0;
    double rf_wheel_force  = 0.0;
    double lb_wheel_force  = 0.0;
    double rb_wheel_force  = 0.0;

    Vector3D lf_foot_exp_pos= Vector3D::Zero(), rf_foot_exp_pos= Vector3D::Zero(), lb_foot_exp_pos= Vector3D::Zero(), rb_foot_exp_pos= Vector3D::Zero();
    Vector3D lf_foot_exp_acc= Vector3D::Zero(), rf_foot_exp_acc= Vector3D::Zero(), lb_foot_exp_acc= Vector3D::Zero(), rb_foot_exp_acc= Vector3D::Zero();
    Vector3D lf_foot_exp_vel= Vector3D::Zero(), rf_foot_exp_vel= Vector3D::Zero(), lb_foot_exp_vel= Vector3D::Zero(), rb_foot_exp_vel= Vector3D::Zero();

};

// 全局函数声明：足部力计算
static inline void computeFootForces(
    foot_f* F,
    Robot* robot,
    double mass,
    const Vector2D& mass_center_pos,
    const Vector3D& lf_foot_exp_pos,
    const Vector3D& rf_foot_exp_pos,
    const Vector3D& lb_foot_exp_pos,
    const Vector3D& rb_foot_exp_pos,
    bool lf_contact,
    bool rf_contact,
    bool lb_contact,
    bool rb_contact)
{
    if (!F) return;

    constexpr double g = 9.8;

    // ================== 足端位置 ==================
    auto lf_pos = (lf_foot_exp_pos + robot->lf_leg_calc->pos_offset).head(2);
    auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
    auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
    auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

    // ================== 构建 A ==================
    Eigen::Matrix<double, 3, 4> A;
    A.setZero();

    if (lf_contact) {
        A(0,0) = 1.0;
        A(1,0) = (lf_pos.y() - mass_center_pos.y());
        A(2,0) = -(lf_pos.x() - mass_center_pos.x());
    }
    if (rf_contact) {
        A(0,1) = 1.0;
        A(1,1) = (rf_pos.y() - mass_center_pos.y());
        A(2,1) = -(rf_pos.x() - mass_center_pos.x());
    }
    if (lb_contact) {
        A(0,2) = 1.0;
        A(1,2) = (lb_pos.y() - mass_center_pos.y());
        A(2,2) = -(lb_pos.x() - mass_center_pos.x());
    }
    if (rb_contact) {
        A(0,3) = 1.0;
        A(1,3) = (rb_pos.y() - mass_center_pos.y());
        A(2,3) = -(rb_pos.x() - mass_center_pos.x());
    }

    // ================== b ==================
    Eigen::Vector3d b;
    b << mass * g, 0.0, 0.0;

    // ================== 权重计算（核心🔥） ==================
    Eigen::Matrix4d W = Eigen::Matrix4d::Identity();

    auto compute_weight = [&](const Eigen::Vector2d& pos, bool contact)
    {
        if (!contact) return 1e6;  // 非接触腿 → 不参与

        double dist = (pos - mass_center_pos).norm();

        // 防止太小（数值稳定）
        return dist + 0.05;
    };

    W(0,0) = compute_weight(lf_pos, lf_contact);
    W(1,1) = compute_weight(rf_pos, rf_contact);
    W(2,2) = compute_weight(lb_pos, lb_contact);
    W(3,3) = compute_weight(rb_pos, rb_contact);

    Eigen::Matrix4d W_inv = W.inverse();

    // ================== 求解 ==================
    Eigen::Matrix3d M = A * W_inv * A.transpose();
    Eigen::Matrix3d reg = 1e-6 * Eigen::Matrix3d::Identity();

    Eigen::Vector4d forces =
        W_inv * A.transpose() * (M + reg).inverse() * b;

    // ================== 非负约束 ==================
    for (int i = 0; i < 4; i++)
        forces(i) = std::max(0.0, forces(i));

    // ================== 力归1 ==================
    double sum_force = 0.0;
    for (int i = 0; i < 4; i++)
        sum_force += forces(i);

    if (sum_force > 1e-6) {
        double scale = (mass * g) / sum_force;
        forces *= scale;
    }

    // ================== 写回 ==================
    F->lf_force = lf_contact ? Vector3D(0,0,-forces(0)) : Vector3D::Zero();
    F->rf_force = rf_contact ? Vector3D(0,0,-forces(1)) : Vector3D::Zero();
    F->lb_force = lb_contact ? Vector3D(0,0,-forces(2)) : Vector3D::Zero();
    F->rb_force = rb_contact ? Vector3D(0,0,-forces(3)) : Vector3D::Zero();
}

#endif
