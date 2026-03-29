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
    // // 机身偏移（为后腿上台阶做准备，向左/右移动质心）
    // void bodyShiftForRearLegClimb(
    //     Robot* robot,
    //     int& shift_step,
    //     rclcpp::Time& shift_start_time,
    //     double shift_y = 0.06  // 偏移量 默认 6cm
    // );

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
    if (!F) return;  // 空指针保护

    Eigen::Matrix<double, 3, 4> A;
    A.setZero();
    Eigen::Vector3d b;
    b << mass * 9.8, 0.0, 0.0;
    auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);
    auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
    auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
    auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
    
    if (lf_contact) A(0,0) = 1;
    if (rf_contact) A(0,1) = 1;
    if (lb_contact) A(0,2) = 1;
    if (rb_contact) A(0,3) = 1;

    if (lf_contact) {
        A(1,0) = (lf_pos.y() - mass_center_pos.y());
        A(2,0) = -(lf_pos.x() - mass_center_pos.x());
    }
    if (rf_contact) {
        A(1,1) = (rf_pos.y() - mass_center_pos.y());
        A(2,1) = -(rf_pos.x() - mass_center_pos.x());
    }
    if (lb_contact) {
        A(1,2) = (lb_pos.y() - mass_center_pos.y());
        A(2,2) = -(lb_pos.x() - mass_center_pos.x());
    }
    if (rb_contact) {
        A(1,3) = (rb_pos.y() - mass_center_pos.y());
        A(2,3) = -(rb_pos.x() - mass_center_pos.x());
    }

    Eigen::Matrix3d ATA = A * A.transpose();
    Eigen::Matrix3d reg = 1e-6 * Eigen::Matrix3d::Identity();

    Eigen::Vector4d forces = A.transpose() * (ATA + reg).inverse() * b;

    for(int i = 0; i < 4; i++)
        forces(i) = std::max(0.0, forces(i));

    F->lf_force = Vector3D(0,0,-forces(0));
    F->rf_force = Vector3D(0,0,-forces(1));
    F->lb_force = Vector3D(0,0,-forces(2));
    F->rb_force = Vector3D(0,0,-forces(3));
}

#endif
