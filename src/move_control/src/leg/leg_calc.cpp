#include "leg/leg_calc.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/logger.hpp>

using namespace std::chrono_literals;

inline double smooth_clamp(double val, double min_val, double max_val, double alpha=0.1) {
    if(val < min_val) return (1.0 - alpha) * val + alpha * min_val;
    else if(val > max_val) return (1.0 - alpha) * val + alpha * max_val;
    else return val;
}

LegCalc::LegCalc(KDL::Chain& chain)
    : chain(chain)
    , fk_solver(chain)
    , jacobain_solver(chain)
    , jdot_solver(chain)
    , vel_solver(chain)
    , ik_pos_solver(chain, Eigen::Vector<double, 6>(1.0, 1.0, 1.0, 0.0, 0.0, 0.0), 1e-6, 150, 1e-10)
    , dynamin_solver(chain, KDL::Vector(0, 0, -9.81)) {

    _temp_joint3_array.resize(3); // 提前resize需要用到的KDL::JntArray防止运行时频繁申请/释放内存
    _temp2_joint3_array.resize(3);
    last_exp_joint_pos.resize(3);
    temp_jacobain.resize(3);
    _temp_joint3_vel_array.resize(3);

    C.resize(3);
    G.resize(3);
    M.resize(3);

    last_exp_joint_pos(0) = 0.0;
    last_exp_joint_pos(1) = 0.0;
    last_exp_joint_pos(2) = 0.0;

    set_joint_pd(0,3.0,0.17);   //设置默认参数
    set_joint_pd(1,2.8,0.14);
    set_joint_pd(2,2.8,0.11);

    set_joint_pd(0,50.0,3.0);   //设置默认参数
    set_joint_pd(1,50.0,3.0);
    set_joint_pd(2,50.0,3.0);

    set_joint_pd(3,0.0,0.5);
}

LegCalc::~LegCalc() {}

Eigen::Vector3d LegCalc::joint_pos(const Eigen::Vector3d &foot_pos,int  *result,const Eigen::Vector3d &cur_joint_pos)
{
    KDL::Frame frame;
    Eigen::Vector3d temp = foot_pos + pos_offset;
    frame.p.x(temp[0]);
    frame.p.y(temp[1]);
    frame.p.z(temp[2]);
    frame.M = KDL::Rotation::Identity();

    _temp_joint3_array(0)=cur_joint_pos[0];
    _temp_joint3_array(1)=cur_joint_pos[1];
    _temp_joint3_array(2)=cur_joint_pos[2];

    *result = ik_pos_solver.CartToJnt(last_exp_joint_pos, frame, _temp_joint3_array);
    if (*result == 0)                                                                      // 缓存本次计算结果,方便下一次迭代
        last_exp_joint_pos = _temp_joint3_array;
    return {_temp_joint3_array(0), _temp_joint3_array(1), _temp_joint3_array(2)};
}

Eigen::Matrix<double, 3, 3> LegCalc::get_3x3_jacobian_(const KDL::Jacobian& full_jacobian) // 只关心前三行的映射关系
{
    Eigen::Matrix<double, 3, 3> jacobian_3x3;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            jacobian_3x3(i, j) = full_jacobian(i, j);
        }
    }
    return jacobian_3x3;
}


// 支持切换限制 IK
Eigen::Vector3d LegCalc::joint_pos(const Eigen::Vector3d& foot_pos, int* result) {
    KDL::Frame frame;
    Eigen::Vector3d temp = foot_pos + pos_offset;
    frame.p.x(temp[0]);
    frame.p.y(temp[1]);
    frame.p.z(temp[2]);
    frame.M = KDL::Rotation::Identity();

    *result = ik_pos_solver.CartToJnt(last_exp_joint_pos, frame, _temp_joint3_array);
    if (*result == 0)                                                                      // 缓存本次计算结果,方便下一次迭代
        last_exp_joint_pos = _temp_joint3_array;
    return {_temp_joint3_array(0), _temp_joint3_array(1), _temp_joint3_array(2)};
}

Eigen::Vector3d LegCalc::joint_vel(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& foot_vel) {
    _temp_joint3_array(0) = joint_rad[0];
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];
    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    Eigen::Matrix<double, 3, 3> jacobian = get_3x3_jacobian_(temp_jacobain);
    return jacobian.inverse() * foot_vel;
}


/**
    @brief 计算关节角加速度
    @param joint_rad 关节角度向量
    @param joint_vel 关节角速度
    @param foot_acc  期望的足端加速度
    @return 关节角加速度向量
 */
Eigen::Vector3d LegCalc::joint_acc(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_vel, Eigen::Vector3d foot_acc) {
    _temp_joint3_array.data          = joint_rad;
    _temp2_joint3_array.data         = joint_vel;
    _temp_joint3_vel_array.q.data    = joint_rad;
    _temp_joint3_vel_array.qdot.data = joint_vel;

    // 计算雅可比矩阵J
    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    jdot_solver.JntToJacDot(_temp_joint3_vel_array, _temp_jdot_qd);

    Eigen::Matrix3d Jac = get_3x3_jacobian_(temp_jacobain);
    Vector3D jdot_dq_eigen;
    for (int i = 0; i < 3; i++) {
        jdot_dq_eigen[i] = _temp_jdot_qd(i);
    }
    return Jac.completeOrthogonalDecomposition().solve(foot_acc - jdot_dq_eigen);
}

Eigen::Vector3d
    LegCalc::joint_torque_dynamic(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_omega, const Eigen::Vector3d& foot_acc) {
    _temp_joint3_array(0)  = joint_rad[0];
    _temp_joint3_array(1)  = joint_rad[1];
    _temp_joint3_array(2)  = joint_rad[2];
    _temp2_joint3_array(0) = joint_omega[0];
    _temp2_joint3_array(1) = joint_omega[1];
    _temp2_joint3_array(2)  = joint_omega[2];
    dynamin_solver.JntToGravity(_temp_joint3_array, G);
    dynamin_solver.JntToCoriolis(_temp_joint3_array, _temp2_joint3_array, C);
    dynamin_solver.JntToMass(_temp_joint3_array, M);

    // 6. 转换 KDL 输出到 Eigen，方便矩阵运算
    Eigen::Matrix<double, 3, 3> M_;
    Eigen::Matrix<double, 3, 1> C_, G_;

    for (int i = 0; i < 3; ++i) {
        C_(i) = C(i);
        G_(i) = G(i);
        for (int j = 0; j < 3; ++j) {
            M_(i, j) = M(i, j);
        }
    }
    // 7. 计算前馈力矩 tau
    return M_ * joint_acc(joint_rad, joint_omega, foot_acc) + C_ + G_;
}

/**
    @brief 足端期望力->计算关节力矩
    @param joint_rad 关节角度
    @param joint_force 关节末端期望力
    @return 关节空间下的力矩
 */
Eigen::Vector3d LegCalc::joint_torque_foot_force(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& foot_force) {
    _temp_joint3_array(0) = joint_rad[0];
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];
    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    Eigen::Matrix<double, 3, 3> jacobian = get_3x3_jacobian_(temp_jacobain);
    Eigen::Vector3d torque(foot_force(0), foot_force(1), foot_force(2));
    return jacobian.transpose() * torque;
}

/**
    @brief 计算足端受力
    @param joint_rad 当前关节角度
    @param joint_torque 总力矩减去克服重力/科氏力/惯性力剩下的力矩（需要在外部计算）
    @return 笛卡尔坐标系下的足端受力
 */
Eigen::Vector3d
    LegCalc::foot_force(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_torque, const Eigen::Vector3d& forward_torque) {
    _temp_joint3_array(0) = joint_rad[0];
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];

    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    auto jacobian = get_3x3_jacobian_(temp_jacobain);

    return jacobian.transpose().inverse() * (joint_torque - forward_torque);
}

/**
    @brief 计算足端速度
    @param joint_rad 当前关节角度
    @param joint_omega 当前关节角速度
    @return 当前足端速度
 */
Eigen::Vector3d LegCalc::foot_vel(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_omega) {
    _temp_joint3_array(0) = joint_rad[0];
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];

    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);

    auto jacobian = get_3x3_jacobian_(temp_jacobain); // 提取雅可比矩阵中与位置相关的部分
    Eigen::Vector3d dq(joint_omega(0), joint_omega(1), joint_omega(2));
    return jacobian * dq;
}

/**
    @brief 计算足端位置
    @param joint_rad 关节角度向量
    @return 当前足端位置
 */
Eigen::Vector3d LegCalc::foot_pos(const Eigen::Vector3d& joint_rad) {
    KDL::Frame frame;
    _temp_joint3_array(0) = joint_rad[0]; // 避免运行时动态分配内存，提高效率
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];

    int fk_result = fk_solver.JntToCart(_temp_joint3_array, frame);

                                          // 添加调试：检查 FK 计算结果
    std::cout << "[FK DEBUG] Joint angles: [" << joint_rad[0] << ", " << joint_rad[1] << ", " << joint_rad[2] << "]" << std::endl;
    std::cout << "[FK DEBUG] FK result code: " << fk_result << std::endl;
    std::cout << "[FK DEBUG] Frame position: [" << frame.p.x() << ", " << frame.p.y() << ", " << frame.p.z() << "]" << std::endl;
    std::cout << "[FK DEBUG] pos_offset: [" << pos_offset[0] << ", " << pos_offset[1] << ", " << pos_offset[2] << "]" << std::endl;

    Eigen::Vector3d temp;
    temp[0] = frame.p.x();
    temp[1] = frame.p.y();
    temp[2] = frame.p.z();

    return temp - pos_offset; // temp是在机器人坐标系下的足端位置，要转换成支撑相中型点的坐标输出
}


robot_interfaces::msg::LegTarget LegCalc::signal_leg_calc(
    const Vector3D& exp_cart_pos, const Vector3D& exp_cart_vel, const Vector3D& exp_cart_acc, const Vector3D& exp_cart_force,
    Vector3D* torque,const double wheel_vel,const double wheel_force) {
    int result;
    auto joint_rad    = joint_pos(exp_cart_pos, &result); // 一般这个位置不可能会迭代失败，所以不再对result进行处理
    auto joint_omega  = joint_vel(joint_rad, exp_cart_vel);
    auto joint_torque = joint_torque_foot_force(joint_rad, exp_cart_force);
    joint_torque += joint_torque_dynamic(joint_rad, joint_omega, exp_cart_acc);

    robot_interfaces::msg::LegTarget leg;
    leg.joints[0].rad    = static_cast<float>(joint_rad[0]);
    leg.joints[0].omega  = static_cast<float>(joint_omega[0]);
    leg.joints[0].torque = static_cast<float>(joint_torque[0]);
    leg.joints[0].kp     = static_cast<float>(kp[0]);
    leg.joints[0].kd     = static_cast<float>(kd[0]);

    leg.joints[1].rad    = static_cast<float>(joint_rad[1]);
    leg.joints[1].omega  = static_cast<float>(joint_omega[1]);
    leg.joints[1].torque = static_cast<float>(joint_torque[1]);
    leg.joints[1].kp     = static_cast<float>(kp[1]);
    leg.joints[1].kd     = static_cast<float>(kd[1]);

    leg.joints[2].rad    = static_cast<float>(joint_rad[2]);
    leg.joints[2].omega  = static_cast<float>(joint_omega[2]);
    leg.joints[2].torque = static_cast<float>(joint_torque[2]);
    leg.joints[2].kp     = static_cast<float>(kp[2]);
    leg.joints[2].kd     = static_cast<float>(kd[2]);

    leg.wheel.omega  = static_cast<float>(wheel_vel / wheel_radius);
    leg.wheel.torque = static_cast<float>(wheel_force * wheel_radius);
    leg.wheel.kd     = static_cast<float>(wheel_kd);
    *torque          = joint_torque;

    return leg;
}

robot_interfaces::msg::LegTarget LegCalc::signal_leg_torque_calc(const Vector3D& cur_joint_pos, const Vector3D& exp_foot_force, const Vector3D& foot_vel,const Vector3D& foot_acc,double wheel_force) {
    Vector3D joint_torque;
    joint_torque = joint_torque_foot_force(cur_joint_pos, exp_foot_force);
    joint_torque += joint_torque_dynamic(cur_joint_pos, joint_vel(cur_joint_pos, foot_vel), foot_acc);

    robot_interfaces::msg::LegTarget leg;
    leg.joints[0].torque = static_cast<float>(joint_torque[0]);
    leg.joints[1].torque = static_cast<float>(joint_torque[1]);
    leg.joints[2].torque = static_cast<float>(joint_torque[2]);
    leg.wheel.torque = static_cast<float>(wheel_force * wheel_radius);

    return leg;
}

/**
    @brief 设置关节kp和kd
    @param index 关节索引
    @param kp 关节kp
    @param kd 关节kd
    @return none
    @note index=0/1/2时，对应前三个关节。index=3时，对应轮子，此时只有kd参数有效，因为轮子没有kp
 */
void LegCalc::set_joint_pd(int index,double kp,double kd)
{
    if(index<3)
    {
        this->kp[index]=kp;
        this->kd[index]=kd;
    }
    else
        wheel_kd=kd;
}

/**
    @brief 得到关节kp和kd
    @param index 关节索引
    @param kp 关节kp
    @param kd 关节kd
    @return none
    @note index=0/1/2时，对应前三个关节。index=3时，对应轮子，此时只有kd参数有效，因为轮子没有kp
 */
void LegCalc::get_joint_pd(int index,double &kp,double &kd)
{
    if(index<3)
    {
        kp=this->kp[index];
        kd=this->kd[index];
    }
    else
        kd=wheel_kd;
}
