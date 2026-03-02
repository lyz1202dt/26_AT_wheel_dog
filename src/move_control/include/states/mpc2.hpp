#include "core/robot.hpp"
#include "fsm/base_state.hpp"
#include <OsqpEigen/OsqpEigen.h>

class Robot;

class MPC2State : public BaseState<Robot> {
public:
    MPC2State(Robot* robot);

    bool enter(Robot* robot, const std::string& last_status) override;
    std::string update(Robot* robot) override;

private:
    constexpr static int n = 8;          // MPC预测步数

    OsqpEigen::Solver solver;            // QP求解器

    Eigen::Vector<double, 12> x0;        // 当前状态

    Eigen::Matrix<double, 12, 12> A;     // 系统状态矩阵
    Eigen::Matrix<double, 12, 12> B;     // 系统输入矩阵 (12x12 for 4 legs x 3 forces)
    Eigen::Matrix<double, 12, 1> g;      // 重力项向量 (离散化后)对系统的影响

    Eigen::DiagonalMatrix<double, 12> Q; // 状态惩罚矩阵
    Eigen::DiagonalMatrix<double, 12> R; // 控制输入惩罚矩阵

    Eigen::Matrix<double, 12 * n, 12> A_qp;
    Eigen::Matrix<double, 12 * n, 12> B_qp;
    Eigen::Matrix<double, 12 * n, 12> C_qp;

    Eigen::Matrix<double, 12 * n, 12 * n> L;
    Eigen::Matrix<double, 12 * n, 12 * n> K;


    Eigen::SparseMatrix<double> H;       // 海森堡矩阵
    Eigen::Matrix<double, 12 * n, 1> G;  // 梯度向量

    // 约束相关
    std::vector<Eigen::Triplet<double>> triplets; // 用于构建稀疏约束矩阵的三元组列表
    Eigen::SparseMatrix<double> A_constraint;     // 约束矩阵
    Eigen::VectorXd lower_bound;                  // 约束下界
    Eigen::VectorXd upper_bound;                  // 约束上界

    // 足端接触状态 (true: 支撑相, false: 摆动相)
    bool contact_states[n][4]; // 每个预测步长的接触状态

    // 重力加速度 (m/s^2)
    constexpr static double gravity = 9.81;

    // 采样时间 (s)
    double dt;
    double mass;
    const Eigen::Matrix3d& inertia;

    // 摩擦系数
    double friction_coeff;

    /**
     * @brief 构建系统状态转移矩阵A和输入矩阵B
     * @param dt 采样时间
     * @param mass 机器人质量
     * @param inertia 机器人惯性张量
     * @param foot_positions 足端位置相对于质心的坐标
     *
     * 状态向量: x = [p; θ; v; ω] (12维)
     * p: 位置 [x, y, z]
     * θ: 姿态角 [roll, pitch, yaw]
     * v: 线速度 [vx, vy, vz]
     * ω: 角速度 [ωx, ωy, ωz]
     *
     * 输入向量: u = [f1; f2; f3; f4] (12维)
     * fi: 第i个足端的力 [fx, fy, fz]
     */
    void buildSystemMatrices(const Eigen::Matrix<double, 3, 4>& foot_positions);

    /**
     * @brief 设置状态惩罚矩阵Q和输入惩罚矩阵R
     * @param q_weights 状态权重向量 (12维)
     * @param r_weights 输入权重向量 (12维)
     */
    void setWeightMatrices(const Eigen::Matrix<double, 12, 1>& q_weights, const Eigen::Matrix<double, 12, 1>& r_weights);

    //从系统矩阵构建全时域状态转移矩阵
    void make_Aqp();

    //从系统输入矩阵构建全时域输入矩阵
    void make_Bqp();

    //构建重力项对全时域状态的影响矩阵
    void make_Cqp();

    
    void make_L();
    void make_K();
    void make_H();
    void make_G(Eigen::Vector<double, 12> x_ref);

    /**
     * @brief 构造力约束矩阵和边界
     * 包括摆动腿零力约束和支撑腿摩擦锥约束
     *
     * 约束形式: lower_bound <= A_constraint * u <= upper_bound
     *
     * 摆动腿约束 (contact_states[k][i] == false):
     *   fx[k,i] = 0
     *   fy[k,i] = 0
     *   fz[k,i] = 0
     *
     * 支撑腿摩擦锥约束 (contact_states[k][i] == true):
     *   使用摩擦金字塔近似（4个面）:
     *   -μ*fz <= fx <= μ*fz
     *   -μ*fz <= fy <= μ*fz
     *   fz_min <= fz <= fz_max
     */
    void buildConstraints();

    /**
     * @brief 求解MPC优化问题并返回第一个控制量
     * @return 第一个控制量 (12维向量)
     */
    Eigen::Matrix<double, 12, 1> MPC_Calc();
};
