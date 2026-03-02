#include "core/robot.hpp"
#include "fsm/base_state.hpp"
#include <OsqpEigen/OsqpEigen.h>

class Robot;

class MPCState : public BaseState<Robot>{
public:
    MPCState(Robot* robot);
    
    bool enter(Robot* robot, const std::string &last_status) override;
    std::string update(Robot* robot) override;
    
private:
    constexpr static int n = 8;    //MPC预测步数
    constexpr static int num_legs = 4;  //足端数量

    OsqpEigen::Solver solver;   //QP求解器

    Eigen::Matrix<double,12,12> a;       //系统状态矩阵
    Eigen::Matrix<double,12,12> b;        //系统输入矩阵 (12x12 for 4 legs x 3 forces)
    Eigen::Matrix<double,12,1> g;         //重力项向量 (离散化后)

    Eigen::DiagonalMatrix<double, 12> Q;    //状态惩罚矩阵
    Eigen::DiagonalMatrix<double, 12> R;    //控制输入惩罚矩阵

    Eigen::SparseMatrix<double> H; //海森堡矩阵
    Eigen::VectorXd G;  //梯度向量

    // 足端接触状态 (true: 支撑相, false: 摆动相)
    bool contact_states[num_legs];
    
    // 摩擦系数
    double friction_coeff;
    
    // 重力加速度 (m/s^2)
    constexpr static double gravity = 9.81;
    
    // 采样时间 (s)
    double dt;

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
    void buildSystemMatrices(double dt, double mass, 
                            const Eigen::Matrix3d& inertia,
                            const Eigen::Matrix<double, 3, 4>& foot_positions);

    /**
     * @brief 设置状态惩罚矩阵Q和输入惩罚矩阵R
     * @param q_weights 状态权重向量 (12维)
     * @param r_weights 输入权重向量 (12维)
     */
    void setWeightMatrices(const Eigen::Matrix<double, 12, 1>& q_weights,
                          const Eigen::Matrix<double, 12, 1>& r_weights);

    /**
     * @brief 设置四个足端的约束类型
     * @param leg_index 足端索引 (0-3)
     * @param is_contact true表示支撑相(摩擦锥约束), false表示摆动相(零力约束)
     * @param mu 摩擦系数(仅在支撑相有效)
     */
    void setFootConstraint(int leg_index, bool is_contact, double mu = 0.5);

    /**
     * @brief 计算海森堡矩阵H
     * H用于二次规划问题: min 0.5 * x^T * H * x + G^T * x
     * H的维度为 [12*(n+1) + 12*n] x [12*(n+1) + 12*n]
     */
    void computeHessianMatrix();

    /**
     * @brief 计算梯度向量G
     * @param x_ref 参考状态轨迹
     * G的维度为 [12*(n+1) + 12*n]
     */
    void computeGradientVector(const Eigen::Matrix<double, 12, 1>& x_ref);

    /**
     * @brief 求解MPC优化问题并返回第一个控制量
     * @param x0 当前状态
     * @param x_ref 参考状态
     * @param u_max 输入上界
     * @param u_min 输入下界
     * @return 第一个控制量 (12维向量)
     */
    Eigen::Matrix<double, 12, 1> solveAndGetFirstControl(
        const Eigen::Matrix<double, 12, 1>& x0,
        const Eigen::Matrix<double, 12, 1>& x_ref,
        const Eigen::Matrix<double, 12, 1>& u_max,
        const Eigen::Matrix<double, 12, 1>& u_min);

};
