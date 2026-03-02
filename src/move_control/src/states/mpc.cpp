#include "states/mpc.hpp"
#include <cmath>

MPCState::MPCState(Robot* robot) : BaseState<Robot>("mpc")
{
    // 初始化足端接触状态（默认都是支撑相）
    for(int i = 0; i < num_legs; i++) {
        contact_states[i] = true;
    }
    
    // 初始化摩擦系数
    friction_coeff = 0.5;
    
    // 初始化采样时间
    dt = 0.01;  // 默认10ms
    
    // 初始化矩阵为零
    a.setZero();
    b.setZero();
    
}

bool MPCState::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;
    
    int total_vars = 12 * (n + 1) + 12 * n;
    G.resize(total_vars);
    H.resize(total_vars, total_vars);

    computeHessianMatrix();

    return true;
}

std::string MPCState::update(Robot* robot) {

    return "mpc";
}

void MPCState::buildSystemMatrices(double dt, double mass, 
                                   const Eigen::Matrix3d& inertia,
                                   const Eigen::Matrix<double, 3, 4>& foot_positions) {
    // 存储采样时间
    this->dt = dt;
    
    // 状态向量: x = [p; θ; v; ω] (12维)
    // 输入向量: u = [f1; f2; f3; f4] (12维, 每个足端3个力分量)
    
    // 连续时间系统矩阵 A_c
    Eigen::Matrix<double, 12, 12> A_c;
    A_c.setZero();
    
    // 位置对速度的导数: dp/dt = v
    A_c.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
    
    // 姿态对角速度的导数: dθ/dt = ω (小角度近似)
    A_c.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    
    // 速度对加速度的导数: dv/dt = a (由控制输入决定)
    // 角速度对角加速度的导数: dω/dt = α (由控制输入决定)
    // 这些由输入矩阵B决定
    
    // 连续时间输入矩阵 B_c
    Eigen::Matrix<double, 12, 12> B_c;
    B_c.setZero();
    
    // 力对加速度的影响: dv/dt = F/m - g
    // 重力补偿（在z方向）
    Eigen::Matrix3d force_to_acc = Eigen::Matrix3d::Identity() / mass;
    
    // 对每个足端
    for(int i = 0; i < num_legs; i++) {
        // 足端力对线加速度的贡献
        B_c.block<3, 3>(6, 3*i) = force_to_acc;
        
        // 足端力对角加速度的贡献（力矩 = r × F）
        // 使用反对称矩阵表示叉乘
        Eigen::Vector3d r = foot_positions.col(i);
        Eigen::Matrix3d r_skew;
        r_skew << 0, -r(2), r(1),
                  r(2), 0, -r(0),
                  -r(1), r(0), 0;
        
        B_c.block<3, 3>(9, 3*i) = inertia.inverse() * r_skew;
    }
    
    // 离散化：使用欧拉法
    // 完整的动力学方程考虑重力：
    // x(k+1) = x(k) + dt * (A_c * x(k) + B_c * u(k) + g_c)
    // 其中 g_c = [0,0,0,0,0,0,0,0,-g,0,0,0]^T (重力只影响z方向速度)
    // x(k+1) = (I + dt * A_c) * x(k) + (dt * B_c) * u(k) + dt * g_c
    
    a = Eigen::Matrix<double, 12, 12>::Identity() + dt * A_c;
    b = dt * B_c;
    
    // 构建重力项向量（离散化后）
    Eigen::Matrix<double, 12, 1> g_c;
    g_c.setZero();
    g_c(8) = -gravity;  // z方向速度受重力影响
    g = dt * g_c;       // 离散化
}

void MPCState::setWeightMatrices(const Eigen::Matrix<double, 12, 1>& q_weights,
                                 const Eigen::Matrix<double, 12, 1>& r_weights) {
    // 设置对角权重矩阵
    Q.diagonal() = q_weights;
    R.diagonal() = r_weights;
}

void MPCState::setFootConstraint(int leg_index, bool is_contact, double mu) {
    if(leg_index < 0 || leg_index >= num_legs) {
        return; // 无效的腿索引
    }
    
    contact_states[leg_index] = is_contact;
    
    if(is_contact) {
        friction_coeff = mu;
    }
}

void MPCState::computeHessianMatrix() {
    H.setZero();
    
    // 状态惩罚：对所有状态应用Q矩阵
    for(int i = 0; i < n + 1; i++) {
        for(int j = 0; j < 12; j++) {
            double value = Q.diagonal()[j];
            if(value != 0) {
                int idx = 12 * i + j;
                H.insert(idx, idx) = value;
            }
        }
    }
    
    // 控制输入惩罚：对所有控制输入应用R矩阵
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < 12; j++) {
            double value = R.diagonal()[j];
            if(value != 0) {
                int idx = 12 * (n + 1) + 12 * i + j;
                H.insert(idx, idx) = value;
            }
        }
    }
    
    H.makeCompressed();
}

void MPCState::computeGradientVector(const Eigen::Matrix<double, 12, 1>& x_ref) {
    G.setZero();
    
    // 计算 Q * (-x_ref)
    Eigen::Matrix<double, 12, 1> Q_x_ref = Q * (-x_ref);
    
    // 为所有状态变量设置梯度
    for(int i = 0; i < n + 1; i++) {
        G.segment<12>(12 * i) = Q_x_ref;
    }
    
    // 控制输入部分的梯度为0（没有参考控制输入）
}

Eigen::Matrix<double, 12, 1> MPCState::solveAndGetFirstControl(
    const Eigen::Matrix<double, 12, 1>& x0,
    const Eigen::Matrix<double, 12, 1>& x_ref,
    const Eigen::Matrix<double, 12, 1>& u_max,
    const Eigen::Matrix<double, 12, 1>& u_min) {
    
    // 构建约束矩阵
    int total_vars = 12 * (n + 1) + 12 * n;
    // 基础约束数：初始状态(12) + 动力学(n*12) + 变量上下界(total_vars)
    int num_constraints = 12 * (n + 1) + total_vars;

    // 为摩擦金字塔添加额外的线性约束
    // 每个采样时刻、每条接触腿产生 4 条不等式：
    //   fx <= μ fz, -fx <= μ fz, fy <= μ fz, -fy <= μ fz
    // 这些不能通过简单的变量上下界表达，因此需要额外的矩阵行。
    int friction_ineq_per_step = 4 * num_legs;
    int extra_constraints = friction_ineq_per_step * n;
    num_constraints += extra_constraints;
    
    Eigen::SparseMatrix<double> constraint_matrix;
    constraint_matrix.resize(num_constraints, total_vars);
    
    // 初始状态约束
    for(int i = 0; i < 12; i++) {
        constraint_matrix.insert(i, i) = -1.0;
    }
    
    // 动力学约束
    for(int i = 0; i < n; i++) {
        // -x(k+1)
        for(int j = 0; j < 12; j++) {
            constraint_matrix.insert(12 * (i + 1) + j, 12 * (i + 1) + j) = -1.0;
        }
        
        // A*x(k)
        for(int j = 0; j < 12; j++) {
            for(int k = 0; k < 12; k++) {
                double value = a(j, k);
                if(std::abs(value) > 1e-10) {
                    constraint_matrix.insert(12 * (i + 1) + j, 12 * i + k) = value;
                }
            }
        }
        
        // B*u(k)
        for(int j = 0; j < 12; j++) {
            for(int k = 0; k < 12; k++) {
                double value = b(j, k);
                if(std::abs(value) > 1e-10) {
                    constraint_matrix.insert(12 * (i + 1) + j, 
                                           12 * (n + 1) + 12 * i + k) = value;
                }
            }
        }
    }
    
    // 不等式约束（状态和控制界限，简单变量上下界）
    for(int i = 0; i < total_vars; i++) {
        constraint_matrix.insert(12 * (n + 1) + i, i) = 1.0;
    }

    // 摩擦金字塔约束行：在基础约束之后追加
    int friction_row_start = 12 * (n + 1) + total_vars;
    for(int step = 0; step < n; step++) {
        int u_base = 12 * (n + 1) + 12 * step;
        for(int leg = 0; leg < num_legs; leg++) {
            int row = friction_row_start + (step * friction_ineq_per_step) + 4 * leg;
            int fx_idx = u_base + 3 * leg + 0;
            int fy_idx = u_base + 3 * leg + 1;
            int fz_idx = u_base + 3 * leg + 2;

            if(contact_states[leg]) {
                // fx - μ fz <= 0
                constraint_matrix.insert(row + 0, fx_idx) = 1.0;
                constraint_matrix.insert(row + 0, fz_idx) = -friction_coeff;
                // -fx - μ fz <= 0
                constraint_matrix.insert(row + 1, fx_idx) = -1.0;
                constraint_matrix.insert(row + 1, fz_idx) = -friction_coeff;
                // fy - μ fz <= 0
                constraint_matrix.insert(row + 2, fy_idx) = 1.0;
                constraint_matrix.insert(row + 2, fz_idx) = -friction_coeff;
                // -fy - μ fz <= 0
                constraint_matrix.insert(row + 3, fy_idx) = -1.0;
                constraint_matrix.insert(row + 3, fz_idx) = -friction_coeff;
            } else {
                // 摆动腿没有力，已经在变量上下界中设置为0，
                // 额外的不等式可以留为零乘以任何变量。
            }
        }
    }
    
    constraint_matrix.makeCompressed();
    
    // 构建约束边界
    Eigen::VectorXd lower_bound(num_constraints);
    Eigen::VectorXd upper_bound(num_constraints);
    
    // 初始状态等式约束
    lower_bound.segment<12>(0) = -x0;
    upper_bound.segment<12>(0) = -x0;
    
    // 动力学等式约束（包含重力补偿）
    // 完整的动力学约束： -x(k+1) + A*x(k) + B*u(k) = -g
    // 其中 g 是在 buildSystemMatrices 中计算的离散化重力项
    for(int i = 1; i < n + 1; i++) {
        lower_bound.segment<12>(12 * i) = -g;
        upper_bound.segment<12>(12 * i) = -g;
    }
    
    // 状态不等式约束（设置为无穷大，表示无约束）
    for(int i = 0; i < n + 1; i++) {
        lower_bound.segment<12>(12 * (n + 1) + 12 * i).setConstant(-OsqpEigen::INFTY);
        upper_bound.segment<12>(12 * (n + 1) + 12 * i).setConstant(OsqpEigen::INFTY);
    }
    
    // 控制输入不等式约束
    for(int i = 0; i < n; i++) {
        int idx = 12 * (n + 1) + 12 * (n + 1) + 12 * i;
        
        // 对于每个足端，根据接触状态设置力的上下界
        for(int leg = 0; leg < num_legs; leg++) {
            double fz_min = u_min(3 * leg + 2);  // 最小垂直力
            double fz_max = u_max(3 * leg + 2);  // 最大垂直力

            if(contact_states[leg]) {
                // 仅对垂直力施加硬界，水平力由摩擦不等式控制
                lower_bound(idx + 3 * leg + 0) = -OsqpEigen::INFTY; // fx 无直接上下界
                upper_bound(idx + 3 * leg + 0) = OsqpEigen::INFTY;
                lower_bound(idx + 3 * leg + 1) = -OsqpEigen::INFTY; // fy 无直接上下界
                upper_bound(idx + 3 * leg + 1) = OsqpEigen::INFTY;
                lower_bound(idx + 3 * leg + 2) = fz_min;
                upper_bound(idx + 3 * leg + 2) = fz_max;
            } else {
                // 摆动相：零力约束
                lower_bound.segment<3>(idx + 3 * leg).setZero();
                upper_bound.segment<3>(idx + 3 * leg).setZero();
            }
        }
    }
    
    // 计算Hessian和梯度
    computeHessianMatrix();
    computeGradientVector(x_ref);
    
    // 设置求解器
    solver.data()->setNumberOfVariables(total_vars);
    solver.data()->setNumberOfConstraints(num_constraints);
    
    if(!solver.data()->setHessianMatrix(H)) {
        return Eigen::Matrix<double, 12, 1>::Zero();
    }
    if(!solver.data()->setGradient(G)) {
        return Eigen::Matrix<double, 12, 1>::Zero();
    }
    if(!solver.data()->setLinearConstraintsMatrix(constraint_matrix)) {
        return Eigen::Matrix<double, 12, 1>::Zero();
    }
    if(!solver.data()->setLowerBound(lower_bound)) {
        return Eigen::Matrix<double, 12, 1>::Zero();
    }
    if(!solver.data()->setUpperBound(upper_bound)) {
        return Eigen::Matrix<double, 12, 1>::Zero();
    }
    
    // 初始化求解器
    if(!solver.initSolver()) {
        return Eigen::Matrix<double, 12, 1>::Zero();
    }
    
    // 求解QP问题
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        return Eigen::Matrix<double, 12, 1>::Zero();
    }
    
    // 提取解
    Eigen::VectorXd solution = solver.getSolution();
    
    // 返回第一个控制量（前12维是第一个时刻的控制输入）
    return solution.segment<12>(12 * (n + 1));
}
