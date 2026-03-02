#include "states/mpc2.hpp"
#include <cmath>

MPC2State::MPC2State(Robot* robot)
    : BaseState<Robot>("mpc2") {
    // 初始化足端接触状态（默认都是支撑相）
    for (int k = 0; k < n; k++) {
        for (int i = 0; i < 4; i++) {
            contact_states[k][i] = true;
        }
    }

    // 初始化QP矩阵大小
    H.resize(12*n, 12*n);
    A_qp.setZero();
    B_qp.setZero();
    C_qp.setZero();
    L.setZero();
    K.setZero();

    // 初始化摩擦系数
    friction_coeff = 0.5;

    // 初始化采样时间
    dt = 0.01; // 默认10ms
}

bool MPC2State::enter(Robot* robot, const std::string& last_status) {
    (void)robot;
    (void)last_status;

    return true;
}

std::string MPC2State::update(Robot* robot) { return "mpc"; }

//目前还存在位置、速度坐标系不正确的问题，暂时不可用，需要机器人质心坐标系和地面坐标系的相互转换
void MPC2State::buildSystemMatrices(const Eigen::Matrix<double, 3, 4>& foot_positions) {
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

    auto inertia_inv=inertia.inverse();

    // 对每个足端
    for (int i = 0; i < 4; i++) {
        // 足端力对线加速度的贡献
        B_c.block<3, 3>(6, 3 * i) = force_to_acc;

        // 足端力对角加速度的贡献（力矩 = r × F）
        // 使用反对称矩阵表示叉乘
        Eigen::Vector3d r = foot_positions.col(i);
        Eigen::Matrix3d r_skew;
        r_skew << 0, -r(2), r(1), r(2), 0, -r(0), -r(1), r(0), 0;

        B_c.block<3, 3>(9, 3 * i) = inertia_inv * r_skew;
    }

    // 离散化：使用欧拉法
    // 完整的动力学方程考虑重力：
    // x(k+1) = x(k) + dt * (A_c * x(k) + B_c * u(k) + g_c)
    // 其中 g_c = [0,0,0,0,0,0,0,0,-g,0,0,0]^T (重力只影响z方向速度)
    // x(k+1) = (I + dt * A_c) * x(k) + (dt * B_c) * u(k) + dt * g_c

    A = Eigen::Matrix<double, 12, 12>::Identity() + dt * A_c;
    B = dt * B_c;

    // 构建重力项向量（离散化后）
    Eigen::Matrix<double, 12, 1> g_c;
    g_c.setZero();
    g_c(8) = -gravity; // z方向速度受重力影响
    g      = dt * g_c; // 离散化
}

void MPC2State::setWeightMatrices(const Eigen::Matrix<double, 12, 1>& q_weights, const Eigen::Matrix<double, 12, 1>& r_weights) {
    // 设置对角权重矩阵
    Q.diagonal() = q_weights;
    R.diagonal() = r_weights;
}

void MPC2State::make_Aqp() {
    auto A_power = A;
    for (int i = 0; i < n; i++) // 堆叠计算A_qp
    {
        A_qp.block(12 * i, 0, 12, 12) = A_power;
        A_power                       = A_power * A;
    }
}

void MPC2State::make_Bqp() {
    for (int row = 0; row < n; row++) {
        for (int col = 0; col <= row; col++) {
            Eigen::Matrix<double, 12, 12> A_power = Eigen::Matrix<double, 12, 12>::Identity();

            for (int p = 0; p < row - col; p++)
                A_power = A_power * A;

            B_qp.block(12 * row, 12 * col, 12, 12) = A_power * B;
        }
    }
}

void MPC2State::make_Cqp() {
    for (int k = 0; k < n; k++) {
        auto sum                              = Eigen::Matrix<double, 12, 12>::Zero(12, 12);
        Eigen::Matrix<double, 12, 12> A_power = Eigen::Matrix<double, 12, 12>::Identity(12, 12);

        for (int i = 0; i <= k; i++) {
            sum += A_power;
            A_power = A_power * A;
        }

        C_qp.block(12 * k, 0, 12, 12) = sum;
    }
}

void MPC2State::make_L() {
    for (int i = 0; i < n; i++) {
        L.block(12 * i, 12 * i, 12, 12) = Q;
    }
}

void MPC2State::make_K() {
    for (int i = 0; i < n; i++) {
        K.block(12 * i, 12 * i, 12, 12) = R;
    }
}

void MPC2State::make_H() {
    H = 2*( B_qp.transpose() * L * B_qp + K );
}

void MPC2State::make_G(Eigen::Vector<double,12> x_ref) {
    Eigen::Matrix<double,12*n,12> X_ref;
    for(int i=0;i<n;i++)
        X_ref.block(12*i,0,12,1) = x_ref;
    G = 2 * B_qp.transpose() * L * (A_qp*x0 + C_qp*g - X_ref);
}

void MPC2State::buildConstraints() {

    constexpr double f_min = 0.0;
    constexpr double f_max = 60.0;

    int num_constraints = 0;

    for (int k = 0; k < n; k++) {
        for (int leg = 0; leg < 4; leg++) {
            num_constraints += contact_states[k][leg] ? 5 : 3;
        }
    }

    A_constraint.resize(num_constraints, 12 * n);
    lower_bound.resize(num_constraints);
    upper_bound.resize(num_constraints);


    triplets.clear();
    triplets.reserve(num_constraints * 2);  // 平均每行2个非零

    int row = 0;

    for (int k = 0; k < n; k++) {
        for (int leg = 0; leg < 4; leg++) {

            int idx = k * 12 + leg * 3;

            if (contact_states[k][leg]) {

                // ===== fz bounds =====
                triplets.emplace_back(row, idx + 2, 1.0);
                lower_bound(row) = f_min;
                upper_bound(row) = f_max;
                row++;

                // ===== friction cone =====

                // fx - μ fz ≤ 0
                triplets.emplace_back(row, idx, 1.0);
                triplets.emplace_back(row, idx + 2, -friction_coeff);
                lower_bound(row) = -OsqpEigen::INFTY;
                upper_bound(row) = 0.0;
                row++;

                // -fx - μ fz ≤ 0
                triplets.emplace_back(row, idx, -1.0);
                triplets.emplace_back(row, idx + 2, -friction_coeff);
                lower_bound(row) = -OsqpEigen::INFTY;
                upper_bound(row) = 0.0;
                row++;

                // fy - μ fz ≤ 0
                triplets.emplace_back(row, idx + 1, 1.0);
                triplets.emplace_back(row, idx + 2, -friction_coeff);
                lower_bound(row) = -OsqpEigen::INFTY;
                upper_bound(row) = 0.0;
                row++;

                // -fy - μ fz ≤ 0
                triplets.emplace_back(row, idx + 1, -1.0);
                triplets.emplace_back(row, idx + 2, -friction_coeff);
                lower_bound(row) = -OsqpEigen::INFTY;
                upper_bound(row) = 0.0;
                row++;

            } else {

                // ===== swing leg: exact equality =====
                for (int axis = 0; axis < 3; axis++) {
                    triplets.emplace_back(row, idx + axis, 1.0);
                    lower_bound(row) = 0.0;
                    upper_bound(row) = 0.0;
                    row++;
                }
            }
        }
    }

    A_constraint.setFromTriplets(triplets.begin(), triplets.end());
}

Eigen::Matrix<double, 12, 1> MPC2State::MPC_Calc() {
    // QP问题形式: min 0.5 * u^T * H * u + G^T * u
    // subject to: lower_bound <= A_constraint * u <= upper_bound
    
    // 构建约束
    buildConstraints();
    
    // 设置QP问题数据
    if (!solver.isInitialized()) {
        // 第一次初始化求解器
        solver.data()->setNumberOfVariables(12 * n);
        solver.data()->setNumberOfConstraints(A_constraint.rows());
        
        if (!solver.data()->setHessianMatrix(H)) {
            throw std::runtime_error("Failed to set Hessian matrix");
        }
        
        if (!solver.data()->setGradient(G)) {
            throw std::runtime_error("Failed to set gradient vector");
        }
        
        if (!solver.data()->setLinearConstraintsMatrix(A_constraint)) {
            throw std::runtime_error("Failed to set constraint matrix");
        }
        
        if (!solver.data()->setLowerBound(lower_bound)) {
            throw std::runtime_error("Failed to set lower bound");
        }
        
        if (!solver.data()->setUpperBound(upper_bound)) {
            throw std::runtime_error("Failed to set upper bound");
        }
        
        // 设置求解器选项
        solver.settings()->setWarmStart(true);
        solver.settings()->setVerbosity(false);
        solver.settings()->setMaxIteration(4000);
        solver.settings()->setAbsoluteTolerance(1e-4);
        solver.settings()->setRelativeTolerance(1e-4);
        
        // 初始化求解器
        if (!solver.initSolver()) {
            throw std::runtime_error("Failed to initialize solver");
        }
    } else {
        // 更新现有求解器的数据（warm start）
        if (!solver.updateHessianMatrix(H)) {
            throw std::runtime_error("Failed to update Hessian matrix");
        }
        
        if (!solver.updateGradient(G)) {
            throw std::runtime_error("Failed to update gradient vector");
        }
        
        if (!solver.updateLinearConstraintsMatrix(A_constraint)) {
            throw std::runtime_error("Failed to update constraint matrix");
        }
        
        if (!solver.updateBounds(lower_bound, upper_bound)) {
            throw std::runtime_error("Failed to update bounds");
        }
    }
    
    // 求解 QP 问题
    OsqpEigen::ErrorExitFlag status = solver.solveProblem();
    if (status != OsqpEigen::ErrorExitFlag::NoError) {
        throw std::runtime_error("Failed to solve QP problem, error code: " + std::to_string(static_cast<int>(status)));
    }
    
    // 获取解
    Eigen::VectorXd solution = solver.getSolution();
    
    // 返回第一个控制量（前12个元素）
    return solution.head<12>();
}
