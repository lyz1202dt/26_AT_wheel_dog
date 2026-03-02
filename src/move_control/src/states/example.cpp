/**
 * @file qpoases_example.cpp
 * @brief qpOASES使用示例
 * @details 演示如何在ROS2项目中使用qpOASES求解二次规划问题
 */

#include <qpOASES.hpp>
#include <iostream>
#include <vector>

/**
 * @brief 简单的QP问题求解示例
 * @details 求解问题:
 *          min  0.5 * x^T * H * x + g^T * x
 *          s.t. lb <= x <= ub
 *               lbA <= A*x <= ubA
 */
void simpleQPExample() {
    USING_NAMESPACE_QPOASES

    std::cout << "\n=== 简单QP问题示例 ===" << std::endl;

    /* 设置QP问题数据 */
    real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };  // Hessian矩阵 (目标函数二次项系数)
    real_t A[1*2] = { 1.0, 1.0 };             // 约束矩阵
    real_t g[2] = { 1.5, 1.0 };               // 目标函数线性项系数
    real_t lb[2] = { 0.5, -2.0 };             // 变量下界
    real_t ub[2] = { 5.0, 2.0 };              // 变量上界
    real_t lbA[1] = { -1.0 };                 // 约束下界
    real_t ubA[1] = { 2.0 };                  // 约束上界

    /* 创建QProblem对象 (2个变量, 1个约束) */
    QProblem example(2, 1);

    /* 设置求解器选项 */
    Options options;
    options.printLevel = PL_MEDIUM;  // 设置打印级别
    example.setOptions(options);

    /* 求解QP问题 */
    int_t nWSR = 10;  // 最大工作集重组次数
    returnValue retVal = example.init(H, g, A, lb, ub, lbA, ubA, nWSR);

    if (retVal == SUCCESSFUL_RETURN) {
        /* 获取并打印解 */
        real_t xOpt[2];
        real_t yOpt[2+1];
        example.getPrimalSolution(xOpt);
        example.getDualSolution(yOpt);
        
        std::cout << "求解成功!" << std::endl;
        std::cout << "最优解: x = [" << xOpt[0] << ", " << xOpt[1] << "]" << std::endl;
        std::cout << "目标函数值: " << example.getObjVal() << std::endl;
    } else {
        std::cout << "求解失败，返回值: " << retVal << std::endl;
    }
}

/**
 * @brief MPC相关的QP问题求解示例
 * @details 展示如何使用热启动求解连续的QP问题，这在MPC中很常见
 */
void mpcQPExample() {
    USING_NAMESPACE_QPOASES

    std::cout << "\n=== MPC热启动示例 ===" << std::endl;

    /* 初始QP问题数据 */
    real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
    real_t A[1*2] = { 1.0, 1.0 };
    real_t g[2] = { 1.5, 1.0 };
    real_t lb[2] = { 0.5, -2.0 };
    real_t ub[2] = { 5.0, 2.0 };
    real_t lbA[1] = { -1.0 };
    real_t ubA[1] = { 2.0 };

    /* 创建QProblem对象 */
    QProblem mpcQP(2, 1);
    Options options;
    options.printLevel = PL_LOW;
    mpcQP.setOptions(options);

    /* 求解初始QP问题 */
    int_t nWSR = 10;
    mpcQP.init(H, g, A, lb, ub, lbA, ubA, nWSR);

    /* 获取初始解 */
    real_t xOpt[2];
    mpcQP.getPrimalSolution(xOpt);
    std::cout << "初始解: x = [" << xOpt[0] << ", " << xOpt[1] << "]" << std::endl;

    /* 模拟MPC的滚动优化：更新参数并热启动求解 */
    real_t g_new[2] = { 1.0, 1.5 };
    real_t lb_new[2] = { 0.0, -1.0 };
    real_t ub_new[2] = { 5.0, -0.5 };
    real_t lbA_new[1] = { -2.0 };
    real_t ubA_new[1] = { 1.0 };

    nWSR = 10;
    mpcQP.hotstart(g_new, lb_new, ub_new, lbA_new, ubA_new, nWSR);

    /* 获取更新后的解 */
    mpcQP.getPrimalSolution(xOpt);
    std::cout << "热启动后的解: x = [" << xOpt[0] << ", " << xOpt[1] << "]" << std::endl;
    std::cout << "目标函数值: " << mpcQP.getObjVal() << std::endl;
}

/**
 * @brief 使用SQProblem求解简单的序列二次规划问题
 */
void sqpExample() {
    USING_NAMESPACE_QPOASES

    std::cout << "\n=== SQProblem示例 ===" << std::endl;

    /* 问题规模 */
    const int nV = 2;  // 变量数量

    /* 设置问题数据 */
    real_t H[nV*nV] = { 1.0, 0.0, 0.0, 0.5 };
    real_t g[nV] = { 1.5, 1.0 };
    real_t lb[nV] = { 0.5, -2.0 };
    real_t ub[nV] = { 5.0, 2.0 };

    /* 创建SQProblem对象（只有边界约束，无线性约束） */
    SQProblem sqp(nV, 0);

    Options options;
    options.printLevel = PL_LOW;
    sqp.setOptions(options);

    /* 求解 */
    int_t nWSR = 10;
    returnValue retVal = sqp.init(H, g, NULL, lb, ub, NULL, NULL, nWSR);

    if (retVal == SUCCESSFUL_RETURN) {
        real_t xOpt[nV];
        sqp.getPrimalSolution(xOpt);
        std::cout << "求解成功!" << std::endl;
        std::cout << "最优解: x = [" << xOpt[0] << ", " << xOpt[1] << "]" << std::endl;
        std::cout << "目标函数值: " << sqp.getObjVal() << std::endl;
    }
}

int main() {
    std::cout << "==================================" << std::endl;
    std::cout << "qpOASES使用示例" << std::endl;
    std::cout << "==================================" << std::endl;

    // 运行不同的示例
    simpleQPExample();
    mpcQPExample();
    sqpExample();

    std::cout << "\n所有示例运行完成!" << std::endl;
    return 0;
}