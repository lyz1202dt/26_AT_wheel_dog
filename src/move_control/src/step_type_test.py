#!/usr/bin/env python3
"""
自动测试脚本：模拟 rqt 修改 step_type 参数
测试流程:
1. 初始 step_type = 0 (站立)
2. 修改 step_type = 1 (跳跃步态准备)
3. 等待 5 秒观察
4. 修改 step_type = 5 (执行跳跃流程)
5. 等待 30 秒观察完整流程
6. 输出测试结果
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import time
import sys

class StepTypeTester(Node):
    def __init__(self):
        super().__init__('step_type_tester')
        
        # 创建参数客户端
        self.param_client = self.create_client(
            'set_parameters',
            '/test_move_node'
        )
        
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 test_move_node 参数服务...')
        
        self.get_logger().info('✅ 连接到 test_move_node')
    
    def set_step_type(self, value):
        """设置 step_type 参数"""
        request = SetParameters.Request()
        request.parameters = [
            Parameter('step_type', Parameter.Type.INTEGER, value).to_parameter_msg()
        ]
        
        self.get_logger().info(f'📤 请求设置 step_type = {value}')
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result():
            result = future.result()
            if result.results[0].successful:
                self.get_logger().info(f'✅ 成功设置 step_type = {value}')
                return True
            else:
                self.get_logger().error(f'❌ 设置失败：{result.results[0].reason}')
                return False
        else:
            self.get_logger().error('❌ 服务调用超时')
            return False
    
    def run_test(self):
        """执行测试流程"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🚀 开始自动化测试流程')
        self.get_logger().info('='*60)
        
        # 步骤 1: 初始状态 (step_type = 0)
        self.get_logger().info('\n【步骤 1】初始状态：step_type = 0 (站立)')
        self.set_step_type(0)
        time.sleep(2.0)
        
        # 步骤 2: 设置为 1 (跳跃步态准备)
        self.get_logger().info('\n【步骤 2】设置 step_type = 1 (跳跃步态准备)')
        self.set_step_type(1)
        self.get_logger().info('⏱️  等待 5 秒观察 Mujoco 仿真...')
        for i in range(5, 0, -1):
            time.sleep(1.0)
            self.get_logger().info(f'   倒计时：{i}秒')
        
        # 步骤 3: 设置为 5 (执行跳跃流程)
        self.get_logger().info('\n【步骤 3】设置 step_type = 5 (执行跳跃流程) ⭐⭐⭐')
        self.set_step_type(5)
        self.get_logger().info('⏱️  等待 30 秒观察完整跳跃流程...')
        self.get_logger().info('💡 请观察 Mujoco 窗口中的机器人是否能完成整个流程')
        
        start_time = time.time()
        for i in range(30, 0, -1):
            elapsed = time.time() - start_time
            time.sleep(1.0)
            if i % 5 == 0:
                self.get_logger().info(f'   已运行：{elapsed:.1f}s, 剩余：{i}秒')
        
        # 步骤 4: 测试结果
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('📊 测试结果评估')
        self.get_logger().info('='*60)
        self.get_logger().info('请根据 Mujoco 仿真回答以下问题:')
        self.get_logger().info('1. 机器人是否完成了所有 16 个阶段 (0-15)?')
        self.get_logger().info('2. 是否有阶段出现摔倒、卡顿或无法继续的情况?')
        self.get_logger().info('3. 哪个阶段出现了问题？(如果有)')
        self.get_logger().info('')
        self.get_logger().info('💡 提示：如果流程失败，请查看 terminal 2 的日志输出')
        self.get_logger().info('💡 根据失败阶段调整 new_cross_step.cpp 中的参数')
        self.get_logger().info('='*60)
        
        # 步骤 5: 返回到安全状态
        self.get_logger().info('\n【步骤 4】测试结束，返回站立状态')
        self.set_step_type(0)
        
        self.get_logger().info('\n✅ 自动化测试脚本执行完毕')
        self.get_logger().info('📝 下一步操作:')
        self.get_logger().info('   - 如果测试失败：修改 new_cross_step.cpp 中对应阶段的参数')
        self.get_logger().info('   - 重新编译：colcon build --packages-select move_control')
        self.get_logger().info('   - 重新运行：source install/setup.bash && ros2 run move_control test_move_node')
        self.get_logger().info('   - 再次测试：ros2 run move_control step_type_test.py')

from rcl_interfaces.srv import SetParameters

def main():
    rclpy.init()
    tester = StepTypeTester()
    tester.run_test()
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
