#!/usr/bin/env python3
"""
遥控器数据验证脚本
用于实时监听和验证遥控器数据的正确性
"""

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import MoveCmd
import time


class RemoteDataValidator(Node):
    def __init__(self):
        super().__init__('remote_data_validator')
        
        # 订阅遥控器数据
        self.subscription = self.create_subscription(
            MoveCmd,
            'robot_move_cmd',
            self.listener_callback,
            10
        )
        
        self.msg_count = 0
        self.last_time = time.time()
        self.last_values = None
        
    def listener_callback(self, msg):
        self.msg_count += 1
        current_time = time.time()
        time_elapsed = current_time - self.last_time
        
        # 计算消息频率
        if time_elapsed > 1.0:
            freq = self.msg_count / time_elapsed
            self.get_logger().info(f"消息频率: {freq:.1f} Hz")
            self.msg_count = 0
            self.last_time = current_time
        
        # 验证数据有效性
        try:
            # 检查速度范围
            assert isinstance(msg.step_mode, int), "step_mode 应为整数"
            assert -2.0 <= msg.vx <= 2.0, f"vx 超出范围: {msg.vx}"
            assert -2.0 <= msg.vy <= 2.0, f"vy 超出范围: {msg.vy}"
            assert -2.0 <= msg.vz <= 2.0, f"vz 超出范围: {msg.vz}"
            assert -2.0 <= msg.wheel_vel <= 2.0, f"wheel_vel 超出范围: {msg.wheel_vel}"
            
            # 检查数据是否变化（可选，用于检测卡死）
            current_values = (msg.vx, msg.vy, msg.vz, msg.wheel_vel, msg.step_mode)
            if self.last_values != current_values:
                self.get_logger().debug(
                    f"数据更新 | vx={msg.vx:6.3f} vy={msg.vy:6.3f} vz={msg.vz:6.3f} "
                    f"wheel={msg.wheel_vel:6.3f} mode=0x{msg.step_mode:02x}"
                )
                self.last_values = current_values
            
        except AssertionError as e:
            self.get_logger().error(f"数据验证失败: {e}")
        except Exception as e:
            self.get_logger().error(f"处理消息异常: {e}")


def main(args=None):
    print("=" * 60)
    print("遥控器数据验证器")
    print("=" * 60)
    print("监听话题: /robot_move_cmd")
    print("按 Ctrl+C 退出")
    print("=" * 60)
    print("")
    
    rclpy.init(args=args)
    node = RemoteDataValidator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n程序已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
