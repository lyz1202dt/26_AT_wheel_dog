#!/usr/bin/env bash
# 🚀 远程遥控器驱动 - 快速参考卡片
# ═══════════════════════════════════════════════════════════════════════════

cat << 'EOF'

╔═══════════════════════════════════════════════════════════════════════════╗
║                    远程遥控器驱动快速参考 v1.0                            ║
║                     ROS2 Remote Control Driver                            ║
╚═══════════════════════════════════════════════════════════════════════════╝

📂 项目位置
─────────────────────────────────────────────────────────────────────────────
  src/remote_node/

🚀 快速开始 (3步)
─────────────────────────────────────────────────────────────────────────────
  1. colcon build --packages-select remote_node
  2. source install/setup.bash
  3. ros2 run remote_node remote_node

🔗 发布话题
─────────────────────────────────────────────────────────────────────────────
  名称: /robot_move_cmd
  类型: robot_interfaces/msg/MoveCmd
  字段: step_mode(uint32) wheel_vel(float32) vx vy vz(float32)

📊 查看数据
─────────────────────────────────────────────────────────────────────────────
  ros2 topic echo /robot_move_cmd
  python3 src/remote_node/scripts/validate_remote_data.py

⚙️  常用参数
─────────────────────────────────────────────────────────────────────────────
  ros2 param set /remote_node filter_gate 0.2        # 滤波系数
  ros2 param set /remote_node max_omega 180.0        # 最大角速度
  ros2 param set /remote_node max_speed 1.0          # 最大速度

🔧 调试
─────────────────────────────────────────────────────────────────────────────
  # 启用调试日志
  ros2 run remote_node remote_node --ros-args --log-level debug

  # 查看原始串口数据
  cat < /dev/ttyUSB1 | xxd

  # 测试串口连接
  stty -F /dev/ttyUSB1 115200

❌ 常见问题排查
─────────────────────────────────────────────────────────────────────────────
  问题: 串口打开失败
  解决: sudo usermod -a -G dialout $USER  # 然后重新登录

  问题: 通信超时
  解决: 检查遥控器电池、距离、波特率

  问题: 摇杆抖动
  解决: ros2 param set /remote_node filter_gate 0.1  # 增大滤波

  问题: 响应不灵敏  
  解决: ros2 param set /remote_node max_speed 2.0  # 增大最大速度

📚 关键文档
─────────────────────────────────────────────────────────────────────────────
  main: REMOTE_NODE_INTEGRATION_GUIDE.md     (集成说明)
  tech: STM32_TO_ROS2_MIGRATION_GUIDE.md     (技术细节)
  check: REMOTE_NODE_COMPLETION_CHECKLIST.md (验收清单)
  pkg: src/remote_node/README.md             (功能说明)

🔌 硬件连接
─────────────────────────────────────────────────────────────────────────────
  遥控器 --[USB]--> Linux PC
  串口设备: /dev/ttyUSB1 (或 ttyUSB0)
  波特率: 115200 bps
  数据包: 12字节 (帧头0xAA + 摇杆x4 + 按键x2 + 帧尾)

📋 工作流程
─────────────────────────────────────────────────────────────────────────────
  接收           处理             发布
  ───────────────────────────────────────────
  串口接收   →  滤波/变换    →  ROS2话题
  12字节      数据处理         MoveCmd消息

🎯 参数配置建议
─────────────────────────────────────────────────────────────────────────────
  [平稳模式] 精细控制    [标准模式] 通用偏好    [敏捷模式] 竞速
  ─────────────────────────────────────────────────────────────
  filter_gate: 0.1     filter_gate: 0.2      filter_gate: 0.3
  max_speed: 0.5       max_speed: 1.0        max_speed: 2.0
  max_omega: 90        max_omega: 180        max_omega: 360

✅ 验收清单
─────────────────────────────────────────────────────────────────────────────
  [ ] 编译成功 (colcon build)
  [ ] 串口连接 (ls /dev/ttyUSB*)
  [ ] 能接收数据 (cat < /dev/ttyUSB1 | xxd)
  [ ] 能启动节点 (ros2 run remote_node remote_node)
  [ ] 能看到话题 (ros2 topic list)
  [ ] 能订阅数据 (ros2 topic echo /robot_move_cmd)
  [ ] 无报错信息 (检查日志)

🔐 安全提示
─────────────────────────────────────────────────────────────────────────────
  ⚠️  200ms没收到遥控信号时会自动复位摇杆
  ⚠️  调试中不要在机器人旁边
  ⚠️  确保控制器能访问串口设备

📞 获取帮助
─────────────────────────────────────────────────────────────────────────────
  详细集成说明: REMOTE_NODE_INTEGRATION_GUIDE.md
  技术架构分析: STM32_TO_ROS2_MIGRATION_GUIDE.md
  项目完成清单: REMOTE_NODE_COMPLETION_CHECKLIST.md
  常见问题排查: src/remote_node/README.md

═══════════════════════════════════════════════════════════════════════════

✨ 版本: 1.0 (稳定)  |  日期: 2026-03-28  |  邮件: cloud1202@qq.com

═══════════════════════════════════════════════════════════════════════════

EOF

# 显示项目统计
echo ""
echo "📊 项目统计:"
echo "───────────────────────────────────────────────────────────────────────────"
echo -n "  核心代码文件: "
find src/remote_node -name "*.cpp" -o -name "*.hpp" | wc -l
echo -n "  代码行数: "
(find src/remote_node -name "*.cpp" -o -name "*.hpp" | xargs wc -l 2>/dev/null | tail -1 | awk '{print $1}') || echo "未计算"
echo -n "  文档文件: "
find . -maxdepth 1 -name "*REMOTE*" -o -name "*MIGRATION*" -o -name "*IMPLEMENTATION*" 2>/dev/null | wc -l
echo "───────────────────────────────────────────────────────────────────────────"
echo ""
echo "🎉 项目已完成！开始使用吧！"
