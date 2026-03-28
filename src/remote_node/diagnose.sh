#!/bin/bash
# 遥控器驱动诊断脚本

#echo "======================================"
#echo "遥控器驱动诊断工具"
#echo "======================================"
#echo ""

# 检查串口设备
#echo "1. 检查串口设备:"
#ls -la /dev/ttyUSB* 2>/dev/null || echo "   ❌ 没有 USB 串口设备"
#ls -la /dev/ttyACM* 2>/dev/null || echo "   (没有 ACM 设备)"
#echo ""

# 检查用户权限
#echo "2. 检查用户权限:"
#if groups | grep -q dialout; then
 #   echo "   ✓ 用户在 dialout 组中"
#else
#    echo "   ❌ 用户不在 dialout 组，需要运行: sudo usermod -a -G dialout \$USER"
#fi
#echo ""

# 显示系统中的所有串口
#echo "3. 系统中的所有串口设备:"
#ls -la /dev/tty* 2>/dev/null | grep -E "USB|ACM" || echo "   (没有 USB/ACM 设备)"
#echo ""

# 运行程序并监听日志
#echo "4. 启动遥控器节点（日志输出）："
#echo "   命令: ros2 run remote_node remote_node --ros-args --log-level debug"
#echo "   按 Ctrl+C 停止"
#echo ""
#echo "确认你已经:"
#echo "   1. 连接遥控器接收器到 USB 端口"
#echo "   2. 打开遥控器电源"
#echo "   3. 确保遥控器与接收器配对成功"
#echo ""
#echo "======================================"
#echo ""

# 启动节点
#source install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
#ros2 run remote_node remote_node --ros-args --log-level debug
