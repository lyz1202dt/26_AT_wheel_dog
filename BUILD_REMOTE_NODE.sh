#!/bin/bash

# 遥控器驱动编译指南

echo "============================================"
echo "remote_node 遥控器驱动 - 编译说明"
echo "============================================"
echo ""

# 进入工作区
cd /home/yzy/26_AT_wheel_dog

echo "1. 清理之前的编译..."
rm -rf build/remote_node install/remote_node log/build_*remote* 2>/dev/null

echo "2. 开始编译 remote_node..."
colcon build --packages-select remote_node --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ 编译成功！"
    echo ""
    echo "3. 加载环境..."
    source install/setup.bash
    
    echo ""
    echo "============================================"
    echo "运行遥控器节点"
    echo "============================================"
    echo "命令: ros2 run remote_node remote_node"
    echo ""
    echo "查看发布消息: ros2 topic echo /robot_move_cmd"
    echo "查看节点信息: ros2 node info /remote_node"
    echo ""
else
    echo "✗ 编译失败！"
    exit 1
fi
