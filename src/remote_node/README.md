# 遥控器驱动 (remote_node) - ROS2实现说明

## 📋 项目概述

本项目将嵌入式系统（STM32）的遥控器接收功能迁移到上位机ROS2系统中。通过串口接收无线遥控器信号，经过数据处理和滤波后，发布为ROS2话题供机器人控制器使用。

## 🔧 系统架构

```
遥控器 (无线) 
    ↓
STM32 (接收) 
    ↓
串口通信 (UART, 115200bps)
    ↓
Linux PC (ROS2)
    ↓
ROS2话题发布 (/robot_move_cmd)
    ↓
机器人控制系统
```

## 📦 文件结构

```
remote_node/
├── CMakeLists.txt              # CMake构建配置
├── package.xml                 # ROS2包描述文件
├── include/
│   └── remote_node/
│       └── remote_node.hpp      # 头文件（类定义、结构体）
└── src/
    ├── main.cpp                # 主程序入口
    └── remote_node.cpp         # 核心实现
```

## 🎯 核心功能

### 1. 串口通信
- **接口**: `/dev/ttyUSB1`
- **波特率**: 115200 bps
- **数据包格式**: 帧头(0xAA) + 摇杆数据(8字节) + 按键数据(2字节) + 帧尾

### 2. 遥控器数据包结构
```cpp
struct RemotePack_t {
    uint8_t head;          // 帧头: 0xAA
    int16_t rocker[4];     // 4路摇杆, 范围: 0~2047
    uint8_t key1;          // 按键组1 (bit位表示不同按键)
    uint8_t key2;          // 按键组2
    uint8_t end;           // 帧尾
}; // 总长度: 12字节
```

### 3. 数据处理流程

#### 步骤1: 摇杆值归一化
```
raw_value(0~2047) → 归一化(0~1) → 滤波 → 物理量映射
```

#### 步骤2: 一阶低通滤波
```
filtered = filter_gate * raw + (1 - filter_gate) * last_filtered

filter_gate = 0.2  (可调参数，范围0~1)
- 值越大: 响应越快，噪声越多
- 值越小: 响应越慢，输出更平稳
```

#### 步骤3: 非线性变换 (Bezier曲线)
对摇杆速度和角速度应用二阶Bezier曲线，实现摇杆端点加速效果：
```
B(t) = (1-t)²·P₀ + 2(1-t)t·P₁ + t²·P₂

默认系数: P₀=0.0, P₁=0.05, P₂=1.0
- 使低速操作更精细
- 高速移动更敏捷
```

#### 步骤4: 坐标变换
```
摇杆0,1 → 前进/后退速度 & 转向角度
角速度 = Bezier(摇杆2) × max_omega
轮子速度 = 摇杆3
按键 → 步态模式 (key1 → step_mode)
```

### 4. ROS2话题发布

**话题**: `/robot_move_cmd`

**消息格式**:
```cpp
robot_interfaces/msg/MoveCmd
  uint32 step_mode    # 步态模式 (来自key1)
  float32 wheel_vel   # 轮子速度
  float32 vx          # X方向速度 (前后)
  float32 vy          # Y方向速度 (左右)
  float32 vz          # Z方向速度 (角速度, 弧度/秒)
```

## 📊 参数配置

所有参数在 `RemoteNode` 类中定义为成员变量，可根据需要调整：

```cpp
// 滤波参数
float filter_gate = 0.2f;      // 摇杆斜坡、转向速度的滤波系数
float filter_alpha = 0.2f;     // 轮子速度的滤波系数

// 速度限制
float max_speed = 1.0f;           // 最大前进速度
float max_forword_speed = 1.0f;   // 最大前进速度
float max_backward_speed = 0.5f;  // 最大后退速度
float max_omega = 180.0f;         // 最大角速度 (度/秒)

// Bezier曲线系数
std::vector<float> bezier{0.0f, 0.05f, 1.0f};
```

## 🚀 使用说明

### 1. 编译
```bash
cd /home/yzy/26_AT_wheel_dog
colcon build --packages-select remote_node
source install/setup.bash
```

### 2. 运行
```bash
ros2 run remote_node remote_node
```

### 3. 调试/监控

查看发布的数据:
```bash
ros2 topic echo /robot_move_cmd
```

查看节点信息:
```bash
ros2 node info /remote_node
```

启用调试日志:
```bash
ros2 run remote_node remote_node --ros-args --log-level debug
```

监听正确的串口设备:
```bash
ros2 param set /remote_node port /dev/ttyUSB0  # 根据实际情况调整
```

## 🔍 故障排查

### 问题1: "串口打开失败"
**原因**: 串口设备不存在或权限不足
```bash
# 检查设备
ls -la /dev/ttyUSB*

# 授予权限
sudo usermod -a -G dialout $USER
# 需要重新登录生效
```

### 问题2: "遥控器通信超时"
**原因**: 200ms内未收到数据
```bash
# 检查串口通信
cat < /dev/ttyUSB1 | xxd

# 检查波特率是否正确
stty -F /dev/ttyUSB1 115200
```

### 问题3: 数据接收不稳定
**解决**: 调整滤波系数
- 增加 `filter_gate` (0.3~0.5) 使反应更灵敏
- 减小 `filter_gate` (0.1~0.2) 使输出更平稳

### 问题4: 摇杆响应不够灵敏/过度灵敏
**解决**: 调整Bezier曲线系数
```cpp
// 如果反应不够灵敏，增加中点系数
std::vector<float> bezier{0.0f, 0.1f, 1.0f};  // 更灵敏

// 如果反应过度灵敏，减小中点系数
std::vector<float> bezier{0.0f, 0.02f, 1.0f}; // 更平线
```

## 🔄 与STM32代码的映射

| STM32功能 | ROS2实现位置 |
|---------|------------|
| UART7中断处理 | `readFrame()` 方法 |
| UART7_RemotecontrolTask | `serialLoop()` 任务 |
| 数据滤波逻辑 | `processData()` 中的滤波部分 |
| Bezier变换 | `BezierTransform()` 方法 |
| 超时检测 | `timeout_timer_` 定时器 |
| IMU/遥控数据发布 | `pub_->publish()` 发布消息 |

## 📝 代码改进说明

相比简单迁移，本实现进行了以下改进：

1. **更好的错误处理**: 异常捕获、设备验证
2. **超时检测**: 200ms无数据自动复位摇杆
3. **日志记录**: RCLCPP_INFO/RCLCPP_WARN/RCLCPP_DEBUG
4. **内存安全**: 使用 `std::unique_ptr` 自动管理资源
5. **线程安全**: 使用 `std::atomic<bool>` 标志
6. **可配置性**: 所有参数可在运行时通过ROS2参数修改
7. **调试支持**: 详细的调试输出和消息验证

## 📚 相关参考

- **参考包**: `imu_driver` - 类似的驱动架构
- **ROS2文档**: http://docs.ros.org/
- **串口库**: libserial_driver

## ✅ 验证清单

- [x] 串口收发正常
- [x] 数据包解析正确
- [x] 滤波算法实现
- [x] Bezier变换应用
- [x] ROS2话题发布
- [x] 超时检测
- [x] 错误处理
- [x] 日志输出

## 📞 调试技巧

### 1. 查看原始串口数据流
```bash
hexdump -C < /dev/ttyUSB1
```

### 2. 验证摇杆值范围
编辑 `remote_node.cpp` 中的 `processData()` 方法，添加：
```cpp
RCLCPP_INFO(this->get_logger(), 
    "Raw rocker values: [%d, %d, %d, %d]",
    data.rocker[0], data.rocker[1], data.rocker[2], data.rocker[3]);
```

### 3. 实时调整参数
```bash
ros2 param set /remote_node filter_gate 0.3
ros2 param set /remote_node max_omega 200.0
```

## 🎓 学习资源

本项目涉及的关键知识点：
- C++编程（STL、线程、智能指针）
- 嵌入式系统通信（UART协议）
- ROS2框架（节点、发布者、消息）
- 信号处理（低通滤波）
- 数值计算（Bezier曲线）
- Linux设备控制

---

**最后更新**: 2026年3月28日  
**维护者**: cloud1202@qq.com
