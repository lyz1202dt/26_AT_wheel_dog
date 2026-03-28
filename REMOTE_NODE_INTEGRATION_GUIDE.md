# 🤖 远程遥控器驱动 - 完整集成指南

## 📌 快速开始

### 1️⃣ 编译
```bash
cd /home/yzy/26_AT_wheel_dog
colcon build --packages-select remote_node
source install/setup.bash
```

### 2️⃣ 运行
```bash
# 方式1: 直接运行
ros2 run remote_node remote_node

# 方式2: 使用launch文件（推荐）
ros2 launch remote_node remote_node.launch.py

# 方式3: 使用自定义串口
ros2 run remote_node remote_node --ros-args -p port:=/dev/ttyUSB0
```

### 3️⃣ 验证
```bash
# 在另一个终端查看数据
ros2 topic echo /robot_move_cmd
```

---

## 📚 详细说明

### 📦 文件组织

```
remote_node/
├── src/
│   ├── main.cpp                 # 主程序入口
│   └── remote_node.cpp          # 核心实现
├── include/remote_node/
│   └── remote_node.hpp          # 类和结构体定义
├── config/
│   └── remote_config.yaml       # 配置参数（可扩展）
├── launch/
│   └── remote_node.launch.py    # ROS2启动脚本
├── scripts/
│   └── validate_remote_data.py  # 数据验证工具
├── CMakeLists.txt              # 构建配置
└── package.xml                 # 包描述
```

### 📊 数据流图

```
┌─────────────────┐
│  遥控器信号     │ (2.4GHz 无线)
└────────┬────────┘
         │
    ┌────▼────┐
    │  STM32  │ (接收器)
    └────┬────┘
         │
    ┌────▼──────────────────┐
    │ UART (115200 bps)     │ 12字节数据包
    └────┬──────────────────┘
         │
┌────────▼─────────────────┐
│ Linux PC (ROS2驱动)      │
│                          │
│ 1. 串口接收              │
│ 2. 数据验证              │
│ 3. 一阶低通滤波          │
│ 4. Bezier曲线变换        │
│ 5. 坐标系变换            │
│ 6. ROS2话题发布          │
└────────┬─────────────────┘
         │
┌────────▼──────────────┐
│ ROS2话题              │
│ /robot_move_cmd       │
│ MoveCmd消息           │
└───────────────────────┘
         │
    ┌────▼────────────┐
    │ 机器人控制器   │
    │ 最终执行动作   │
    └────────────────┘
```

### 🔌 ROS2集成

#### 发布话题

**话题名**: `/robot_move_cmd`

**消息类型**: `robot_interfaces/msg/MoveCmd`

**发布频率**: ~10Hz (取决于遥控器发送频率)

**字段说明**:

| 字段 | 类型 | 范围 | 含义 |
|------|------|------|------|
| `step_mode` | uint32 | 0-255 | 步态模式（来自按键1） |
| `wheel_vel` | float32 | -1.0~1.0 | 轮子速度 |
| `vx` | float32 | -1.0~1.0 | 前后速度 (m/s 或相对值) |
| `vy` | float32 | -1.0~1.0 | 左右速度 (m/s 或相对值) |
| `vz` | float32 | -π~π | 自转角速度 (rad/s) |

#### 订阅示例

```cpp
// 在另一个ROS2节点中订阅
auto sub = node_->create_subscription<robot_interfaces::msg::MoveCmd>(
    "robot_move_cmd", 
    10,
    [this](const robot_interfaces::msg::MoveCmd& msg) {
        // 处理接收到的遥控指令
        RCLCPP_INFO(this->get_logger(), 
            "收到指令: vx=%.2f, vy=%.2f, vz=%.2f", 
            msg.vx, msg.vy, msg.vz);
    }
);
```

---

## ⚙️ 参数设置

### 方式1: 在代码中修改（编译时）

编辑 [include/remote_node/remote_node.hpp]

```cpp
float filter_gate = 0.2f;          // 修改这里
float max_omega = 180.0f;          // 或这里
```

然后重新编译。

### 方式2: 运行时参数设置

节点支持ROS2参数系统，可在运行时修改：

```bash
# 查看所有参数
ros2 param list /remote_node

# 获取参数值
ros2 param get /remote_node filter_gate

# 设置参数值
ros2 param set /remote_node filter_gate 0.3
ros2 param set /remote_node max_omega 200.0
```

**注意**: 当前版本参数设置后需要重启节点生效。

---

## 🔍 调试技巧

### 启用调试日志

```bash
ros2 run remote_node remote_node --ros-args --log-level debug
```

输出示例:
```
[remote_node]: Remote: rocker[0]=0.123, rocker[1]=-0.456, omega=45.23°/s, wheel_v=0.00, key1=0x01
[remote_node]: Remote: rocker[0]=0.125, rocker[1]=-0.450, omega=45.30°/s, wheel_v=0.00, key1=0x01
```

### 海龟图形化监控 🐢

```bash
ros2 topic hz /robot_move_cmd          # 话题频率
ros2 topic bw /robot_move_cmd          # 数据带宽
ros2 topic type /robot_move_cmd        # 消息类型
```

### 串口层面调试

```bash
# 查看原始串口数据
sudo hexdump -C < /dev/ttyUSB1

# 设置串口波特率（验证）
stty -F /dev/ttyUSB1 115200 cs8 -cstopb -parity

# 测试一帧数据
dd if=/dev/ttyUSB1 bs=12 count=1 | hexdump -C
```

### 数据验证工具

```bash
# 终端1: 启动节点
ros2 run remote_node remote_node

# 终端2: 运行验证器
python3 src/remote_node/scripts/validate_remote_data.py
```

输出示例:
```
消息频率: 10.2 Hz
数据更新 | vx= 0.123 vy= 0.456 vz= 1.571 wheel= 0.000 mode=0x01
```

---

## 🛠️ 故障排除

### ❌ "串口打开失败"

**原因**: 
- 串口不存在
- 权限不足
- 串口被占用

**解决**:
```bash
# 1. 检查设备
ls -la /dev/ttyUSB*

# 2. 授予权限
sudo usermod -a -G dialout $USER
# 重新登录或运行:
newgrp dialout

# 3. 检查是否被占用
lsof | grep ttyUSB
ps aux | grep ttyUSB
```

### ⏱️ "遥控器通信超时"

**原因**: 200ms内未收到数据

**解决**:
```bash
# 1. 验证遥控器是否工作
cat < /dev/ttyUSB1 | xxd  # 应该看到连续的0xAA开头的数据

# 2. 检查波特率
stty -F /dev/ttyUSB1 115200

# 3. 增加超时延迟
# 编辑 include/remote_node/remote_node.hpp
static constexpr int TIMEOUT_MS = 500;  // 改为500ms
```

### 📊 "数据获取不稳定或噪声"

**原因**: 滤波系数不合适

**解决**:
```bash
# 增加滤波强度
ros2 param set /remote_node filter_gate 0.1

# 或修改源代码中的系数
float filter_gate = 0.1f;  // 减小值→更平稳，但响应变慢
```

### 🎮 "摇杆响应不够灵敏"

**原因**: Bezier曲线或最大速度设置不当

**解决**:
```cpp
// 方案1: 调整Bezier中点(增加指数)
std::vector<float> bezier{0.0f, 0.15f, 1.0f};  // 更灵敏

// 方案2: 增加最大速度
float max_speed = 2.0f;  // 原来1.0
float max_omega = 360.0f; // 原来180.0
```

### 🔄 "坐标系方向错误"

如果前进速度显示为后退，或左转显示为右转：

**解决** - 编辑 [src/remote_node.cpp] 的 `processData()` 方法：

```cpp
// 原版
msg->vx = vel[1];    // 前后
msg->vy = vel[0];    // 左右
msg->vz = last_omega * M_PI / 180.0f;

// 修正方案：取反某个轴
msg->vx = -vel[1];   // 反转前后
msg->vy = vel[0];
msg->vz = -last_omega * M_PI / 180.0f;  // 反转自转
```

---

## 💡 性能优化建议

1. **使用launch文件**: 避免重复输入参数
2. **预配置YAML**: 针对不同场景存储配置
3. **管理日志级别**: INFO级别适合生产环境，DEBUG用于调试
4. **调整队列深度**: 根据控制频率调整话题队列大小

---

## 📖 参考文档

### 相关ROS2概念
- [ROS2发布者和订阅者](http://docs.ros.org/en/foxy/Concepts.html)
- [ROS2参数系统](http://docs.ros.org/en/foxy/Concepts/About-Parameters.html)
- [launch文件使用](http://docs.ros.org/en/foxy/Tutorials/Launching-multiple-nodes.html)

### 相关包依赖
- **serial**: 串口通信库
- **rclcpp**: ROS2 C++客户端库
- **robot_interfaces**: 自定义消息定义

---

## 🎯 常见集成场景

### 场景1: 与move_control集成

```cpp
// 在 move_control 节点中订阅遥控数据
auto move_cmd_sub = node_->create_subscription<robot_interfaces::msg::MoveCmd>(
    "robot_move_cmd", 
    10,
    [this](const robot_interfaces::msg::MoveCmd& msg) {
        // 根据遥控指令控制机器人
        this->applyMotionCommand(msg);
    }
);
```

### 场景2: 与dog_controller集成

```cpp
// dog_controller 订阅遥控命令并执行步态
if (msg.step_mode & 0x01) {
    leg_controller->setGaitMode(GaitMode::WALK);
} else if (msg.step_mode & 0x02) {
    leg_controller->setGaitMode(GaitMode::TROT);
}

// 应用速度指令
leg_controller->setVelocity(msg.vx, msg.vy, msg.vz);
```

### 场景3: 多遥控器支持（进阶）

可在remote_node中添加多个串口接口，或创建多个实例：

```bash
ros2 run remote_node remote_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p namespace:=remote1

ros2 run remote_node remote_node --ros-args \
  -p port:=/dev/ttyUSB1 \
  -p namespace:=remote2
```

---

## ✅ 验收清单

- [ ] 串口连接正常
- [ ] 数据包周期性接收
- [ ] 摇杆输入响应正确
- [ ] 滤波效果符合预期
- [ ] ROS2话题成功发布
- [ ] 下游节点成功订阅
- [ ] 机器人响应遥控指令
- [ ] 无串口超时警告
- [ ] 日志输出无误

---

**最后修改日期**: 2026年3月28日  
**作者**: AI Assistant  
**维护者**: cloud1202@qq.com
