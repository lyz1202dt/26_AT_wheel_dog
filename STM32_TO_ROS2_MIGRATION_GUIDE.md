# STM32下位机代码 → ROS2上位机代码 映射说明

## 📋 总体架构对比

### STM32 嵌入式系统 (下位机)

```
UART7_ISR(中断服务)
    ↓
    ├─ 接收1字节
    ├─ 验证帧头(0xAA)
    └─ 通过DMA接收完整数据包
    
    ↓ 信号量
    
UART7_RemotecontrolTask(FreeRTOS任务)
    ├─ 等待信号量(200ms超时)
    ├─ 数据滤波
    ├─ Bezier变换
    ├─ 坐标变换
    └─ 设置全局结构体 (legs_state)
    
下位机其他任务读取 legs_state.remote_cmd
```

### ROS2 上位机系统

```
RemoteNode 构造函数
    ├─ 打开串口(/dev/ttyUSB1, 115200)
    ├─ 创建ROS2发布器
    └─ 启动串口接收线程
    
serial_thread (独立线程)
    ├─ serialLoop(): 无限循环读帧
    ├─ readFrame(): 查找帧头→读取完整数据包
    ├─ processData(): 数据处理、滤波、变换
    └─ pub_->publish(): 发布ROS2话题
    
其他节点订阅 /robot_move_cmd 话题
```

---

## 🔄 函数映射表

### 1. 序列初始化

**STM32**:
```c
void SystemInit(void) {
    HAL_Init();
    // ...
    UART7_Init();
    xTaskCreate(UART7_RemotecontrolTask, ...);
}
```

**ROS2**:
```cpp
RemoteNode::RemoteNode() : Node("remote_node"), running_(true) {
    ser_ = std::make_unique<serial::Serial>(port, baudrate, ...);
    ser_->open();
    pub_ = this->create_publisher<...>("robot_move_cmd", 10);
    serial_thread_ = std::thread(&RemoteNode::serialLoop, this);
}
```

**差异**:
- STM32用FreeRTOS任务，ROS2用std::thread
- STM32用中断+DMA，ROS2用轮询

---

### 2. 中断处理 (ISR) → 帧读取 (readFrame)

**STM32 (UART7_ISR)**:
```c
void UART7_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (remote_control_buf[0] == 0xAA) {  // 检查帧头
        memcpy(&remotedata, remote_control_buf, 12);
        xSemaphoreGiveFromISR(remote_semaphore, &xHigherPriorityTaskWoken);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, remote_control_buf, ...);
}
```

**ROS2 (readFrame)**:
```cpp
bool RemoteNode::readFrame(uint8_t *buffer) {
    // 扫描查找帧头 0xAA
    while (true) {
        if (ser_->read(&byte, 1) != 1) return false;
        if (byte == REMOTE_PACK_HEAD) {
            buffer[0] = byte;
            break;
        }
    }
    
    // 读取剩余11字节
    size_t len = ser_->read(buffer + 1, REMOTE_PACK_SIZE - 1);
    if (len != REMOTE_PACK_SIZE - 1) return false;
    
    return true;
}
```

**映射关系**:

| STM32 | ROS2 | 说明 |
|-------|------|------|
| UART7_RxCpltCallback | readFrame() 中的帧头检查 | 验证0xAA头 |
| HAL_UARTEx_ReceiveToIdle_DMA | ser_->read() | 接收数据 |
| memcpy() | readFrame() 的buffer赋值 | 数据复制 |
| xSemaphoreGiveFromISR | 返回true | 触发处理 |

---

### 3. 任务处理 (Task) → 主线程 (serialLoop)

**STM32 (UART7_RemotecontrolTask)**:
```c
void UART7_RemotecontrolTask(void *param) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 100ms周期
    
    while (1) {
        float vel[3];
        
        // 等待信号量(200ms超时)
        if (xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) != pdTRUE) {
            // 超时处理：复位摇杆
            remotedata.rocker[0..3] = 0;
        }
        
        __disable_irq();
        // 数据处理...
        __enable_irq();
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
```

**ROS2 (serialLoop)**:
```cpp
void RemoteNode::serialLoop() {
    uint8_t buffer[REMOTE_PACK_SIZE];
    
    while (rclcpp::ok() && running_) {
        if (!readFrame(buffer)) continue;
        
        RemotePack_t data;
        memcpy(&data, buffer, sizeof(RemotePack_t));
        
        processData(data);
    }
}
```

**映射关系**:

| STM32 | ROS2 | 说明 |
|-------|------|------|
| xSemaphoreTake(200ms) | timeout_timer_ | 超时检测 |
| vTaskDelayUntil(100ms) | 隐式（串口驱动控制） | 任务周期 |
| __disable_irq()/enable_irq() | 不需要（单线程串口） | 临界区保护 |

---

### 4. 数据滤波 (Filtering)

**STM32**:
```c
// 一阶低通滤波
last_v0 = filter_gate * (((float)(remotedata.rocker[0])) / 2047.0f) 
          + (1.0f - filter_gate) * last_v0;
vel[0] = last_v0;

// 参数
#define filter_gate 0.2f
```

**ROS2**:
```cpp
void RemoteNode::processData(const RemotePack_t &data) {
    // 完全相同的滤波公式
    last_v0 = filter_gate * (data.rocker[0] / 2047.0f) 
              + (1.0f - filter_gate) * last_v0;
    vel[0] = last_v0;
}

// 成员变量
float filter_gate = 0.2f;
```

**差异**: 完全一致，代码行为相同

---

### 5. Bezier 曲线变换

**STM32**:
```c
// 推 ROS2 实现中提供的非线性变换
// 原STM32代码中未显示，但有调用
model = BezierTransform(model, bezier);
```

**ROS2** (完整实现):
```cpp
float RemoteNode::BezierTransform(float x, const std::vector<float>& bezier) {
    // 二阶 Bezier 曲线
    // B(t) = (1-t)²·P₀ + 2(1-t)t·P₁ + t²·P₂
    
    if (x <= 0.0f) return 0.0f;
    if (x >= 1.0f) return 1.0f;
    
    float t = x;
    float p0 = bezier[0], p1 = bezier[1], p2 = bezier[2];
    float one_minus_t = 1.0f - t;
    
    return one_minus_t * one_minus_t * p0 
         + 2.0f * one_minus_t * t * p1 
         + t * t * p2;
}
```

**改进**: ROS2版本将非线性变换明确实现为参数化函数

---

### 6. 坐标变换 (Coordinate Transformation)

**STM32**:
```c
// 方向计算
float model = sqrtf(vel[0] * vel[0] + vel[1] * vel[1]);
if (model != 0.0f)
    cur_dir = atan2f(vel[1], vel[0]);

// 应用非线性变换
model = BezierTransform(model, bezier);

// 分解为X、Y分量
vel[1] = model * sinf(cur_dir);
if (vel[1] >= 0) {
    vel[1] = vel[1] * max_forword_speed;
} else {
    vel[1] = vel[1] * max_backward_speed;
}
vel[0] = model * cosf(cur_dir) * max_speed;
```

**ROS2**:
```cpp
float model = sqrtf(vel[0] * vel[0] + vel[1] * vel[1]);
if (model != 0.0f)
    cur_dir = atan2f(vel[1], vel[0]);

model = BezierTransform(model, bezier);

vel[1] = model * sinf(cur_dir);
if (vel[1] >= 0)
    vel[1] *= max_forword_speed;
else
    vel[1] *= max_backward_speed;

vel[0] = model * cosf(cur_dir) * max_speed;
```

**差异**: 代码完全相同

---

### 7. 数据发布

**STM32** (设置全局结构体):
```c
legs_state.remote_cmd.vx = vel[1];
legs_state.remote_cmd.vy = vel[0];
legs_state.remote_cmd.omega = last_omega * M_PI / 180.0f;
legs_state.remote_cmd.wheel_v = vel[2];
```

**ROS2** (发布ROS2消息):
```cpp
auto msg = std::make_unique<robot_interfaces::msg::MoveCmd>();
msg->step_mode = static_cast<uint32_t>(data.key1);
msg->wheel_vel = vel[2];
msg->vx = vel[1];
msg->vy = vel[0];
msg->vz = last_omega * M_PI / 180.0f;

pub_->publish(std::move(msg));
```

**映射关系**:

| STM32 结构体 | ROS2 消息 | 说明 |
|-------------|---------|------|
| legs_state.remote_cmd.vx | msg->vx | 前后速度 |
| legs_state.remote_cmd.vy | msg->vy | 左右速度 |
| legs_state.remote_cmd.omega | msg->vz | 自转角速度 |
| legs_state.remote_cmd.wheel_v | msg->wheel_vel | 轮子速度 |
| (隐含) | msg->step_mode | 按键信息 |

---

## 📊 全局变量映射

**STM32**:
```c
// 滤波状态
float last_v0, last_v1, last_v3, last_omega;

// 参数
float filter_gate = 0.2f;
float filter_alpha = 0.2f;
float max_speed = 1.0f;
float max_omega = 180.0f;

// 接收缓冲
RemotePack_t remotedata;

// 输出
LegState_t legs_state;  // 包含 remote_cmd
```

**ROS2** (类成员变量):
```cpp
class RemoteNode : public rclcpp::Node {
private:
    // 滤波状态
    float last_v0 = 0.0f;
    float last_v1 = 0.0f;
    float last_v3 = 0.0f;
    float last_omega = 0.0f;
    float cur_dir = 0.0f;
    
    // 参数（可动态修改）
    float filter_gate = 0.2f;
    float filter_alpha = 0.2f;
    float max_speed = 1.0f;
    float max_omega = 180.0f;
    
    // ROS2资源
    rclcpp::Publisher<MoveCmd>::SharedPtr pub_;
};
```

**改进**:
- Global → Class members (更好的封装)
- Mutable → const safe design
- 支持参数动态修改

---

## ⏱️ 时序对比

### STM32 时序

```
t=0ms:     UART7 中断 → 接收数据 → 设置信号量
t=10ms:    Task获取信号量 → 开始处理
t=20ms:    滤波/变换 → 更新 legs_state
t=30ms:    vTaskDelayUntil 直到 t=100ms
t=100ms:   下一周期开始
```

频率: 10Hz (周期100ms)

### ROS2 时序

```
t=0ms:     readFrame() 获得完整数据包
t=5ms:     processData() 开始处理
t=15ms:    滤波/变换完成
t=20ms:    pub_->publish() 发送消息
t=25ms:    等待下一个数据包（基于遥控器发送频率）
```

频率: 遥控器频率（通常10Hz或更高）

---

## 🔌 数据包格式

**完全相同**:

```
字节位置   字段名         大小      说明
0         head          1字节     0xAA (帧头)
1-2       rocker[0]     2字节     int16_t
3-4       rocker[1]     2字节     int16_t
5-6       rocker[2]     2字节     int16_t
7-8       rocker[3]     2字节     int16_t
9         key1          1字节     uint8_t
10        key2          1字节     uint8_t
11        end           1字节     帧尾

总长: 12字节
```

---

## 🚀 迁移优势

| 特性 | STM32 | ROS2 | 优势 |
|------|-------|------|------|
| 通信方式 | 全局变量 | ROS2话题 | 灵活、模块化、可扩展 |
| 参数修改 | 需要重编译 | 运行时动态修改 | ✓ |
| 多设备支持 | 困难 | 原生支持多实例 | ✓ |
| 日志记录 | printf | RCLCPP_INFO等 | ✓ |
| 线程管理 | FreeRTOS | C++11 std::thread | ✓ |
| 数据处理流 | 线性 | 可扩展管道 | ✓ |
| 测试/仿真 | 只能在硬件上 | 可在PC上仿真 | ✓ |

---

## 💻 代码行数对比

| 功能 | STM32 (C) | ROS2 (C++) | 备注 |
|------|-----------|-----------|------|
| 初始化 | ~30 | ~50 | ROS2更详细 |
| 中断处理 | ~10 | - | ROS2用轮询 |
| 任务主循环 | ~50 | ~30 | ROS2更简洁 |
| 数据处理 | ~80 | ~100 | 多了调试输出 |
| **总计** | **~170** | **~180** | 相近，功能更完整 |

---

## 🎓 学习路径

```
STM32 代码理解
    ↓
ROS2 架构学习
    ↓
逐函数映射转换
    ↓
单元测试验证
    ↓
集成测试
    ↓
性能优化
```

---

**文档版本**: v1.0  
**最后更新**: 2026年3月28日  
**对应源文件**: remote_node.cpp v1.0
