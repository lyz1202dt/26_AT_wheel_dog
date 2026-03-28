#include "remote_node/remote_node.hpp"
#include <cstring>
#include <cmath>

// 常数定义
static constexpr size_t REMOTE_PACK_SIZE = sizeof(RemotePack_t);
static constexpr uint8_t REMOTE_PACK_HEAD = 0xAA;

RemoteNode::RemoteNode()
    : Node("remote_node"), running_(true)
{
    std::string port = "/dev/ttyUSB1";
    int baudrate = 115200;

    // 创建序列对象
    try
    {
        ser_ = std::make_unique<serial::Serial>(port, baudrate, serial::Timeout::simpleTimeout(50));
        
        if (!ser_->isOpen())
        {
            ser_->open();
        }
        RCLCPP_INFO(this->get_logger(), "成功打开遥控器串口: %s, 波特率: %d", port.c_str(), baudrate);
    }
    catch (const serial::SerialException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "遥控器串口打开失败: %s", e.what());
        ser_ = nullptr;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "遥控器串口初始化异常: %s", e.what());
        ser_ = nullptr;
    }

    // 创建ROS2发布器
    move_cmd_pub = this->create_publisher<robot_interfaces::msg::MoveCmd>("robot_move_cmd", 10);
    RCLCPP_INFO(this->get_logger(), "RemoteNode 初始化完成，发布话题: robot_move_cmd");

    // 记录初始时间
    last_receive_time_ = this->now();

    // 创建超时检测定时器（100ms检测一次）
    timeout_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            auto time_diff = (this->now() - last_receive_time_).seconds() * 1000.0f;  // 转换为毫秒
            if (time_diff > TIMEOUT_MS)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                    "遥控器通信超时 (%.0fms)，摇杆已复位", time_diff);
            }
        }
    );

    // 仅在串口打开成功时启动接收线程
    if (ser_ && ser_->isOpen())
    {
        serial_thread_ = std::thread(&RemoteNode::serialLoop, this);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "串口未打开，接收线程未启动");
        running_ = false;
    }
}

RemoteNode::~RemoteNode()
{
    running_ = false;
    if (serial_thread_.joinable())
        serial_thread_.join();

    if (ser_ && ser_->isOpen())
        ser_->close();

    RCLCPP_INFO(this->get_logger(), "RemoteNode 已关闭");
}

// ==================== 串口线程 ====================
void RemoteNode::serialLoop()
{
    uint8_t buffer[REMOTE_PACK_SIZE];

    while (rclcpp::ok() && running_)
    {
        try
        {
            if (!ser_ || !ser_->isOpen())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            if (!readFrame(buffer))
                continue;

            RemotePack_t data;
            memcpy(&data, buffer, sizeof(RemotePack_t));
            
            processData(data);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "串口线程异常: %s", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "串口线程发生未知异常");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

// ==================== 帧读取 ====================
bool RemoteNode::readFrame(uint8_t *buffer)
{
    if (!ser_ || !ser_->isOpen())
        return false;

    try
    {
        uint8_t byte;

        // 扫描寻找帧头 0xAA
        while (true)
        {
            if (ser_->available() < 1)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                if (!running_)
                    return false;
                continue;
            }

            size_t bytes_read = ser_->read(&byte, 1);
            if (bytes_read != 1)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            if (byte == REMOTE_PACK_HEAD)
            {
                buffer[0] = byte;
                break;
            }
        }

        // 读取剩余字节
        size_t remaining = REMOTE_PACK_SIZE - 1;
        size_t bytes_read = 0;

        while (bytes_read < remaining)
        {
            if (!ser_->available())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                if (!running_)
                    return false;
                continue;
            }

            size_t len = ser_->read(buffer + 1 + bytes_read, remaining - bytes_read);
            if (len == 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            bytes_read += len;
        }

        // 验证帧尾
        if (buffer[REMOTE_PACK_SIZE - 1] != 0x00)  // 根据需要调整帧尾标识
        {
            RCLCPP_DEBUG(this->get_logger(), "无效的帧尾: 0x%02x", buffer[REMOTE_PACK_SIZE - 1]);
            return false;
        }

        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "读取帧异常: %s", e.what());
        return false;
    }
}

// ==================== Bezier曲线变换 ====================
float RemoteNode::BezierTransform(float x, const std::vector<float>& bezier)
{
    // 二阶 Bezier 曲线: B(t) = (1-t)²*P0 + 2*(1-t)*t*P1 + t²*P2
    // 其中输入 x 作为参数 t，范围 [0, 1]
    if (x <= 0.0f) return 0.0f;
    if (x >= 1.0f) return 1.0f;

    if (bezier.size() < 3)
        return x;  // 如果没有提供足够的系数，返回线性结果

    float t = x;
    float p0 = bezier[0];
    float p1 = bezier[1];
    float p2 = bezier[2];

    float one_minus_t = 1.0f - t;
    float result = one_minus_t * one_minus_t * p0 + 
                   2.0f * one_minus_t * t * p1 + 
                   t * t * p2;

    return result;
}

// ==================== 数据处理 ====================
void RemoteNode::processData(const RemotePack_t &data)
{
    // 更新最后接收时间
    last_receive_time_ = this->now();

    float vel[3];

    // ====== 摇杆值归一化 & 滤波（对应STM32代码）======
    // 摇杆0：X轴速度
    last_v0 = filter_gate * (data.rocker[0] / 2047.0f) + (1.0f - filter_gate) * last_v0;
    vel[0] = last_v0;

    // 摇杆1：Y轴速度
    last_v1 = filter_gate * (data.rocker[1] / 2047.0f) + (1.0f - filter_gate) * last_v1;
    vel[1] = last_v1;

    // 摇杆2：角速度（使用Bezier变换）
    float raw_omega = data.rocker[2] / 2047.0f;
    last_omega = (BezierTransform(raw_omega, bezier) * max_omega) * filter_gate + 
                 (1.0f - filter_gate) * last_omega;

    // 摇杆3：轮子速度
    float rocker_val = data.rocker[3] / 2047.0f;
    last_v3 = filter_alpha * rocker_val + (1.0f - filter_alpha) * last_v3;
    vel[2] = last_v3;

    // ====== 方向计算与坐标变换 ======
    float model = sqrtf(vel[0] * vel[0] + vel[1] * vel[1]);

    if (model != 0.0f)
        cur_dir = atan2f(vel[1], vel[0]);

    // 应用Bezier变换到速度模
    model = BezierTransform(model, bezier);

    // 计算旋转后的速度分量
    vel[1] = model * sinf(cur_dir);

    if (vel[1] >= 0)
        vel[1] *= max_forword_speed;
    else
        vel[1] *= max_backward_speed;

    vel[0] = model * cosf(cur_dir) * max_speed;

    // ====== 按键处理 ======
    key1 = data.key1;

    // ====== 构建并发布消息 ======
    auto msg = std::make_unique<robot_interfaces::msg::MoveCmd>();

    msg->step_mode = static_cast<uint32_t>(data.key1);  // 按键1 → 步态模式
    msg->wheel_vel = vel[2];                             // 摇杆4 → 轮子速度
    msg->vx = vel[1];                                    // Y方向速度
    msg->vy = vel[0];                                    // X方向速度
    msg->vz = last_omega * M_PI / 180.0f;               // 将角速度从度转换为弧度

    move_cmd_pub->publish(std::move(msg));

    // 调试输出（可以使用--ros-args --log-level debug 启用）
    RCLCPP_DEBUG(this->get_logger(), 
        "Remote: rocker[0]=%f, rocker[1]=%f, omega=%.2f°/s, wheel_v=%.2f, key1=0x%02x",
        vel[0], vel[1], last_omega, vel[2], data.key1);
}
