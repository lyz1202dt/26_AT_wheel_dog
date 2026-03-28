#include "remote_node/remote_node.hpp"
#include <cstring>
#include <cmath>
#include <rclcpp/logging.hpp>

// ==================== 构造函数 ====================
RemoteNode::RemoteNode()
    : Node("remote_node")
{
    std::string port = "/dev/ttyUSB0";
    int baudrate = 115200;

    // 创建通信模块
    comm_ = std::make_shared<RemoteComm>();
    
    // 初始化通信模块
    auto error_cb = [this](uint32_t type)
    {
        std::string error_msg;
        switch (type)
        {
            case COMM_BAD_HEAD: error_msg = "数据包头错误"; break;
            case COMM_BAD_SUM:  error_msg = "校验和错误"; break;
            case COMM_BAD_LEN:  error_msg = "数据包长度错误"; break;
            case COMM_BAD_ACK:  error_msg = "应答包错误"; break;
            default:            error_msg = "未知错误"; break;
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "遥控器通信错误: %s", error_msg.c_str());
    };

    if (!comm_->Init(port, baudrate, error_cb))
    {
        RCLCPP_ERROR(this->get_logger(), 
            "遥控器通信初始化失败 %s:%d\n"
            "故障排查:\n"
            "  1. 确认串口设备 %s 存在: ls -la %s\n"
            "  2. 确认用户有权限: groups | grep dialout\n"
            "  3. 波特率 %d bps 与遥控器一致",
                    port.c_str(), baudrate, 
                    port.c_str(), port.c_str(),
                    baudrate);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "遥控器通信已初始化 %s:%d", 
                   port.c_str(), baudrate);
    }

    // 创建ROS2发布器
    move_cmd_pub = this->create_publisher<robot_interfaces::msg::MoveCmd>("robot_move_cmd", 10);
    RCLCPP_INFO(this->get_logger(), "遥控器节点已初始化，发布'robot_move_cmd'话题");

    // 记录初始时间
    last_receive_time_ = this->now();

    // 注册数据接收回调（命令字 1 对应遥控器数据）
    recv_cb_id_ = comm_->RegisterRecvCb(
        [this](uint8_t *src, uint16_t size, void* user_data)
        {
            if (size >= sizeof(RemoteData_t))
            {
                RemoteData_t data;
                memcpy(&data, src, sizeof(RemoteData_t));
                this->ProcessData(data);
                RCLCPP_INFO(this->get_logger(), "接收到遥控器数据包，已处理");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), 
                    "数据包大小不足: 收到=%u字节, 期望=%zu字节\n"
                    "故障排查:\n"
                    "  1. 检查遥控器发送的数据包格式\n"
                    "  2. 确认命令字 (0x01) 是否正确\n"
                    "  3. 查看原始数据 (十六进制):",
                    size, sizeof(RemoteData_t));
                    
                // 打印前20字节数据用于诊断
                if (size > 0)
                {
                    std::string hex_str;
                    for (uint16_t i = 0; i < size && i < 20; i++)
                    {
                        char buf[4];
                        snprintf(buf, sizeof(buf), "%02x ", src[i]);
                        hex_str += buf;
                    }
                    RCLCPP_WARN(this->get_logger(), "原始数据: %s", hex_str.c_str());
                }
            }
        },
        1,  // 命令字 1
        nullptr
    );

    // 现在启动接收线程（在回调注册后）
    comm_->StartReceiver();

    // 创建超时检测定时器（100ms检测一次）
    timeout_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]()
        {
            auto time_diff = (this->now() - last_receive_time_).seconds() * 1000.0f;
            if (time_diff > TIMEOUT_MS)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                    "遥控器通信超时 (%.0fms)，摇杆已复位\n"
                    "故障排查:\n"
                    "  1. 遥控器设备是否已连接\n"
                    "  2. 遥控器是否通电\n"
                    "  3. 接收器是否与遥控器配对\n"
                    "  4. 数据包命令字是否为 0x01\n"
                    "  5. 使用 'ros2 run remote_node remote_node --ros-args --log-level debug' 查看详细日志", 
                    time_diff);
            }
        }
    );
}

// ==================== 析构函数 ====================
RemoteNode::~RemoteNode()
{
    if (comm_)
    {
        comm_->UnregisterRecvCb(recv_cb_id_);
        comm_->Stop();
    }
    RCLCPP_INFO(this->get_logger(), "遥控器节点已关闭");
}

// ==================== 错误处理回调（未在此版本使用） ====================
void RemoteNode::OnBadDataPack(uint32_t type)
{
    std::string error_msg;
    switch (type)
    {
        case COMM_BAD_HEAD: error_msg = "数据包头错误"; break;
        case COMM_BAD_SUM:  error_msg = "校验和错误"; break;
        case COMM_BAD_LEN:  error_msg = "数据包长度错误"; break;
        case COMM_BAD_ACK:  error_msg = "应答包错误"; break;
        default:            error_msg = "未知错误"; break;
    }
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
        "遥控器通信错误: %s", error_msg.c_str());
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
void RemoteNode::ProcessData(const RemoteData_t &data)
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
        "遥控器数据: 摇杆0=%.3f, 摇杆1=%.3f, 角速度=%.1f°/s, 轮速=%.3f, 按键=0x%02x",
        vel[0], vel[1], last_omega, vel[2], data.key1);
}

