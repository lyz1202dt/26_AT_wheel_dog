#include "remote_node/remote_node.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <sstream>
#include <iomanip>

RemoteNode::RemoteNode()
    : Node("remote_node"), remote_control_cb_id_(0)
{
    std::string port = "/dev/ttyUSB0";
    int baudrate = 115200;
    
    // 打开串口
    serial_ = std::make_unique<serial::Serial>(port, baudrate, serial::Timeout::simpleTimeout(100));
    if (!serial_->isOpen()) {
        RCLCPP_ERROR(this->get_logger(), "打开设备失败:%s", port.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "成功打开设备:%s,波特率为%d", port.c_str(), baudrate);
    
    // 创建ROS2发布器
    move_cmd_pub = this->create_publisher<robot_interfaces::msg::MoveCmd>(
            "robot_move_cmd", 10);
    
    RCLCPP_INFO(this->get_logger(), "遥控器数据发布器已创建");
    
    // 初始化通信协议处理器
    remote_comm_ = std::make_unique<RemoteComm>();
    remote_comm_->init([this](uint32_t error_type) {
        on_bad_packet(error_type);
    });

    // 注册遥控器数据接收回调
    remote_control_cb_id_ = remote_comm_->register_recv_cb(
        [this](const uint8_t* data, uint16_t size, void* user_data) {
            on_remote_control_data(data, size, user_data);
        },
        CMD_REMOTE_CONTROL,
        this
    );

    // 启动串口接收线程
    serial_recv_thread_ = std::make_unique<std::thread>([this]() { serial_recv_task(); });
}

RemoteNode::~RemoteNode()
{
    if (remote_comm_) {
        remote_comm_->unregister_recv_cb(remote_control_cb_id_);
    }
    
    if (serial_ && serial_->isOpen())
        serial_->close();
    
    if (serial_recv_thread_) {
        if (serial_recv_thread_->joinable())
            serial_recv_thread_->join();
    }
}

void RemoteNode::serial_recv_task()
{
    int error_count = 0;
    const int MAX_ERRORS = 5;
    
    while (rclcpp::ok()) {
        if (!serial_ || !serial_->isOpen()) {
            break;
        }

        try {
            uint8_t byte;
            size_t bytes_read = serial_->read(&byte, 1);
            if (bytes_read > 0) {
                // 重置错误计数
                error_count = 0;
                // 将字节传入通信协议处理器
                remote_comm_->process_recv_byte(byte);
            }
        } catch (const std::exception& e) {
            error_count++;
            if (error_count <= 3) {
                RCLCPP_WARN(this->get_logger(), "串口读取异常 (%d): %s", error_count, e.what());
            } else if (error_count == MAX_ERRORS) {
                RCLCPP_ERROR(this->get_logger(), "串口连续出错%d次，可能串口断开", MAX_ERRORS);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void RemoteNode::on_remote_control_data(const uint8_t* data, uint16_t size, void* user_data)
{
    RemoteNode* node = reinterpret_cast<RemoteNode*>(user_data);
    
    // 调试：打印收到数据的原始十六进制
    std::stringstream hex_dump;
    hex_dump << std::hex << std::uppercase << std::setfill('0');
    for (uint16_t i = 0; i < size; i++) {
        hex_dump << std::setw(2) << (int)data[i] << " ";
        if ((i + 1) % 16 == 0) hex_dump << "\n  ";
    }
    RCLCPP_INFO(node->get_logger(), "接收到遥控器数据包 (大小:%u字节):\n  %s", 
                size, hex_dump.str().c_str());
    
    // 打印前20个字节的raw hex用于分析
    std::stringstream raw_analysis;
    raw_analysis << std::hex << std::uppercase << std::setfill('0');
    raw_analysis << "[Raw Hex Analysis] ";
    for (uint16_t i = 0; i < std::min((uint16_t)20, size); i++) {
        raw_analysis << std::setw(2) << (int)data[i] << " ";
    }
    RCLCPP_INFO(node->get_logger(), "%s (first 20 bytes)", raw_analysis.str().c_str());

    // data 应包含遥控器的控制数据
    // 格式：float[4] (rocker0-3) + uint32_t (key) = 20字节
    if (size < 20) {
        RCLCPP_WARN(node->get_logger(), 
                   "遥控器数据长度错误: 期望>=20字节，实际%u字节", size);
        return;
    }
    
    float rocker0, rocker1, rocker2, rocker3;
    uint32_t key_data;
    
    memcpy(&rocker0, data + 0, sizeof(float));
    memcpy(&rocker1, data + 4, sizeof(float));
    memcpy(&rocker2, data + 8, sizeof(float));
    memcpy(&rocker3, data + 12, sizeof(float));
    memcpy(&key_data, data + 16, sizeof(uint32_t));

    // 打印解析后的float值（及其hex表示）
    uint32_t hex0, hex1, hex2, hex3;
    memcpy(&hex0, &rocker0, sizeof(uint32_t));
    memcpy(&hex1, &rocker1, sizeof(uint32_t)); 
    memcpy(&hex2, &rocker2, sizeof(uint32_t));
    memcpy(&hex3, &rocker3, sizeof(uint32_t));
    
    RCLCPP_INFO(node->get_logger(), 
                "[Float Debug] r0=%f (0x%08X) r1=%f (0x%08X) r2=%f (0x%08X) r3=%f (0x%08X) key=0x%08X", 
                rocker0, hex0, rocker1, hex1, rocker2, hex2, rocker3, hex3, key_data);

    // 发布ROS消息
    auto msg = std::make_unique<robot_interfaces::msg::MoveCmd>();
    msg->vx = rocker0;
    msg->vy = rocker1;
    msg->vz = rocker2;
    msg->wheel_vel = rocker3;
    msg->step_mode = (key_data & 0xFF);

    node->move_cmd_pub->publish(std::move(msg));

    RCLCPP_INFO(node->get_logger(), 
                "✓ 遥控器数据解析: rocker=[%.2f, %.2f, %.2f, %.2f], key=0x%02X", 
                rocker0, rocker1, rocker2, rocker3, (key_data & 0xFF));
}

void RemoteNode::on_bad_packet(uint32_t error_type)
{
    switch (error_type) {
        case 1:  // BAD_CHECKSUM
            RCLCPP_DEBUG(rclcpp::get_logger("remote_node"), "校验和错误");
            break;
        case 2:  // BAD_LENGTH
            RCLCPP_DEBUG(rclcpp::get_logger("remote_node"), "包长度错误");
            break;
        case 3:  // BAD_HEAD
            RCLCPP_DEBUG(rclcpp::get_logger("remote_node"), "包头错误");
            break;
        case 4:  // BAD_ACK
            RCLCPP_DEBUG(rclcpp::get_logger("remote_node"), "ACK包错误");
            break;
        default:
            RCLCPP_WARN(rclcpp::get_logger("remote_node"), "未知错误类型: %u", error_type);
    }
}
