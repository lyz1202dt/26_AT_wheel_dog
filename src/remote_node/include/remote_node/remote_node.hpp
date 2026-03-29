#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <serial/serial.h>
#include <robot_interfaces/msg/move_cmd.hpp>
#include <thread>
#include "remote_node/remote_comm.hpp"

// 远程控制器数据包命令类型
constexpr uint8_t CMD_REMOTE_CONTROL = 0x01;  // 遥控器数据包命令

class RemoteNode : public rclcpp::Node
{
public:
    RemoteNode();
    ~RemoteNode();

private:
    // ROS2 Publishers
    rclcpp::Publisher<robot_interfaces::msg::MoveCmd>::SharedPtr move_cmd_pub;

    // 串口通信
    std::unique_ptr<serial::Serial> serial_;
    std::unique_ptr<std::thread> serial_recv_thread_;
    
    // 通信协议处理器
    std::unique_ptr<RemoteComm> remote_comm_;

    // 回调ID（用于取消注册）
    uint32_t remote_control_cb_id_;

    // 串口接收线程函数
    void serial_recv_task();

    // 串口发送线程函数
    void serial_send_task();

    // 遥控器数据接收回调
    static void on_remote_control_data(const uint8_t* data, uint16_t size, void* user_data);

    // 错误包处理回调
    static void on_bad_packet(uint32_t error_type);
};
