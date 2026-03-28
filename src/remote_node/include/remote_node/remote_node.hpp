#pragma once

#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/move_cmd.hpp>
#include "remote_node/comm.hpp"

#include <thread>
#include <atomic>
#include <vector>
#include <cmath>
#include <memory>

// ==================== 遥控器数据包（协议定义） ====================
// 在通信协议中，数据部分包含摇杆和按键信息
#pragma pack(push, 1)
struct RemoteData_t
{
    int16_t rocker[4];     // 摇杆 ADC 值，范围 0~2047
    uint8_t key1;          // 按键组 1 状态
    uint8_t key2;          // 按键组 2 状态
};
#pragma pack(pop)

// ==================== ROS2节点 ====================
class RemoteNode : public rclcpp::Node
{
public:
    RemoteNode();
    ~RemoteNode();

private:
    // 数据处理回调
    void OnRemoteDataReceived(uint8_t *src, uint16_t size, void* user_data);
    
    // 错误处理回调
    void OnBadDataPack(uint32_t type);

    // 数据处理
    void ProcessData(const RemoteData_t &data);
    
    // Bezier 曲线变换（对应STM32中的非线性映射）
    float BezierTransform(float x, const std::vector<float>& bezier);

private:
    // 通信模块
    std::shared_ptr<RemoteComm> comm_;
    uint32_t recv_cb_id_;

    // 发布器
    rclcpp::Publisher<robot_interfaces::msg::MoveCmd>::SharedPtr move_cmd_pub;

    // ==================== 滤波变量（对应STM32） ====================
    float last_v0 = 0.0f;          // 摇杆0滤波值
    float last_v1 = 0.0f;          // 摇杆1滤波值
    float last_v3 = 0.0f;          // 摇杆3滤波值
    float last_omega = 0.0f;       // 角速度滤波值
    float cur_dir = 0.0f;          // 当前方向

    // 滤波系数
    float filter_gate  = 1.0f;      // 摇杆滤波系数
    float filter_alpha = 0.2f;     // 第4摇杆滤波系数

    // ==================== 运动参数 ====================
    float max_speed = 0.4f;            // 最大前进速度
    float max_forword_speed = 1.0f;    // 最大前进速度
    float max_backward_speed = 0.5f;   // 最大后退速度
    float max_omega = 120.0f;          // 最大角速度（度/秒）

    // ==================== 超时检测 ====================
    rclcpp::Time last_receive_time_;   // 最后一次接收数据的时间
    rclcpp::TimerBase::SharedPtr timeout_timer_;  // 超时检测定时器
    static constexpr int TIMEOUT_MS = 200;        // 200ms 超时
    
    // 按键状态
    uint8_t key1 = 0;

    // Bezier 曲线系数（可根据需要调整，这里使用默认的二阶Bezier）
    std::vector<float> bezier{0.0f, 0.05f, 1.0f};
};

