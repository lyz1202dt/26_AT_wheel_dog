#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class IMUDriver{
public:
    IMUDriver(const std::shared_ptr<rclcpp::Node> node,const std::string &port,unsigned long int baudrate);
    ~IMUDriver();
    void data_recv();
    int pack_parsing();
private:
    // ROS2 Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_angular_vel_pub_;
    
    // CRC calculation functions
    static uint8_t calc_crc8(const uint8_t* data, size_t len);
    static uint16_t calc_crc16(const uint8_t* data, size_t len);
    
    // Helper function to find valid packet
    bool find_packet_header(size_t& start_pos, size_t available);
    
    // Data packet constants
    static constexpr uint8_t PACKET_START = 0xFC;
    static constexpr uint8_t PACKET_END = 0xFD;
    static constexpr uint8_t MSG_ACCELERATION = 0x61;
    static constexpr uint8_t MSG_ANGULAR_VEL = 0x66;
    static constexpr uint8_t MSG_QUAT_ORIEN = 0x64;
    
    unsigned char buffer[128];
    size_t buffer_len;
    
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<serial::Serial> serial_;
    std::unique_ptr<std::thread> serial_transmit_thread_;

    tf2::Quaternion rotation;
    tf2::Vector3 angular_velocity;
    tf2::Vector3 acceleration;

    // Sequence number tracking for packet loss detection
    // first_packet indicates we haven't received any packets yet
    uint8_t last_seq_num{0};
    bool first_packet{true};
    
    // CRC8 lookup table
    static const uint8_t CRC8_Table[256];
    
    // CRC16 lookup table
    static const uint16_t CRC16_Table[256];
};
