#include "imu_driver/imu_driver.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <cstring>

// CRC8 lookup table
const uint8_t IMUDriver::CRC8_Table[256] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

// CRC16 lookup table
const uint16_t IMUDriver::CRC16_Table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

IMUDriver::IMUDriver(const std::shared_ptr<rclcpp::Node> node, const std::string& port, unsigned long int baudrate) 
    : buffer_len(0) {
    node_   = node;
    serial_ = std::make_unique<serial::Serial>(port, baudrate, serial::Timeout::simpleTimeout(100));
    if (!serial_->isOpen()) {
        RCLCPP_ERROR(node_->get_logger(), "打开设备失败:%s", port.c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "成功打开设备:%s,波特率为%ld", port.c_str(), baudrate);
    serial_transmit_thread_ = std::make_unique<std::thread>([this]() { data_recv(); });

    // initialize sequence tracking
    last_seq_num = 0;
    first_packet = true;
}

IMUDriver::~IMUDriver() {
    if (serial_->isOpen())
        serial_->close();
    if (serial_transmit_thread_) {
        if (serial_transmit_thread_->joinable())
            serial_transmit_thread_->join();
    }
}

uint8_t IMUDriver::calc_crc8(const uint8_t* data, size_t len) {
    uint8_t crc8 = 0;
	for (int i = 0; i < len; i++)
	{
		uint8_t value = data[i];
		uint8_t new_index = crc8 ^ value;
		crc8 = CRC8_Table[new_index];
	}
	return (crc8);
}

uint16_t IMUDriver::calc_crc16(const uint8_t* data, size_t len) {
    uint16_t crc16 = 0;
	for (int i = 0; i < len; i++)
	{
		uint8_t value = data[i];
		crc16 = CRC16_Table[((crc16 >> 8) ^ value) & 0xff] ^ (crc16 << 8);
	}
	return (crc16);
}

bool IMUDriver::find_packet_header(size_t& start_pos, size_t available) {
    // Search for valid packet ID in buffer
    for (size_t i = 0; i < available; i++) {
        uint8_t id = buffer[i];
        if (id == MSG_ACCELERATION || id == MSG_ANGULAR_VEL || id == MSG_QUAT_ORIEN) {
            start_pos = i;
            return true;
        }
    }
    return false;
}

void IMUDriver::data_recv() {
    while (rclcpp::ok()) {
        if (!serial_->isOpen()) {
            break;
        }

        try {
            // Step 1: 循环滑动接收一个字节直到这个字节等于包起始0xFC
            uint8_t byte;
            while (rclcpp::ok() && serial_->isOpen()) {
                serial_->read(&byte, 1);
                if (byte == PACKET_START) {
                    buffer[0] = byte;
                    buffer_len = 1;
                    break;
                }
            }
            
            if (!serial_->isOpen() || !rclcpp::ok()) {
                break;
            }

            // Step 2: 接收一个完整帧头（ID + len + seq + CRC8），共4字节
            size_t header_size = 4; // ID(1) + len(1) + seq(1) + CRC8(1)
            size_t to_read = header_size;
            size_t bytes_read = serial_->read(&buffer[buffer_len], to_read);
            buffer_len += bytes_read;

            // 验证CRC8（对ID + len + seq，即buffer[1], buffer[2], buffer[3]）
            uint8_t crc8_calc = calc_crc8(buffer, 4);
            uint8_t crc8_received = buffer[4];
            
            if (crc8_calc != crc8_received) {
                RCLCPP_WARN(node_->get_logger(), "CRC8校验失败: 计算=%02X, pack=%02X,%02X,%02X,%02X", crc8_calc, buffer[1],buffer[2],buffer[3],buffer[4]);
                buffer_len = 0;
                continue; // 重新开始步骤1
            }

            // 提取帧头信息
            uint8_t packet_id = buffer[1];
            uint8_t data_len = buffer[2];
            uint8_t seq_num = buffer[3];

            // 检查流水号以检测丢包
            if (!first_packet) {
                // 序号应该递增1，考虑28位循环
                uint8_t expected = last_seq_num + 1;
                if (seq_num != expected) {
                    RCLCPP_WARN(node_->get_logger(), "检测到丢包: 上一序号=%u 当前=%u", last_seq_num, seq_num);
                }
            } else {
                first_packet = false;
            }
            last_seq_num = seq_num;

            // 验证packet_id是否合法
            if (packet_id != MSG_ACCELERATION && packet_id != MSG_ANGULAR_VEL && packet_id != MSG_QUAT_ORIEN) {
                RCLCPP_WARN(node_->get_logger(), "未知的数据包类型: 0x%02X", packet_id);
                buffer_len = 0;
                continue;
            }

            // Step 3: 接收CRC16（2字节）和完整数据体和包结束符
            to_read = 2 + data_len + 1; // CRC16(2) + payload + END(1)
            bytes_read = serial_->read(&buffer[buffer_len], to_read);
            buffer_len += bytes_read;

            // 验证CRC16（对payload）
            uint16_t crc16_received = (buffer[5] << 8) | buffer[6];
            uint8_t* payload_start = &buffer[7];
            uint16_t crc16_calc = calc_crc16(payload_start, data_len);
            
            if (crc16_calc != crc16_received) {
                RCLCPP_WARN(node_->get_logger(), "CRC16校验失败: 计算=%04X, 接收=%04X", crc16_calc, crc16_received);
                buffer_len = 0;
                continue; // 重新开始步骤1
            }

            // Step 4: 验证包结束0xFD
            size_t end_pos = 7 + data_len; // 0xFC + header(4) + CRC16(2) + payload
            uint8_t end_marker = buffer[end_pos];
            if (end_marker != PACKET_END) {
                RCLCPP_WARN(node_->get_logger(), "数据包结束标志错误: 期望=0xFD, 接收=%02X", end_marker);
                buffer_len = 0;
                continue; // 重新开始步骤1
            }

            // Step 5: 所有校验成功，执行pack_parsing
            pack_parsing();
            
            // 清空buffer准备接收下一个数据包
            buffer_len = 0;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "串口读取异常: %s", e.what());
            buffer_len = 0;
        }
    }
}

int IMUDriver::pack_parsing() {
    // Buffer format: [0xFC][ID][len][seq][CRC8][CRC16_H][CRC16_L][payload...][0xFD]
    uint8_t packet_id = buffer[1];
    uint8_t* payload = buffer + 7; // Skip 0xFC, ID, len, seq, CRC8, CRC16

    // Parse based on packet type
    if (packet_id == MSG_ACCELERATION) {
        // Parse acceleration data (3 floats: X, Y, Z)
        float acc_x, acc_y, acc_z;
        memcpy(&acc_x, payload + 0, sizeof(float));
        memcpy(&acc_y, payload + 4, sizeof(float));
        memcpy(&acc_z, payload + 8, sizeof(float));
        
        acceleration.setX(acc_x);
        acceleration.setY(acc_y);
        acceleration.setZ(acc_z);
        
        RCLCPP_INFO(node_->get_logger(), "加速度: X=%.3f, Y=%.3f, Z=%.3f m/s²", acc_x, acc_y, acc_z);
        
    } else if (packet_id == MSG_ANGULAR_VEL) {
        // Parse angular velocity data (3 floats: X, Y, Z)
        float gyro_x, gyro_y, gyro_z;
        memcpy(&gyro_x, payload + 0, sizeof(float));
        memcpy(&gyro_y, payload + 4, sizeof(float));
        memcpy(&gyro_z, payload + 8, sizeof(float));
        
        angular_velocity.setX(gyro_x);
        angular_velocity.setY(gyro_y);
        angular_velocity.setZ(gyro_z);
        
        RCLCPP_INFO(node_->get_logger(), "角速度: X=%.3f, Y=%.3f, Z=%.3f rad/s", gyro_x, gyro_y, gyro_z);
        
    } else if (packet_id == MSG_QUAT_ORIEN) {
        // Parse quaternion orientation data (4 floats: Q0, Q1, Q2, Q3)
        float q0, q1, q2, q3;
        memcpy(&q0, payload + 0, sizeof(float));
        memcpy(&q1, payload + 4, sizeof(float));
        memcpy(&q2, payload + 8, sizeof(float));
        memcpy(&q3, payload + 12, sizeof(float));
        
        // Set quaternion (w, x, y, z)
        rotation.setW(q0);
        rotation.setX(q1);
        rotation.setY(q2);
        rotation.setZ(q3);
        
        RCLCPP_INFO(node_->get_logger(), "四元数: W=%.3f, X=%.3f, Y=%.3f, Z=%.3f", q0, q1, q2, q3);
        
    } else {
        RCLCPP_WARN(node_->get_logger(), "未知的数据包类型: 0x%02X", packet_id);
        return -1;
    }

    return 0;
}
