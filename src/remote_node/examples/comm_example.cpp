/**
 * @file comm_example.cpp
 * @brief RemoteComm 通信模块使用示例
 * 
 * 此文件演示如何直接使用 RemoteComm 类实现自定义通信功能
 * （不使用 ROS2 RemoteNode 包装）
 */

#include "remote_node/comm.hpp"
#include <iostream>
#include <thread>
#include <chrono>

// ==================== 简单的接收数据回调 ====================
void OnReceiveData(uint8_t *src, uint16_t size, void* user_data)
{
    std::cout << "Received " << size << " bytes, cmd=" << (int)user_data << std::endl;
    for (uint16_t i = 0; i < size && i < 20; i++)
    {
        printf("%02X ", src[i]);
    }
    std::cout << std::endl;
}

// ==================== 错误处理回调 ====================
void OnBadDataPack(uint32_t type)
{
    std::string error_msg;
    switch (type)
    {
        case COMM_BAD_HEAD: error_msg = "Bad packet head"; break;
        case COMM_BAD_SUM:  error_msg = "Bad checksum"; break;
        case COMM_BAD_LEN:  error_msg = "Bad packet length"; break;
        case COMM_BAD_ACK:  error_msg = "Bad ACK packet"; break;
        default:            error_msg = "Unknown error"; break;
    }
    std::cerr << "Communication error: " << error_msg << std::endl;
}

// ==================== 异步发送完成回调 ====================
void OnAsyncSendComplete(void* user_data, uint32_t is_success)
{
    const char* result = is_success ? "SUCCESS" : "FAILED";
    std::cout << "Async send completed: " << result << ", user_data=" << (int)(intptr_t)user_data << std::endl;
}

// ==================== 主程序示例 ====================
int main()
{
    std::cout << "RemoteComm Module Example" << std::endl;
    std::cout << "=========================" << std::endl;

    // 创建通信模块
    auto comm = std::make_shared<RemoteComm>();

    // 初始化（连接到 /dev/ttyUSB1，波特率 115200）
    if (!comm->Init("/dev/ttyUSB1", 115200, OnBadDataPack))
    {
        std::cerr << "Failed to initialize RemoteComm" << std::endl;
        return -1;
    }

    std::cout << "RemoteComm initialized successfully" << std::endl;

    // ==================== 示例 1: 注册接收回调 ====================
    std::cout << "\n[Example 1] Register receive callback for command 0x01" << std::endl;
    uint32_t cb_id = comm->RegisterRecvCb(OnReceiveData, 0x01, (void*)1);
    std::cout << "Registered callback ID: " << cb_id << std::endl;

    // ==================== 示例 2: 异步发送（不需要 ACK） ====================
    std::cout << "\n[Example 2] Async send (NAK - No ACK)" << std::endl;
    uint8_t data_nak[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
    uint32_t ret = comm->AsyncSendPackNak(data_nak, 0x02, 10);
    std::cout << "AsyncSendPackNak returned: " << ret << std::endl;

    // ==================== 示例 3: 异步发送（需要 ACK） ====================
    std::cout << "\n[Example 3] Async send (ACK with callback)" << std::endl;
    uint8_t data_ack[10] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA};
    ret = comm->AsyncSendPackAck(data_ack, 0x03 | PACK_NEED_ACK, 10, 
                                OnAsyncSendComplete, (void*)100, 3);
    std::cout << "AsyncSendPackAck returned: " << ret << std::endl;

    // 等待异步操作完成
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // ==================== 示例 4: 同步发送（需要 ACK，阻塞等待） ====================
    std::cout << "\n[Example 4] Sync send (ACK - blocking)" << std::endl;
    uint8_t data_sync[10] = {0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA};
    std::cout << "Sending and waiting for ACK (timeout: 1000ms, max retry: 2)..." << std::endl;
    ret = comm->SendPackAck(data_sync, 0x04 | PACK_NEED_ACK, 10, 1000, 2);
    std::cout << "SendPackAck returned: " << ret << (ret ? " (SUCCESS)" : " (TIMEOUT)") << std::endl;

    // ==================== 示例 5: 注册第二个回调 ====================
    std::cout << "\n[Example 5] Register another callback for command 0x05" << std::endl;
    uint32_t cb_id2 = comm->RegisterRecvCb(OnReceiveData, 0x05, (void*)5);
    std::cout << "Registered callback ID: " << cb_id2 << std::endl;

    // 运行一段时间，接收可能的数据
    std::cout << "\nWaiting 5 seconds for incoming data..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // ==================== 示例 6: 取消注册回调 ====================
    std::cout << "\n[Example 6] Unregister callbacks" << std::endl;
    ret = comm->UnregisterRecvCb(cb_id);
    std::cout << "UnregisterRecvCb(" << cb_id << ") returned: " << ret << std::endl;

    ret = comm->UnregisterRecvCb(cb_id2);
    std::cout << "UnregisterRecvCb(" << cb_id2 << ") returned: " << ret << std::endl;

    // ==================== 关闭通信 ====================
    std::cout << "\n[Example 7] Stop communication" << std::endl;
    comm->Stop();
    std::cout << "RemoteComm stopped" << std::endl;

    std::cout << "\nExample completed!" << std::endl;
    return 0;
}

/**
 * 编译方法:
 * 
 * 如果在 ROS2 工作空间中：
 * $ cd ~/26_AT_wheel_dog
 * $ colcon build --packages-select remote_node
 * $ source install/setup.bash
 * 
 * 手动编译此示例（需要已安装的 remote_node 包）：
 * $ g++ -std=c++17 comm_example.cpp \
 *       -I/path/to/remote_node/include \
 *       -I/path/to/colcon_ws/install/remote_node/include \
 *       -L/path/to/colcon_ws/install/remote_node/lib \
 *       -o comm_example -lpthread
 * 
 * 运行:
 * $ ./comm_example
 * 
 * 注意: 确保有正确的串口设备和权限
 */
