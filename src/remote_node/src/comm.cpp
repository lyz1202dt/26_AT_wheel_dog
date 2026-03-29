#include "remote_node/comm.hpp"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <iostream>

/* -------------------- 串口操作 -------------------- */

bool RemoteComm::OpenSerial(const std::string& port_name, int baudrate)
{
    // 打开串口
    serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0)
    {
        std::cerr << "Failed to open serial port: " << port_name << std::endl;
        return false;
    }

    // 获取串口配置
    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0)
    {
        std::cerr << "Failed to get serial attributes" << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    // 设置波特率
    speed_t speed;
    switch (baudrate)
    {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:     speed = B115200; break;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 配置串口参数
    tty.c_cflag |= (CLOCAL | CREAD);    // 忽略调制解调器控制线、启用接收
    tty.c_cflag &= ~CSIZE;              // 清除数据位掩码
    tty.c_cflag |= CS8;                 // 8位数据位
    tty.c_cflag &= ~PARENB;             // 无奇偶校验
    tty.c_cflag &= ~CSTOPB;             // 1位停止位

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // 关闭软件流控
    tty.c_oflag = 0;                     // 原始输出
    tty.c_lflag = 0;                     // 原始输入

    // 设置最小读取字符数和超时
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
        std::cerr << "Failed to set serial attributes" << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    port_name_ = port_name;
    baudrate_ = baudrate;
    return true;
}

void RemoteComm::CloseSerial()
{
    if (serial_fd_ >= 0)
    {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

int RemoteComm::SerialRead(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    if (serial_fd_ < 0)
        return -1;

    fd_set readfds;
    struct timeval tv;

    FD_ZERO(&readfds);
    FD_SET(serial_fd_, &readfds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(serial_fd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0)
        return -1;

    if (!FD_ISSET(serial_fd_, &readfds))
        return -1;

    int n = read(serial_fd_, buf, len);
    return (n > 0) ? n : -1;
}

int RemoteComm::SerialWrite(uint8_t *buf, size_t len)
{
    if (serial_fd_ < 0)
        return -1;

    int n = write(serial_fd_, buf, len);
    return (n > 0) ? n : -1;
}

/* -------------------- 工具函数 -------------------- */

uint8_t RemoteComm::SumCheck(uint16_t size, uint8_t *src)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < size; i++)
        sum += src[i];
    return sum;
}

int64_t RemoteComm::GetTickMS()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

/* -------------------- 公共接口 -------------------- */

bool RemoteComm::Init(const std::string& port_name, int baudrate, BadDataPackCb_t error_callback)
{
    if (!OpenSerial(port_name, baudrate))
        return false;

    bad_data_cb_ = error_callback;
    send_ack_blocks_.resize(8);
    for (auto& block : send_ack_blocks_)
        block.is_using = 0;

    running_ = true;

    // 启动发送和超时线程（不启动接收线程，等待 StartReceiver() 调用）
    send_thread_ = std::thread(&RemoteComm::SendDataPackTask, this);
    ack_timeout_thread_ = std::thread(&RemoteComm::ACKTimeoutCheckTask, this);

    return true;
}

void RemoteComm::StartReceiver()
{
    // 启动接收线程（应该在注册回调后调用）
    recv_thread_ = std::thread(&RemoteComm::ReceiveDataPackTask, this);
}

void RemoteComm::Stop()
{
    running_ = false;

    // 唤醒所有等待的线程
    send_req_queue_cv_.notify_all();

    // 等待所有线程结束
    if (send_thread_.joinable())
        send_thread_.join();
    if (recv_thread_.joinable())
        recv_thread_.join();
    if (ack_timeout_thread_.joinable())
        ack_timeout_thread_.join();

    CloseSerial();
}

uint32_t RemoteComm::RegisterRecvCb(CommPackRecv_Cb callback, uint8_t cmd, void* user_data)
{
    std::lock_guard<std::mutex> lock(recv_cb_list_mutex_);
    uint32_t id = ++callback_id_counter_;
    recv_cb_list_.push_back({callback, user_data, id, cmd});
    return id;
}

uint32_t RemoteComm::UnregisterRecvCb(uint32_t cb_id)
{
    std::lock_guard<std::mutex> lock(recv_cb_list_mutex_);
    auto it = std::find_if(recv_cb_list_.begin(), recv_cb_list_.end(),
                          [cb_id](const PackDealFunc_t& obj) { return obj.id == cb_id; });
    if (it != recv_cb_list_.end())
    {
        recv_cb_list_.erase(it);
        return 1;
    }
    return 0;
}

uint32_t RemoteComm::AsyncSendPackNak(uint8_t *src, uint8_t cmd, uint16_t size)
{
    if (!running_)
        return 0;

    DataTransReq_t req;
    req.cmd = cmd & (~((uint8_t)PACK_NEED_ACK));
    req.data = src;
    req.size = size;
    req.finished_cb = nullptr;
    req.user_data = nullptr;
    req.max_retry_cnt = 0;
    req.timeout_ms = 0;

    {
        std::lock_guard<std::mutex> lock(send_req_queue_mutex_);
        send_req_queue_.push(req);
    }
    send_req_queue_cv_.notify_one();
    return 1;
}

uint32_t RemoteComm::SendPackAck(uint8_t *src, uint8_t cmd, uint16_t size, 
                                uint32_t timeout_ms, uint8_t max_retry_num)
{
    if (!running_)
        return 0;

    DataTransReq_t req;
    req.cmd = cmd | PACK_NEED_ACK;
    req.data = src;
    req.size = size;
    req.max_retry_cnt = max_retry_num;
    req.timeout_ms = timeout_ms;
    req.finished_cb = nullptr;
    req.user_data = nullptr;

    // 创建二进制信号量用于等待ACK
    auto sem = std::make_shared<std::condition_variable>();
    auto mutex = std::make_shared<std::mutex>();
    auto result = std::make_shared<bool>(false);

    req.finished_cb = [sem, mutex, result](void* user_data, uint32_t is_success)
    {
        {
            std::lock_guard<std::mutex> lock(*mutex);
            *result = (is_success != 0);
        }
        sem->notify_one();
    };
    req.user_data = nullptr;

    {
        std::lock_guard<std::mutex> lock(send_req_queue_mutex_);
        send_req_queue_.push(req);
    }
    send_req_queue_cv_.notify_one();

    // 等待ACK或超时
    std::unique_lock<std::mutex> lock(*mutex);
    bool signaled = sem->wait_for(lock, std::chrono::milliseconds(timeout_ms * max_retry_num),
                                   [result]() { return *result; });

    return signaled ? 1 : 0;
}

uint32_t RemoteComm::AsyncSendPackAck(uint8_t *src, uint8_t cmd, uint16_t size,
                                     CommPackSend_Cb send_cb, void* user_data, uint8_t max_retry_num)
{
    if (!running_)
        return 0;

    DataTransReq_t req;
    req.cmd = cmd | PACK_NEED_ACK;
    req.data = src;
    req.size = size;
    req.max_retry_cnt = max_retry_num;
    req.timeout_ms = 1000;  // 默认超时1秒
    req.finished_cb = send_cb;
    req.user_data = user_data;

    {
        std::lock_guard<std::mutex> lock(send_req_queue_mutex_);
        send_req_queue_.push(req);
    }
    send_req_queue_cv_.notify_one();
    return 1;
}

RemoteComm::~RemoteComm()
{
    Stop();
}

/* -------------------- 线程任务 -------------------- */

void RemoteComm::SendDataPackTask()
{
    while (running_)
    {
        DataTransReq_t req;

        {
            std::unique_lock<std::mutex> lock(send_req_queue_mutex_);
            send_req_queue_cv_.wait(lock, [this]() { return !send_req_queue_.empty() || !running_; });

            if (!running_)
                break;

            if (send_req_queue_.empty())
                continue;

            req = send_req_queue_.front();
            send_req_queue_.pop();
        }

        uint32_t pack_id = g_pack_id_++;

        if (req.size + PACK_OVERHEAD > PACK_MAX_SIZE)
        {
            if (req.finished_cb)
                req.finished_cb(req.user_data, 0);
            continue;
        }

        // 构造数据包
        send_buffer_[0] = PACK_HEAD;
        send_buffer_[1] = (uint8_t)(req.size + PACK_OVERHEAD);
        send_buffer_[2] = req.cmd;
        memcpy(&send_buffer_[3], &pack_id, 4);
        memcpy(&send_buffer_[7], req.data, req.size);
        send_buffer_[7 + req.size] = SumCheck(req.size + 7, send_buffer_);

        // 发送数据包
        {
            std::lock_guard<std::mutex> lock(uart_tx_mutex_);
            SerialWrite(send_buffer_, req.size + PACK_OVERHEAD);
        }

        // 处理ACK
        if (!(req.cmd & PACK_NEED_ACK))
        {
            if (req.finished_cb)
                req.finished_cb(req.user_data, 1);
        }
        else
        {
            // 挂起ACK等待
            bool inserted = false;
            {
                std::lock_guard<std::mutex> lock(send_ack_blocks_mutex_);
                for (auto& block : send_ack_blocks_)
                {
                    if (!block.is_using)
                    {
                        block.is_using = 1;
                        block.pack_id = pack_id;
                        block.finished_cb = req.finished_cb;
                        block.user_data = req.user_data;
                        block.cmd = req.cmd;
                        block.data = req.data;
                        block.size = req.size;
                        block.retry_cnt = req.max_retry_cnt;
                        block.timeout_ms = req.timeout_ms;
                        block.send_time_ms = GetTickMS();
                        inserted = true;
                        break;
                    }
                }
            }

            if (!inserted && req.finished_cb)
                req.finished_cb(req.user_data, 0);
        }
    }
}

void RemoteComm::ReceiveDataPackTask()
{
    while (running_)
    {
        uint8_t head;
        int ret = SerialRead(&head, 1, 1000);
        if (ret != 1)
            continue;

        // ACK 包
        if (head == ACK_HEAD)
        {
            uint32_t ack_id = 0;
            int got = SerialRead((uint8_t *)&ack_id, 4, 20);
            if (got != 4)
            {
                if (bad_data_cb_)
                    bad_data_cb_(COMM_BAD_ACK);
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(send_ack_blocks_mutex_);
                for (auto& block : send_ack_blocks_)
                {
                    if (block.is_using && block.pack_id == ack_id)
                    {
                        if (block.finished_cb)
                            block.finished_cb(block.user_data, 1);
                        block.is_using = 0;
                        break;
                    }
                }
            }
            continue;
        }

        // 普通数据包
        if (head != PACK_HEAD)
        {
            if (bad_data_cb_)
                bad_data_cb_(COMM_BAD_HEAD);
            continue;
        }

        recv_buffer_[0] = head;

        // 读取长度
        int size_got = SerialRead(&recv_buffer_[1], 1, 20);
        if (size_got != 1)
        {
            if (bad_data_cb_)
                bad_data_cb_(COMM_BAD_LEN);
            continue;
        }

        uint16_t data_len = recv_buffer_[1];
        if (data_len > PACK_MAX_SIZE || data_len < PACK_OVERHEAD)
        {
            if (bad_data_cb_)
                bad_data_cb_(COMM_BAD_LEN);
            continue;
        }

        // 循环读取完整的有效载荷数据（20字节：CMD(1) + ID(4) + DATA(16)）
        uint16_t payload_to_read = 20;  // 新格式固定为20字节
        int payload_got = 0;
        int read_timeout = 200;  // 总超时时间
        int read_attempts = 0;
        const int max_attempts = 50;  // 最多重试50次，约100ms
        
        while (payload_got < payload_to_read && read_attempts < max_attempts)
        {
            int chunk = SerialRead(&recv_buffer_[2 + payload_got], 
                                  payload_to_read - payload_got, 2);
            if (chunk > 0)
            {
                payload_got += chunk;
            }
            else
            {
                read_attempts++;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }

        if (payload_got < 20)
        {
            if (bad_data_cb_)
                bad_data_cb_(COMM_BAD_LEN);
            continue;
        }

        // 命令字和验证
        uint8_t cmd = recv_buffer_[2];  // 命令在位置2
        
        // 调用回调函数（数据从位置7开始，共20字节RemoteData）
        uint8_t *data_ptr = recv_buffer_ + 7;  // 跳过包头(1) + 长度(1) + 命令(1) + ID(4)
        uint16_t data_size = 20;  // 新格式固定为20字节(float[4] + uint32_t)
        
        if (cmd & PACK_NEED_ACK)
        {
            uint8_t ack[5];
            ack[0] = ACK_HEAD;
            memcpy(ack + 1, recv_buffer_ + 3, 4);

            {
                std::lock_guard<std::mutex> lock(uart_tx_mutex_);
                SerialWrite(ack, sizeof(ack));
            }
        }

        // 调用回调函数
        {
            std::lock_guard<std::mutex> lock(recv_cb_list_mutex_);
            for (const auto& obj : recv_cb_list_)
            {
                if (obj.cmd == (cmd & PACK_CMD_MASK))
                {
                    obj.callback(data_ptr, data_size, obj.user_data);
                }
            }
        }
    }
}

void RemoteComm::ACKTimeoutCheckTask()
{
    while (running_)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(4));

        int64_t cut_time = GetTickMS();

        {
            std::lock_guard<std::mutex> lock(send_ack_blocks_mutex_);
            for (auto& block : send_ack_blocks_)
            {
                if (block.is_using)
                {
                    if (cut_time - block.send_time_ms >= (int64_t)block.timeout_ms)
                    {
                        if (block.retry_cnt)
                        {
                            // 重新发送
                            DataTransReq_t req;
                            req.cmd = block.cmd;
                            req.data = block.data;
                            req.finished_cb = block.finished_cb;
                            req.max_retry_cnt = block.retry_cnt - 1;
                            req.size = block.size;
                            req.timeout_ms = block.timeout_ms;
                            req.user_data = block.user_data;

                            {
                                std::lock_guard<std::mutex> req_lock(send_req_queue_mutex_);
                                send_req_queue_.push(req);
                            }
                            send_req_queue_cv_.notify_one();

                            block.is_using = 0;
                        }
                        else
                        {
                            // 超时失败
                            if (block.finished_cb)
                                block.finished_cb(block.user_data, 0);
                            block.is_using = 0;
                        }
                    }
                }
            }
        }
    }
}
