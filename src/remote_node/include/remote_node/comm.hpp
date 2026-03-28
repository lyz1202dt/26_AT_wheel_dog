#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <chrono>
#include <atomic>

/* -------------------- 协议常量 -------------------- */
#define PACK_NEED_ACK   (0x80u)
#define PACK_CMD_MASK   (0x7Fu)
#define PACK_OVERHEAD   (8u)     // head(1)+len(1)+cmd(1)+id(4)+sum(1)
#define PACK_MAX_SIZE   (256u)
#define PACK_HEAD       (0x5Au)  // 标准帧头
#define ACK_HEAD        (0xAAu)  // ACK帧头

/* -------------------- 回调函数类型 -------------------- */
typedef std::function<void(uint32_t type)> BadDataPackCb_t;
typedef std::function<void(uint8_t *src, uint16_t size, void* user_data)> CommPackRecv_Cb;
typedef std::function<void(void* user_data, uint32_t is_success)> CommPackSend_Cb;

/* -------------------- 错误类型 -------------------- */
enum CommBadType_t
{
    COMM_BAD_HEAD = 1,
    COMM_BAD_SUM  = 2,
    COMM_BAD_LEN  = 3,
    COMM_BAD_ACK  = 4,
};

/* -------------------- 发送请求结构 -------------------- */
struct DataTransReq_t
{
    uint16_t size;
    uint8_t cmd;
    uint8_t *data;
    uint8_t max_retry_cnt;
    uint32_t timeout_ms;
    CommPackSend_Cb finished_cb;
    void *user_data;
};

/* -------------------- 接收回调结构 -------------------- */
struct PackDealFunc_t
{
    CommPackRecv_Cb callback;
    void *user_data;
    uint32_t id;
    uint8_t cmd;
};

/* -------------------- ACK等待块 -------------------- */
struct AckWaitBlock_t
{
    uint32_t timeout_ms;
    int64_t send_time_ms;
    uint8_t is_using;
    uint32_t pack_id;
    CommPackSend_Cb finished_cb;
    void *user_data;

    // 零拷贝：data 指向外部缓冲
    uint16_t size;
    uint8_t cmd;
    uint8_t *data;
    uint8_t retry_cnt;
};

/* -------------------- 通信模块接口 -------------------- */
class RemoteComm
{
public:
    /**
     * @brief 初始化通信模块
     * @param port_name 串口设备名 (例如 "/dev/ttyUSB0")
     * @param baudrate 波特率
     * @param error_callback 错误数据包回调
     * @return 初始化是否成功
     */
    bool Init(const std::string& port_name, int baudrate, BadDataPackCb_t error_callback);

    /**
     * @brief 启动接收线程（必须在注册回调后调用）
     */
    void StartReceiver();

    /**
     * @brief 停止通信模块
     */
    void Stop();

    /**
     * @brief 注册接收回调
     * @param callback 接收回调函数
     * @param cmd 命令字节
     * @param user_data 用户数据
     * @return 注册ID
     */
    uint32_t RegisterRecvCb(CommPackRecv_Cb callback, uint8_t cmd, void* user_data);

    /**
     * @brief 取消注册接收回调
     * @param cb_id 注册ID
     * @return 1成功；0失败
     */
    uint32_t UnregisterRecvCb(uint32_t cb_id);

    /**
     * @brief 异步发送不需要ACK的数据包
     * @param src 数据指针
     * @param cmd 命令字节
     * @param size 数据长度
     * @return 1成功；0失败
     */
    uint32_t AsyncSendPackNak(uint8_t *src, uint8_t cmd, uint16_t size);

    /**
     * @brief 同步发送需要ACK的数据包（阻塞等待）
     * @param src 数据指针
     * @param cmd 命令字节
     * @param size 数据长度
     * @param timeout_ms 单次超时时间
     * @param max_retry_num 最大重试次数
     * @return 1成功；0失败
     */
    uint32_t SendPackAck(uint8_t *src, uint8_t cmd, uint16_t size, 
                        uint32_t timeout_ms, uint8_t max_retry_num);

    /**
     * @brief 异步发送需要ACK的数据包（带完成回调）
     * @param src 数据指针
     * @param cmd 命令字节
     * @param size 数据长度
     * @param send_cb 完成回调
     * @param user_data 用户数据
     * @param max_retry_num 最大重试次数
     * @return 1成功；0失败
     */
    uint32_t AsyncSendPackAck(uint8_t *src, uint8_t cmd, uint16_t size,
                             CommPackSend_Cb send_cb, void* user_data, uint8_t max_retry_num);

    /**
     * @brief 析构函数
     */
    ~RemoteComm();

private:
    // 校验和计算
    static uint8_t SumCheck(uint16_t size, uint8_t *src);

    // 获取当前时间（毫秒）
    static int64_t GetTickMS();

    // 线程函数
    void SendDataPackTask();
    void ReceiveDataPackTask();
    void ACKTimeoutCheckTask();

    // 打开和关闭串口
    bool OpenSerial(const std::string& port_name, int baudrate);
    void CloseSerial();
    int SerialRead(uint8_t *buf, size_t len, uint32_t timeout_ms);
    int SerialWrite(uint8_t *buf, size_t len);

private:
    // 串口相关
    int serial_fd_ = -1;
    std::string port_name_;
    int baudrate_;

    // 管理标志
    std::atomic<bool> running_{false};

    // 接收回调列表
    std::vector<PackDealFunc_t> recv_cb_list_;
    std::mutex recv_cb_list_mutex_;
    uint32_t callback_id_counter_ = 0;

    // 发送请求队列
    std::queue<DataTransReq_t> send_req_queue_;
    std::mutex send_req_queue_mutex_;
    std::condition_variable send_req_queue_cv_;

    // 串口发送互斥锁
    std::mutex uart_tx_mutex_;

    // ACK等待块
    std::vector<AckWaitBlock_t> send_ack_blocks_;
    std::mutex send_ack_blocks_mutex_;

    // 全局包ID
    uint32_t g_pack_id_ = 1;

    // 错误回调
    BadDataPackCb_t bad_data_cb_;

    // 发送和接收缓冲
    uint8_t send_buffer_[PACK_MAX_SIZE];
    uint8_t recv_buffer_[PACK_MAX_SIZE];

    // 工作线程
    std::thread send_thread_;
    std::thread recv_thread_;
    std::thread ack_timeout_thread_;
};
