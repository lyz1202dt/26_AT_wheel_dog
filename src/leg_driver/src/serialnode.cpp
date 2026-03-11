#include "leg_driver/serialnode.hpp"
#include "leg_driver/cdc_trans.hpp"
#include "leg_driver/data_pack.h"
#include "leg_driver/kalman_filter.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include <robot_interfaces/msg/robot.hpp>
#include <robot_interfaces/msg/robot_target.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>


using namespace std::chrono_literals;

SerialNode::SerialNode()
    : Node("driver_node") {

    // 初始化状态
    exit_thread           = false;
    legs_target.pack_type = 0x00;

    // 初始化卡尔曼滤波器
    // 参数: process_noise=0.001, measurement_noise=0.1
    // 这些参数可以根据实际电机力矩噪声特性调整
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            torque_filters[i][j] = KalmanFilter(0.005f, 0.01f, 0.0f, 1.0f);
        }
        wheel_torque_filters[i] = KalmanFilter(0.001f, 0.5f, 0.0f, 1.0f);
    }

    this->declare_parameter("publish_imu", true); // 声明该节点是否负责发布IMU数据

    // 声明关节卡尔曼滤波器参数
    this->declare_parameter("joint_kalman_filter_q", 0.005);
    this->declare_parameter("joint_kalman_filter_r", 0.02);

    // 声明轮子卡尔曼滤波器参数
    this->declare_parameter("wheel_kalman_filter_q", 0.001);
    this->declare_parameter("wheel_kalman_filter_r", 0.5);

    param_server_ = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        bool pid_updated = false;

        for (const auto& param : params) {
            if (param.get_name() == "joint_kalman_filter_q") {
                float q_value = static_cast<float>(param.as_double());
                // 更新所有关节卡尔曼滤波器的 Q 值
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 3; j++) {
                        torque_filters[i][j].setProcessNoise(q_value);
                    }
                }
                RCLCPP_INFO(this->get_logger(), "更新关节卡尔曼滤波器 Q 参数: %.6f", q_value);
            } else if (param.get_name() == "joint_kalman_filter_r") {
                float r_value = static_cast<float>(param.as_double());
                // 更新所有关节卡尔曼滤波器的 R 值
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 3; j++) {
                        torque_filters[i][j].setMeasurementNoise(r_value);
                    }
                }
                RCLCPP_INFO(this->get_logger(), "更新关节卡尔曼滤波器 R 参数: %.6f", r_value);
            } else if (param.get_name() == "wheel_kalman_filter_q") {
                float q_value = static_cast<float>(param.as_double());
                // 更新所有轮子卡尔曼滤波器的 Q 值
                for (int i = 0; i < 4; i++) {
                    wheel_torque_filters[i].setProcessNoise(q_value);
                }
                RCLCPP_INFO(this->get_logger(), "更新轮子卡尔曼滤波器 Q 参数: %.6f", q_value);
            } else if (param.get_name() == "wheel_kalman_filter_r") {
                float r_value = static_cast<float>(param.as_double());
                // 更新所有轮子卡尔曼滤波器的 R 值
                for (int i = 0; i < 4; i++) {
                    wheel_torque_filters[i].setMeasurementNoise(r_value);
                }
                RCLCPP_INFO(this->get_logger(), "更新轮子卡尔曼滤波器 R 参数: %.6f", r_value);
            }
        }

        if (pid_updated) {
            RCLCPP_INFO(this->get_logger(), "更新 PID 参数");
        }

        return result;
    });

    publish_imu = this->get_parameter("publish_imu").as_bool();

    // 先创建 publisher/subscriber，确保回调中 publish 时 publisher 已就绪
    robot_pub = this->create_publisher<robot_interfaces::msg::Robot>("legs_status", 10);

    if (publish_imu) {
        imu_posture_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/imu_pose_sensor/pose", rclcpp::SensorDataQoS());
        imu_state_pub   = this->create_publisher<sensor_msgs::msg::Imu>("/imu_imu_sensor/imu", rclcpp::SensorDataQoS());
    }

    robot_sub = this->create_subscription<robot_interfaces::msg::RobotTarget>(
        "legs_target", rclcpp::SensorDataQoS(), std::bind(&SerialNode::legsSubscribCb, this, std::placeholders::_1));
    remote_pub = this->create_publisher<robot_interfaces::msg::MoveCmd>("robot_move_cmd", 10);


    cdc_trans = std::make_unique<CDCTrans>();                           // 创建CDC传输对象
    cdc_trans->regeiser_recv_cb([this](const uint8_t* data, int size) { // 注册接收回调
        int pack_type = *((int*)data);
        if (pack_type == 0) {
            if (size == sizeof(DogStatePack0_t)) {
                const DogStatePack0_t* pack = reinterpret_cast<const DogStatePack0_t*>(data);
                publishLegState(pack);
            }
        } else if (pack_type == 1) {
            if (size == sizeof(DogStatePack1_t)) {
                const DogStatePack1_t* pack = reinterpret_cast<const DogStatePack1_t*>(data);
                publishLegState(pack);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "接收到错误的数据包类型%d", pack_type);
        }
    });

    if (!cdc_trans->open(0x0483, 0x5740))                               // 开启USB_CDC传输接口
        exit_thread = true;

    // 创建线程处理CDC消息（在 open 之后、publisher 创建之后）
    usb_event_handle_thread = std::make_unique<std::thread>([this]() {
        do {
            cdc_trans->process_once();
        } while (!exit_thread);
    });

    base_time = this->get_clock()->now();
}

SerialNode::~SerialNode() {
    // 请求线程退出并等待其结束，保证安全关闭
    exit_thread = true;
    if (usb_event_handle_thread && usb_event_handle_thread->joinable()) {
        usb_event_handle_thread->join();
    }
    if (cdc_trans) {
        cdc_trans->close();
    }
}

void SerialNode::publishLegState(const DogStatePack0_t* legs_state) {
    robot_interfaces::msg::Robot msg;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            msg.legs[i].joints[j].rad   = legs_state->leg[i].joint[j].rad;
            msg.legs[i].joints[j].omega = legs_state->leg[i].joint[j].omega;
            // 对力矩值应用卡尔曼滤波
            float raw_torque             = legs_state->leg[i].joint[j].torque;
            msg.legs[i].joints[j].torque = torque_filters[i][j].update(raw_torque);
        }
        msg.legs[i].wheel.omega = legs_state->leg[i].wheel.omega;
        // 对轮子力矩值应用卡尔曼滤波
        float raw_wheel_torque   = legs_state->leg[i].wheel.torque;
        msg.legs[i].wheel.torque = wheel_torque_filters[i].update(raw_wheel_torque);
    }

    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 3; j++) {
    //         msg.legs[i].joints[j].rad    = legs_state->leg[i].joint[j].rad;
    //         msg.legs[i].joints[j].omega  = legs_state->leg[i].joint[j].omega;
    //         msg.legs[i].joints[j].torque = legs_state->leg[i].joint[j].torque;
    //     }
    //     msg.legs[i].wheel.omega  = legs_state->leg[i].wheel.omega;
    //     msg.legs[i].wheel.torque = legs_state->leg[i].wheel.torque;
    // }

    state_log_print_cnt++;
    if (state_log_update_cnt == state_log_print_cnt) {
        state_log_print_cnt = 0;
        RCLCPP_INFO(this->get_logger(), "发布电机状态");
        publishremote(legs_state);
    }
    if (legs_state->motor_state) {
        RCLCPP_INFO(get_logger(), "电机异常%d", legs_state->motor_state);
        exit(-1);
    }
    robot_pub->publish(msg);


    if (publish_imu) {
        geometry_msgs::msg::PoseStamped imu_msg;
        tf2::Quaternion q;
        q.setRPY(legs_state->JY61.Angle.Roll, legs_state->JY61.Angle.Pitch, legs_state->JY61.Angle.Yaw);
        imu_msg.pose.orientation.x = q.x();
        imu_msg.pose.orientation.y = q.y();
        imu_msg.pose.orientation.z = q.z();
        imu_msg.pose.orientation.w = q.w();
        sensor_msgs::msg::Imu imu_state_msg;
        imu_state_msg.header.frame_id    = "world";
        imu_state_msg.header.stamp       = this->get_clock()->now();
        imu_state_msg.angular_velocity.x = legs_state->JY61.AngularVelocity.X;
        imu_state_msg.angular_velocity.y = legs_state->JY61.AngularVelocity.Y;
        imu_state_msg.angular_velocity.z = legs_state->JY61.AngularVelocity.Z;
        imu_state_pub->publish(imu_state_msg);
        imu_posture_pub->publish(imu_msg);
    }
}

void SerialNode::publishLegState(const DogStatePack1_t* legs_state) {
    robot_interfaces::msg::Robot msg;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            msg.legs[i].joints[j].rad   = legs_state->leg[i].joint[j].rad;
            msg.legs[i].joints[j].omega = legs_state->leg[i].joint[j].omega;
            // 对力矩值应用卡尔曼滤波
            float raw_torque             = legs_state->leg[i].joint[j].torque;
            msg.legs[i].joints[j].torque = torque_filters[i][j].update(raw_torque);
        }
        msg.legs[i].wheel.omega = legs_state->leg[i].wheel.omega;
        // 对轮子力矩值应用卡尔曼滤波
        float raw_wheel_torque   = legs_state->leg[i].wheel.torque;
        msg.legs[i].wheel.torque = wheel_torque_filters[i].update(raw_wheel_torque);
    }

    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 3; j++) {
    //         msg.legs[i].joints[j].rad    = legs_state->leg[i].joint[j].rad;
    //         msg.legs[i].joints[j].omega  = legs_state->leg[i].joint[j].omega;
    //         msg.legs[i].joints[j].torque = legs_state->leg[i].joint[j].torque;
    //     }
    //     msg.legs[i].wheel.omega  = legs_state->leg[i].wheel.omega;
    //     msg.legs[i].wheel.torque = legs_state->leg[i].wheel.torque;
    // }

    state_log_print_cnt++;
    if (state_log_update_cnt == state_log_print_cnt) {
        state_log_print_cnt = 0;
        RCLCPP_INFO(this->get_logger(), "发布电机状态");
        publishremote(legs_state);
    }
    if (legs_state->motor_state) {
        RCLCPP_INFO(get_logger(), "电机异常%d", legs_state->motor_state);
        exit(-1);
    }
    robot_pub->publish(msg);
}

void SerialNode::legsSubscribCb(const robot_interfaces::msg::RobotTarget& msg) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            legs_target.leg[i].joint[j].rad    = msg.legs[i].joints[j].rad;
            legs_target.leg[i].joint[j].omega  = msg.legs[i].joints[j].omega;
            legs_target.leg[i].joint[j].torque = msg.legs[i].joints[j].torque;
            legs_target.leg[i].joint[j].kp     = msg.legs[i].joints[j].kp;
            legs_target.leg[i].joint[j].kd     = msg.legs[i].joints[j].kd;
        }
        legs_target.leg[i].wheel.omega  = msg.legs[i].wheel.omega;
        legs_target.leg[i].wheel.torque = msg.legs[i].wheel.torque;
        legs_target.leg[i].wheel.kd     = msg.legs[i].wheel.kd;
    }

    // if(enable_control)
    cdc_trans->send_struct(legs_target); // 一旦订阅到最新的包，立即发送到下位机

    target_log_print_cnt++;
    if (target_log_update_cnt == target_log_print_cnt) {
        target_log_print_cnt = 0;
        RCLCPP_INFO(this->get_logger(), "订阅到电机目标值");
    }

    first_update = false;
}

void SerialNode::publishremote(const DogStatePack0_t* legs_remote) {
    robot_interfaces::msg::MoveCmd remote_msg;
    remote_msg.vx        = legs_remote->remote.vx;
    remote_msg.vy        = legs_remote->remote.vy;
    remote_msg.vz        = legs_remote->remote.omega;
    remote_msg.wheel_vel = legs_remote->remote.wheel_v;

    if (std::abs(remote_msg.vx) > 0.01 || std::abs(remote_msg.vy) > 0.01 || std::abs(remote_msg.vz) > 0.01) {
        base_time            = this->get_clock()->now();
        remote_msg.step_mode = 2;
        runned               = true;
    } else if ((this->get_clock()->now() - base_time).seconds() < 2.0 && runned) {
        remote_msg.step_mode = 2;
    } else
        remote_msg.step_mode = 1;

    // RCLCPP_INFO(this->get_logger(), "remote.vx %f,imu_r %f", legs_remote->remote.vx, legs_remote->JY61.Angle.Pitch);
    remote_pub->publish(remote_msg);
}

void SerialNode::publishremote(const DogStatePack1_t* legs_remote) {
    robot_interfaces::msg::MoveCmd remote_msg;
    remote_msg.vx        = legs_remote->remote.vx;
    remote_msg.vy        = legs_remote->remote.vy;
    remote_msg.vz        = legs_remote->remote.omega;
    remote_msg.wheel_vel = legs_remote->remote.wheel_v;

    if (std::abs(remote_msg.vx) > 0.01 || std::abs(remote_msg.vy) > 0.01 || std::abs(remote_msg.vz) > 0.01) {
        base_time            = this->get_clock()->now();
        remote_msg.step_mode = 2;
        runned               = true;
    } else if ((this->get_clock()->now() - base_time).seconds() < 2.0 && runned) {
        remote_msg.step_mode = 2;
    } else
        remote_msg.step_mode = 1;

    remote_pub->publish(remote_msg);
}
