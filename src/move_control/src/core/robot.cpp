#include "core/robot.hpp"
#include "fsm/fsm.hpp"
#include "leg/step.hpp"

#include <chrono>
#include <rclcpp/create_timer.hpp>

#include "states/idel.hpp"
#include "states/setup.hpp"
#include "states/stop.hpp"
#include "states/mpc.hpp"
#include "states/walk.hpp"


using namespace std::chrono_literals;

Robot::Robot(const std::shared_ptr<rclcpp::Node> node)
    : fsm(this, "setup") {
    node_ = node;

    // 初始化参数回调vector
    param_cb_vector.clear();
    joint_display_msg.position.resize(16);
    joint_display_msg.name = joint_names;

    lf_z_vmc = std::make_shared<VMC>(500, 120, 4.0, 0.5, 0.2, 0.1, 10ms);
    rf_z_vmc = std::make_shared<VMC>(500, 120, 4.0, 0.5, 0.2, 0.1, 10ms);
    lb_z_vmc = std::make_shared<VMC>(500, 120, 4.0, 0.5, 0.2, 0.1, 10ms);
    rb_z_vmc = std::make_shared<VMC>(500, 120, 4.0, 0.5, 0.2, 0.1, 10ms);

    lf_x_vmc = std::make_shared<VMC>(160, 60, 3.0, 0.5, 0.2, 0.1, 10ms);
    lf_y_vmc = std::make_shared<VMC>(160, 60, 3.0, 0.5, 0.2, 0.1, 10ms);
    rf_x_vmc = std::make_shared<VMC>(160, 60, 3.0, 0.5, 0.2, 0.1, 10ms);
    rf_y_vmc = std::make_shared<VMC>(160, 60, 3.0, 0.5, 0.2, 0.1, 10ms);
    lb_x_vmc = std::make_shared<VMC>(160, 60, 3.0, 0.5, 0.2, 0.1, 10ms);
    lb_y_vmc = std::make_shared<VMC>(160, 60, 3.0, 0.5, 0.2, 0.1, 10ms);
    rb_x_vmc = std::make_shared<VMC>(160, 60, 3.0, 0.5, 0.2, 0.1, 10ms);
    rb_y_vmc = std::make_shared<VMC>(160, 60, 3.0, 0.5, 0.2, 0.1, 10ms);

    // 狗身平衡VMC
    roll_vmc  = std::make_shared<SimpleVMC>(-200.0, 0.0, 100);
    pitch_vmc = std::make_shared<SimpleVMC>(500.0, 100.0, 100);

    node_->declare_parameter("direction_filter_gate", 0.2);
    node_->declare_parameter("vmc_kp", 100.0);
    node_->declare_parameter("vmc_kd", 120.0);
    node_->declare_parameter("vmc_mass", 0.5);

    node_->declare_parameter("horizontal_vmc_kp", 500.0);
    node_->declare_parameter("horizontal_vmc_kd", 150.0);
    node_->declare_parameter("horizontal_vmc_mass", 3.0);

    node_->declare_parameter("roll_vmc_kp", -300.0);
    node_->declare_parameter("roll_vmc_kd", -100.0);
    node_->declare_parameter("pitch_vmc_kp", 500.0);
    node_->declare_parameter("pitch_vmc_kd", 0.0);

    node_->declare_parameter("lf_grivate", 22.0);
    node_->declare_parameter("rf_grivate", 22.0);
    node_->declare_parameter("lb_grivate", 26.0);
    node_->declare_parameter("rb_grivate", 26.0);
    node_->declare_parameter("lf_dx", 0.0);
    node_->declare_parameter("rf_dx", 0.0);
    node_->declare_parameter("lb_dx", 0.0);
    node_->declare_parameter("rb_dx", 0.0);
    node_->declare_parameter("body_height", 0.25);


    param_server_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        RCLCPP_INFO(node_->get_logger(), "更新参数");

        for (const auto& param : params) {
            default_param_cb(param);
            for (const auto& callback : param_cb_vector) {
                bool ret = callback(param);
                if (!ret)
                    RCLCPP_INFO(node_->get_logger(), "参数更新错误!");
            }
        }
        return result;
    });


    // 读取所有默认参数
    // ========== direction filter ==========
    node_->get_parameter("direction_filter_gate", direction_filter_gate);
    direction_filter_gate = std::clamp(direction_filter_gate, 0.0, 1.0);

    // ========== vertical VMC (Z) ==========
    node_->get_parameter("vmc_kp", lf_z_vmc->kp);
    rf_z_vmc->kp = lb_z_vmc->kp = rb_z_vmc->kp = lf_z_vmc->kp;

    node_->get_parameter("vmc_kd", lf_z_vmc->kd);
    rf_z_vmc->kd = lb_z_vmc->kd = rb_z_vmc->kd = lf_z_vmc->kd;

    node_->get_parameter("vmc_mass", lf_z_vmc->mass);
    rf_z_vmc->mass = lb_z_vmc->mass = rb_z_vmc->mass = lf_z_vmc->mass;

    // ========== horizontal VMC (X/Y) ==========
    double horizontal_kp, horizontal_kd, horizontal_mass;
    node_->get_parameter("horizontal_vmc_kp", horizontal_kp);
    node_->get_parameter("horizontal_vmc_kd", horizontal_kd);
    node_->get_parameter("horizontal_vmc_mass", horizontal_mass);

    lf_x_vmc->kp = lf_y_vmc->kp = rf_x_vmc->kp = rf_y_vmc->kp = lb_x_vmc->kp = lb_y_vmc->kp = rb_x_vmc->kp = rb_y_vmc->kp = horizontal_kp;

    lf_x_vmc->kd = lf_y_vmc->kd = rf_x_vmc->kd = rf_y_vmc->kd = lb_x_vmc->kd = lb_y_vmc->kd = rb_x_vmc->kd = rb_y_vmc->kd = horizontal_kd;

    lf_x_vmc->mass = lf_y_vmc->mass = rf_x_vmc->mass = rf_y_vmc->mass = lb_x_vmc->mass = lb_y_vmc->mass = rb_x_vmc->mass = rb_y_vmc->mass =
        horizontal_mass;

    // ========== roll / pitch VMC ==========
    node_->get_parameter("roll_vmc_kp", roll_vmc->kp);
    node_->get_parameter("roll_vmc_kd", roll_vmc->kd);
    node_->get_parameter("pitch_vmc_kp", pitch_vmc->kp);
    node_->get_parameter("pitch_vmc_kd", pitch_vmc->kd);

    // ========== leg gravity ==========
    node_->get_parameter("lf_grivate", robot_lf_grivate);
    node_->get_parameter("rf_grivate", robot_rf_grivate);
    node_->get_parameter("lb_grivate", robot_lb_grivate);
    node_->get_parameter("rb_grivate", robot_rb_grivate);

    node_->get_parameter("body_height", body_height);

    // ========== foot x offset ==========
    double lf_dx_temp, rf_dx_temp, lb_dx_temp, rb_dx_temp;
    node_->get_parameter("lf_dx", lf_dx_temp);
    node_->get_parameter("rf_dx", rf_dx_temp);
    node_->get_parameter("lb_dx", lb_dx_temp);
    node_->get_parameter("rb_dx", rb_dx_temp);
    lf_base_offset << 0.25 + lf_dx_temp, 0.18, -body_height;
    rf_base_offset << 0.25 + rf_dx_temp, -0.18, -body_height;
    lb_base_offset << -0.22 + lb_dx_temp, 0.18, -body_height;
    rb_base_offset << -0.22 + rb_dx_temp, -0.18, -body_height;
    

    robot_rotation.setRPY(0.0, 0.0, 0.0);

    // RVIZ2可视化
    marker_publisher     = node_->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    rviz_joint_publisher = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    robot_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    // 发布机器人关节期望
    legs_target_pub = node_->create_publisher<robot_interfaces::msg::Robot>("legs_target", 10);

    // 订阅机器人位姿信息
    // pose_sensor 发布的是 PoseStamped（见 mujoco_ros2_control/src/pose_sensor.cpp），这里必须用 PoseStamped 才能收到消息
    // 同时 QoS 需要与发布端兼容（发布端当前是 RELIABLE + TRANSIENT_LOCAL）
    imu_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/imu_pose_sensor/pose", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
        [this](const geometry_msgs::msg::PoseStamped& msg) {
            const auto& q_msg = msg.pose.orientation;

            double qw = robot_rotation.getW();
            double qx = robot_rotation.getX();
            double qy = robot_rotation.getY();
            double qz = robot_rotation.getZ();

            quaternionLowPassFilter(qw, qx, qy, qz, q_msg.w, q_msg.x, q_msg.y, q_msg.z, direction_filter_gate); //(0-强滤波、1-不滤波)

            robot_rotation.setW(qw);
            robot_rotation.setX(qx);
            robot_rotation.setY(qy);
            robot_rotation.setZ(qz);
        });

    imu_angular_vel_sub = node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/imu_imu_sensor/imu", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
        [this](const geometry_msgs::msg::Vector3& msg) {
            robot_velocity.angular.x = robot_velocity.angular.x + direction_filter_gate * (msg.x - robot_velocity.angular.x);
            robot_velocity.angular.y = robot_velocity.angular.y + direction_filter_gate * (msg.y - robot_velocity.angular.y);
            robot_velocity.angular.z = robot_velocity.angular.z + direction_filter_gate * (msg.z - robot_velocity.angular.z);
        });

    // 订阅机器人关节状态信息
    legs_state_sub =
        node_->create_subscription<robot_interfaces::msg::Robot>("legs_status", 10, [this](const robot_interfaces::msg::Robot& msg) {
            for (int i = 0; i < 3; i++) {
                lf_joint_pos[i] = (double)msg.legs[0].joints[i].rad;
                rf_joint_pos[i] = (double)msg.legs[1].joints[i].rad;
                lb_joint_pos[i] = (double)msg.legs[2].joints[i].rad;
                rb_joint_pos[i] = (double)msg.legs[3].joints[i].rad;

                lf_joint_vel[i] = (double)msg.legs[0].joints[i].omega;
                rf_joint_vel[i] = (double)msg.legs[1].joints[i].omega;
                lb_joint_vel[i] = (double)msg.legs[2].joints[i].omega;
                rb_joint_vel[i] = (double)msg.legs[3].joints[i].omega;

                lf_joint_torque[i] = (double)msg.legs[0].joints[i].torque;
                rf_joint_torque[i] = (double)msg.legs[1].joints[i].torque;
                lb_joint_torque[i] = (double)msg.legs[2].joints[i].torque;
                rb_joint_torque[i] = (double)msg.legs[3].joints[i].torque;
            }
            legs_data_updated=true;
        });

    // 订阅机器人的运动期望
    move_cmd_sub = node_->create_subscription<robot_interfaces::msg::MoveCmd>(
        "robot_move_cmd", 10, [this](const robot_interfaces::msg::MoveCmd& msg) { move_cmd = msg; });

    // 订阅机器人URDF描述文件
    robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "/robot_state_publisher");
    auto params              = robot_description_param_->get_parameters({"robot_description"});
    urdf_xml                 = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "无法读取URDF文件，不能进行动力学计算");
        return;
    }

    kdl_parser::treeFromString(urdf_xml, tree); // 解析四条腿的KDL树结构
    // 注意：只提取到 link3，不包括 link4（轮子），因为我们只需要 3 个关节的运动学
    tree.getChain("body_link", "lf_link3", lf_leg_chain);
    tree.getChain("body_link", "rf_link3", rf_leg_chain);
    tree.getChain("body_link", "lb_link3", lb_leg_chain);
    tree.getChain("body_link", "rb_link3", rb_leg_chain);

    // 初始化狗腿解算器，定义足端中性点位置
    lf_leg_calc             = std::make_shared<LegCalc>(lf_leg_chain);
    rf_leg_calc             = std::make_shared<LegCalc>(rf_leg_chain);
    lb_leg_calc             = std::make_shared<LegCalc>(lb_leg_chain);
    rb_leg_calc             = std::make_shared<LegCalc>(rb_leg_chain);
    lf_leg_calc->pos_offset = lf_base_offset;
    rf_leg_calc->pos_offset = rf_base_offset;
    lb_leg_calc->pos_offset = lb_base_offset;
    rb_leg_calc->pos_offset = rb_base_offset;

    int result;
    comm_pos = get_grivate_center_pose(
        lf_leg_calc->joint_pos(lf_leg_stop_pos, &result), rf_leg_calc->joint_pos(rf_leg_stop_pos, &result),
        lb_leg_calc->joint_pos(lb_leg_stop_pos, &result), rb_leg_calc->joint_pos(rb_leg_stop_pos, &result));

    //注册状态机
    fsm.register_state(std::make_unique<IdelState>(this));
    fsm.register_state(std::make_unique<SetupState>(this));
    fsm.register_state(std::make_unique<StopState>(this));
    fsm.register_state(std::make_unique<WalkState>(this));
    fsm.register_state(std::make_unique<MPCState>(this));

    control_timer   = node->create_wall_timer(4ms, [this]() { if(legs_data_updated){fsm.run();} });
    ui_update_timer = node_->create_wall_timer(10ms, std::bind(&Robot::show_callback, this));
}

Robot::~Robot() {}

robot_interfaces::msg::Leg Robot::signal_leg_calc(
    const Vector3D& exp_cart_pos, const Vector3D& exp_cart_vel, const Vector3D& exp_cart_acc, const Vector3D& exp_cart_force,
    std::shared_ptr<LegCalc> leg_calc, Vector3D* torque, double wheel_vel, double wheel_force) {
    Vector3D joint_pos, joint_omega, joint_torque;

    int result;
    joint_pos    = leg_calc->joint_pos(exp_cart_pos, &result); // 一般这个位置不可能会迭代失败，所以不再对result进行处理
    joint_omega  = leg_calc->joint_vel(joint_pos, exp_cart_vel);
    joint_torque = leg_calc->joint_torque_foot_force(joint_pos, exp_cart_force);
    joint_torque += leg_calc->joint_torque_dynamic(joint_pos, joint_omega, exp_cart_acc);

    robot_interfaces::msg::Leg leg;
    for (int i = 0; i < 3; i++) {
        leg.joints[i].rad    = static_cast<float>(joint_pos[i]);
        leg.joints[i].omega  = static_cast<float>(joint_omega[i]);
        leg.joints[i].torque = static_cast<float>(joint_torque[i]);
    }
    leg.wheel.omega  = static_cast<float>(wheel_vel / WHEEL_RADIUS);
    leg.wheel.torque = static_cast<float>(wheel_force * WHEEL_RADIUS);
    *torque          = joint_torque;
    return leg;
}

void Robot::quaternionLowPassFilter(double& w, double& x, double& y, double& z, double w1, double x1, double y1, double z1, double alpha) {

    auto normalizeQuaternion = [](double& w, double& x, double& y, double& z) {
        double norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm < 1e-12) {
            // 退化情况，回到单位四元数
            w = 1.0;
            x = y = z = 0.0;
            return;
        }
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    };


    // 1. 单位化输入（防御性编程）
    normalizeQuaternion(w, x, y, z);
    normalizeQuaternion(w1, x1, y1, z1);

    // 2. 点积，判断是否需要取反（双覆盖问题）
    double dot = w * w1 + x * x1 + y * y1 + z * z1;
    if (dot < 0.0) {
        w1  = -w1;
        x1  = -x1;
        y1  = -y1;
        z1  = -z1;
        dot = -dot;
    }

    // 3. 小角度近似（避免 acos / sin 数值不稳定）
    const double DOT_THRESHOLD = 0.9995;
    if (dot > DOT_THRESHOLD) {
        // 线性插值
        w = w + alpha * (w1 - w);
        x = x + alpha * (x1 - x);
        y = y + alpha * (y1 - y);
        z = z + alpha * (z1 - z);
        normalizeQuaternion(w, x, y, z);
        return;
    }

    // 4. 标准 SLERP
    double theta     = std::acos(dot);
    double sin_theta = std::sin(theta);

    double w_a = std::sin((1.0 - alpha) * theta) / sin_theta;
    double w_b = std::sin(alpha * theta) / sin_theta;

    double w_new = w_a * w + w_b * w1;
    double x_new = w_a * x + w_b * x1;
    double y_new = w_a * y + w_b * y1;
    double z_new = w_a * z + w_b * z1;

    w = w_new;
    x = x_new;
    y = y_new;
    z = z_new;

    // 5. 再次单位化（强烈建议保留）
    normalizeQuaternion(w, x, y, z);
}

Vector3D Robot::get_grivate_center_pose(
    const Vector3D& lf_joint_pos, const Vector3D& rf_joint_pos, const Vector3D& lb_joint_pos, const Vector3D& rb_joint_pos) {
    // 使用 KDL 计算全身质心（在 body_link 坐标系下）
    // 注意：这里用各 link 的 RigidBodyInertia 来做质量加权平均。
    //      需要 URDF 中每个 link 都有 inertial 标签，否则质量会为 0。

    auto accumulate_chain_com = [](const KDL::Chain& chain, const Vector3D& q_eigen, double& mass_sum, KDL::Vector& com_sum) {
        KDL::JntArray q(chain.getNrOfJoints());
        for (unsigned int i = 0; i < chain.getNrOfJoints() && i < 3; ++i) {
            q(i) = q_eigen[i];
        }

        KDL::Frame T = KDL::Frame::Identity();

        unsigned int joint_idx = 0;
        for (unsigned int seg_idx = 0; seg_idx < chain.getNrOfSegments(); ++seg_idx) {
            const auto& seg = chain.getSegment(seg_idx);

            // 计算该段末端在基座下的位姿
            if (seg.getJoint().getType() != KDL::Joint::None) {
                T = T * seg.pose(q(joint_idx));
                joint_idx++;
            } else {
                T = T * seg.pose(0.0);
            }

            // 该 segment 的刚体惯量（在 segment 坐标系下）
            const KDL::RigidBodyInertia& rbi = seg.getInertia();
            const double m                   = rbi.getMass();
            if (m <= 0.0) {
                continue;
            }

            // COM 在 segment 坐标系下的位置
            const KDL::Vector c_seg = rbi.getCOG();
            // 转到基座坐标系
            const KDL::Vector c_base = T * c_seg;

            mass_sum += m;
            com_sum = com_sum + c_base * m;
        }
    };

    double mass_sum = 0.0;
    KDL::Vector com_sum(0.0, 0.0, 0.0);

    // 4 条腿分别从 body_link 到足端 link4；它们共享 body_link，但各自 inertia 不重复（body_link 本体要单独加）
    accumulate_chain_com(lf_leg_chain, lf_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(rf_leg_chain, rf_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(lb_leg_chain, lb_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(rb_leg_chain, rb_joint_pos, mass_sum, com_sum);

    // 加上 body_link 自身的质量与质心（树中 body_link 是 root，不在各腿 chain 的 segment 中）
    {
        const auto it = tree.getSegment("body_link");
        if (it != tree.getSegments().end()) {
            const auto body_seg = it->second.segment;
            const auto& rbi     = body_seg.getInertia();
            const double m      = rbi.getMass();
            if (m > 0.0) {
                mass_sum += m;
                com_sum = com_sum + rbi.getCOG() * m; // body_link 在 body_link 坐标系下
            }
        }
    }

    if (mass_sum <= 1e-9) {
        return Vector3D(0.0, 0.0, 0.0);
    }

    const KDL::Vector com = com_sum / mass_sum;
    return Vector3D(com.x(), com.y(), com.z());
}


void Robot::show_callback() {

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp            = node_->get_clock()->now();
    t.header.frame_id         = "world";
    t.child_frame_id          = "body_link";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = robot_rotation.x();
    t.transform.rotation.y = robot_rotation.y();
    t.transform.rotation.z = robot_rotation.z();
    t.transform.rotation.w = robot_rotation.w();

    robot_tf_broadcaster->sendTransform(t);

    visualization_msgs::msg::Marker com_marker;
    com_marker.header.frame_id = "body_link";
    com_marker.header.stamp    = node_->get_clock()->now();
    com_marker.ns              = "center_of_mass";
    com_marker.id              = 1;
    com_marker.type            = visualization_msgs::msg::Marker::SPHERE;
    com_marker.action          = visualization_msgs::msg::Marker::ADD;

    // 设置质心位置
    com_marker.pose.position.x    = comm_pos[0];
    com_marker.pose.position.y    = comm_pos[1];
    com_marker.pose.position.z    = comm_pos[2];
    com_marker.pose.orientation.x = 0.0;
    com_marker.pose.orientation.y = 0.0;
    com_marker.pose.orientation.z = 0.0;
    com_marker.pose.orientation.w = 1.0;

    // 设置球体尺寸
    com_marker.scale.x = 0.08;
    com_marker.scale.y = 0.08;
    com_marker.scale.z = 0.08;

    // 设置颜色 - 红色表示质心
    com_marker.color.a = 1.0;
    com_marker.color.r = 1.0;
    com_marker.color.g = 0.0;
    com_marker.color.b = 0.0;

    com_marker.lifetime = rclcpp::Duration(0, 0);


    const auto now = node_->get_clock()->now();

    visualization_msgs::msg::MarkerArray mark_array;

    auto make_force_arrow = [&](int id, const Vector3D& foot_pos, const Vector3D& pos_offset, const Vector3D& foot_force) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "body_link";
        m.header.stamp    = now;
        m.ns              = "foot_force";
        m.id              = id;
        m.type            = visualization_msgs::msg::Marker::ARROW;
        m.action          = visualization_msgs::msg::Marker::ADD;

        m.pose.orientation.w = 1.0;

        m.scale.x = 0.02;
        m.scale.y = 0.04;
        m.scale.z = 0.06;


        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = foot_pos[0] + pos_offset[0];
        p_start.y = foot_pos[1] + pos_offset[1];
        p_start.z = foot_pos[2] + pos_offset[2];

        p_end.x = p_start.x + foot_force[0] * 0.05;
        p_end.y = p_start.y + foot_force[1] * 0.05;
        p_end.z = p_start.z + foot_force[2] * 0.05;

        m.points.clear();
        m.points.push_back(p_start);
        m.points.push_back(p_end);

        m.color.a = 1.0;
        m.color.r = 1.0;
        m.color.g = 1.0;
        m.color.b = 0.0;

        m.lifetime = rclcpp::Duration(0, 0);

        mark_array.markers.push_back(m);
    };

    // LF
    make_force_arrow(
        0, lf_leg_calc->foot_pos(lf_joint_pos), lf_leg_calc->pos_offset,
        lf_leg_calc->foot_force(lf_joint_pos, lf_joint_torque, lf_forward_torque));

    // RF
    make_force_arrow(
        1, rf_leg_calc->foot_pos(rf_joint_pos), rf_leg_calc->pos_offset,
        rf_leg_calc->foot_force(rf_joint_pos, rf_joint_torque, rf_forward_torque));

    // LB
    make_force_arrow(
        2, lb_leg_calc->foot_pos(lb_joint_pos), lb_leg_calc->pos_offset,
        lb_leg_calc->foot_force(lb_joint_pos, lb_joint_torque, lb_forward_torque));

    // RB
    make_force_arrow(
        3, rb_leg_calc->foot_pos(rb_joint_pos), rb_leg_calc->pos_offset,
        rb_leg_calc->foot_force(rb_joint_pos, rb_joint_torque, rb_forward_torque));

    mark_array.markers.push_back(com_marker);

    marker_publisher->publish(mark_array);


    joint_display_msg.position[0] = lf_joint_pos[0];
    joint_display_msg.position[1] = lf_joint_pos[1];
    joint_display_msg.position[2] = lf_joint_pos[2];
    joint_display_msg.position[3] = 0.0;

    joint_display_msg.position[4] = rf_joint_pos[0];
    joint_display_msg.position[5] = rf_joint_pos[1];
    joint_display_msg.position[6] = rf_joint_pos[2];
    joint_display_msg.position[7] = 0.0;

    joint_display_msg.position[8]  = lb_joint_pos[0];
    joint_display_msg.position[9]  = lb_joint_pos[1];
    joint_display_msg.position[10] = lb_joint_pos[2];
    joint_display_msg.position[11] = 0.0;

    joint_display_msg.position[12] = rb_joint_pos[0];
    joint_display_msg.position[13] = rb_joint_pos[1];
    joint_display_msg.position[14] = rb_joint_pos[2];
    joint_display_msg.position[15] = 0.0;

    joint_display_msg.header.stamp = node_->get_clock()->now();
    rviz_joint_publisher->publish(joint_display_msg);

    // RCLCPP_INFO(node_->get_logger(), "roll:%lf,pitch=%lf", roll_offset_virtual_torque, pitch_offset_virtual_torque);

    // RCLCPP_INFO(node_->get_logger(), "kp=%lf,kd=%lf,mass=%lf", vmc->kp, vmc->kd, vmc->mass);
}

bool Robot::add_param_cb(std::function<bool(const rclcpp::Parameter& params)> callback) {
    param_cb_vector.push_back(callback);
    return true;
}

bool Robot::default_param_cb(const rclcpp::Parameter& param) {
    std::string name = param.get_name();

    // 处理Robot类的默认参数
    if (name == "horizontal_vmc_kp") {
        double value = param.as_double();
        lf_x_vmc->kp = value;
        lf_y_vmc->kp = value;
        rf_x_vmc->kp = value;
        rf_y_vmc->kp = value;
        lb_x_vmc->kp = value;
        lb_y_vmc->kp = value;
        rb_x_vmc->kp = value;
        rb_y_vmc->kp = value;
        return true;
    } else if (name == "horizontal_vmc_kd") {
        double value = param.as_double();
        lf_x_vmc->kd = value;
        lf_y_vmc->kd = value;
        rf_x_vmc->kd = value;
        rf_y_vmc->kd = value;
        lb_x_vmc->kd = value;
        lb_y_vmc->kd = value;
        rb_x_vmc->kd = value;
        rb_y_vmc->kd = value;
        return true;
    } else if (name == "horizontal_vmc_mass") {
        double value   = param.as_double();
        lf_x_vmc->mass = value;
        lf_y_vmc->mass = value;
        rf_x_vmc->mass = value;
        rf_y_vmc->mass = value;
        lb_x_vmc->mass = value;
        lb_y_vmc->mass = value;
        rb_x_vmc->mass = value;
        rb_y_vmc->mass = value;
        return true;
    } else if (name == "roll_vmc_kp") {
        roll_vmc->kp = param.as_double();
        return true;
    } else if (name == "roll_vmc_kd") {
        roll_vmc->kd = param.as_double();
        return true;
    } else if (name == "pitch_vmc_kp") {
        pitch_vmc->kp = param.as_double();
        return true;
    } else if (name == "pitch_vmc_kd") {
        pitch_vmc->kd = param.as_double();
        return true;
    } else if (name == "direction_filter_gate") {
        direction_filter_gate = param.as_double();
        direction_filter_gate = std::clamp(direction_filter_gate, 0.0, 1.0);
        return true;
    } else if (name == "vmc_kp") {
        lf_z_vmc->kp = param.as_double();
        rf_z_vmc->kp = param.as_double();
        lb_z_vmc->kp = param.as_double();
        rb_z_vmc->kp = param.as_double();
        return true;
    } else if (name == "vmc_kd") {
        lf_z_vmc->kd = param.as_double();
        rf_z_vmc->kd = param.as_double();
        lb_z_vmc->kd = param.as_double();
        rb_z_vmc->kd = param.as_double();
        return true;
    } else if (name == "vmc_mass") {
        lf_z_vmc->mass = param.as_double();
        rf_z_vmc->mass = param.as_double();
        lb_z_vmc->mass = param.as_double();
        rb_z_vmc->mass = param.as_double();
        return true;
    } else if (name == "lf_grivate") {
        robot_lf_grivate = param.as_double();
        return true;
    } else if (name == "rf_grivate") {
        robot_rf_grivate = param.as_double();
        return true;
    } else if (name == "lb_grivate") {
        robot_lb_grivate = param.as_double();
        return true;
    } else if (name == "rb_grivate") {
        robot_rb_grivate = param.as_double();
        return true;
    } else if (name == "lf_dx") {
        lf_base_offset[0] = 0.25 + param.as_double();
        return true;
    } else if (name == "rf_dx") {
        rf_base_offset[0] = 0.25 + param.as_double();
        return true;
    } else if (name == "lb_dx") {
        lb_base_offset[0] = -0.23 + param.as_double();
        return true;
    } else if (name == "rb_dx") {
        rb_base_offset[0] = -0.23 + param.as_double();
        return true;
    }

    // 未识别的参数
    return false;
}
