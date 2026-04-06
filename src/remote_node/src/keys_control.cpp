#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/move_cmd.hpp>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <cctype>
#include <algorithm>

class ProKeyboardTeleop : public rclcpp::Node {
public:
    ProKeyboardTeleop() : Node("pro_keyboard_teleop") {
        initKeyboard();

        // 统一一个话题
        pub_ = this->create_publisher<robot_interfaces::msg::MoveCmd>("robot_move_cmd", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ProKeyboardTeleop::update, this));

        last_time_ = this->now();
    }

    ~ProKeyboardTeleop() {
        restoreKeyboard();
    }

private:
    struct termios orig_termios;

    // 当前状态
    double vx_ = 0, vy_ = 0, vz_ = 0;

    // 目标值
    double target_vx_ = 0, target_vy_ = 0, target_vz_ = 0;

    int mode_ = 0;

    // 参数
    double accel_ = 3.5;
    double fast_accel_ = 4.0;
    double max_v_ = 0.5;
    double fast_max_v_ = 1.0;
    double decay_ = 3.0;

    rclcpp::Publisher<robot_interfaces::msg::MoveCmd>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;

    void initKeyboard() {
        tcgetattr(STDIN_FILENO, &orig_termios);
        struct termios new_termios = orig_termios;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }

    void restoreKeyboard() {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
    }

    void update() {
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        char c;
        bool key_pressed = false;

        double cur_accel = accel_;
        double cur_max_v = max_v_;

        // 默认目标为0，如果没有按键，则后面 decay 会回零
        target_vx_ = 0;
        target_vy_ = 0;
        target_vz_ = 0;

        // 读取按键
        while (read(STDIN_FILENO, &c, 1) > 0) {
            key_pressed = true;

            bool fast = (c >= 'A' && c <= 'Z'); // SHIFT
            if (fast) {
                cur_accel = fast_accel_;
                cur_max_v = fast_max_v_;
                c = std::tolower(c);
            }

            // 模式切换
            if (c >= '0' && c <= '9') {
                mode_ = c - '0';
                RCLCPP_INFO(this->get_logger(), "Mode -> %d", mode_);
                continue;
            }

            // 速度控制
            switch (c) {
                case 'w': target_vx_ =  cur_max_v; break;
                case 's': target_vx_ = -cur_max_v; break;
                case 'a': target_vy_ = -cur_max_v; break;
                case 'd': target_vy_ =  cur_max_v; break;
                case 'q': target_vz_ = -cur_max_v; break;
                case 'e': target_vz_ =  cur_max_v; break;
            }
        }

        // 平滑逼近目标
        vx_ = approach(vx_, target_vx_, cur_accel * dt);
        vy_ = approach(vy_, target_vy_, cur_accel * dt);
        vz_ = approach(vz_, target_vz_, cur_accel * dt);

        // 没按键 → 慢慢回零
        if (!key_pressed) {
            vx_ = approach(vx_, 0.0, decay_ * dt);
            vy_ = approach(vy_, 0.0, decay_ * dt);
            vz_ = approach(vz_, 0.0, decay_ * dt);
        }

        // 限幅
        vx_ = std::clamp(vx_, -cur_max_v, cur_max_v);
        vy_ = std::clamp(vy_, -cur_max_v, cur_max_v);
        vz_ = std::clamp(vz_, -cur_max_v, cur_max_v);

        // 发布
        publishCmd();
    }

    double approach(double v, double target, double step) {
        if (v < target) {
            v += step;
            if (v > target) v = target;
        } else if (v > target) {
            v -= step;
            if (v < target) v = target;
        }
        return v;
    }

    void publishCmd() {
        robot_interfaces::msg::MoveCmd msg;
        msg.vx = static_cast<float>(vx_);
        msg.vy = static_cast<float>(-vy_);
        msg.vz = static_cast<float>(-vz_);
        msg.step_mode = mode_;
        msg.vx = std::clamp(msg.vx,-1.0f,1.0f);
        msg.vy = std::clamp(msg.vy,-0.4f,0.4f);
        msg.vz = std::clamp(msg.vz,-1.0f,1.0f);
        pub_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProKeyboardTeleop>());
    rclcpp::shutdown();
    return 0;
}


// #include <rclcpp/rclcpp.hpp>
// #include <robot_interfaces/msg/move_cmd.hpp>

// #include <unistd.h>
// #include <termios.h>
// #include <fcntl.h>
// #include <cctype>
// #include <algorithm>
// #include <array>

// class ProKeyboardTeleop : public rclcpp::Node {
// public:
//     ProKeyboardTeleop() : Node("pro_keyboard_teleop") {
//         initKeyboard();

//         pub_ = this->create_publisher<robot_interfaces::msg::MoveCmd>("robot_move_cmd", 10);

//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(20),
//             std::bind(&ProKeyboardTeleop::update, this));

//         last_time_ = this->now();
//     }

//     ~ProKeyboardTeleop() {
//         restoreKeyboard();
//     }

// private:
//     struct termios orig_termios;

//     // 当前速度
//     double vx_ = 0, vy_ = 0, vz_ = 0;

//     // 目标速度
//     double target_vx_ = 0, target_vy_ = 0, target_vz_ = 0;

//     int mode_ = 0;

//     // 参数
//     double accel_ = 3.5;
//     double fast_accel_ = 4.0;

//     double max_v_ = 0.5;
//     double fast_max_v_ = 1.0;

//     double decay_ = 4.0;

//     // 按键保持时间
//     double hold_time_ = 0.15;

//     // W S A D Q E
//     std::array<double,6> key_time_{0,0,0,0,0,0};

//     rclcpp::Publisher<robot_interfaces::msg::MoveCmd>::SharedPtr pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Time last_time_;

//     void initKeyboard() {
//         tcgetattr(STDIN_FILENO, &orig_termios);

//         struct termios new_termios = orig_termios;
//         new_termios.c_lflag &= ~(ICANON | ECHO);

//         tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
//         fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
//     }

//     void restoreKeyboard() {
//         tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
//     }

//     void update() {
//         auto now = this->now();
//         double dt = (now - last_time_).seconds();
//         last_time_ = now;

//         char c;
//         double cur_accel = accel_;
//         double cur_max_v = max_v_;

//         // === 读取键盘 ===
//         while (read(STDIN_FILENO, &c, 1) > 0) {

//             bool fast = (c >= 'A' && c <= 'Z');
//             if (fast) {
//                 cur_accel = fast_accel_;
//                 cur_max_v = fast_max_v_;
//                 c = std::tolower(c);
//             }

//             // 模式切换（只触发一次）
//             if (c >= '0' && c <= '9') {
//                 int new_mode = c - '0';
//                 if (new_mode != mode_) {
//                     mode_ = new_mode;
//                     RCLCPP_INFO(this->get_logger(), "Mode -> %d", mode_);
//                 }
//                 continue;
//             }

//             // 按键记录（刷新保持时间）
//             switch(c){
//                 case 'w': key_time_[0] = hold_time_; break;
//                 case 's': key_time_[1] = hold_time_; break;
//                 case 'a': key_time_[2] = hold_time_; break;
//                 case 'd': key_time_[3] = hold_time_; break;
//                 case 'q': key_time_[4] = hold_time_; break;
//                 case 'e': key_time_[5] = hold_time_; break;
//             }
//         }

//         // === 按键时间衰减 ===
//         for(auto &t : key_time_) {
//             if(t > 0) t -= dt;
//         }

//         // === 判断是否“按住” ===
//         bool w = key_time_[0] > 0;
//         bool s = key_time_[1] > 0;
//         bool a = key_time_[2] > 0;
//         bool d = key_time_[3] > 0;
//         bool q = key_time_[4] > 0;
//         bool e = key_time_[5] > 0;

//         // === 组合控制（关键）===
//         target_vx_ = (w ? cur_max_v : 0) + (s ? -cur_max_v : 0);
//         target_vy_ = (d ? cur_max_v : 0) + (a ? -cur_max_v : 0);
//         target_vz_ = (e ? cur_max_v : 0) + (q ? -cur_max_v : 0);

//         // === 平滑逼近 ===
//         vx_ = approach(vx_, target_vx_, cur_accel * dt);
//         vy_ = approach(vy_, target_vy_, cur_accel * dt);
//         vz_ = approach(vz_, target_vz_, cur_accel * dt);

//         // === 无输入时自动回零 ===
//         if (!(w||s||a||d||q||e)) {
//             vx_ = approach(vx_, 0.0, decay_ * dt);
//             vy_ = approach(vy_, 0.0, decay_ * dt);
//             vz_ = approach(vz_, 0.0, decay_ * dt);
//         }

//         // === 限幅 ===
//         vx_ = std::clamp(vx_, -cur_max_v, cur_max_v);
//         vy_ = std::clamp(vy_, -cur_max_v, cur_max_v);
//         vz_ = std::clamp(vz_, -cur_max_v, cur_max_v);

//         publishCmd();
//     }

//     double approach(double v, double target, double step) {
//         if (v < target) {
//             v += step;
//             if (v > target) v = target;
//         } else if (v > target) {
//             v -= step;
//             if (v < target) v = target;
//         }
//         return v;
//     }

//     void publishCmd() {
//         robot_interfaces::msg::MoveCmd msg;

//         msg.vx = static_cast<float>(vx_);
//         msg.vy = static_cast<float>(-vy_);
//         msg.vz = static_cast<float>(-vz_);
//         msg.step_mode = mode_;

//         // 二次限幅（安全）
//         msg.vx = std::clamp(msg.vx, -1.0f, 1.0f);
//         msg.vy = std::clamp(msg.vy, -0.4f, 0.4f);
//         msg.vz = std::clamp(msg.vz, -1.0f, 1.0f);

//         pub_->publish(msg);
//     }
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ProKeyboardTeleop>());
//     rclcpp::shutdown();
//     return 0;
// }