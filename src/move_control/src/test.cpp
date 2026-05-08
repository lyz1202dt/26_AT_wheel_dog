#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <robot_interfaces/msg/move_cmd.hpp>
#include <robot_interfaces/msg/jump_cmd.hpp>
#include <memory>
#include <chrono>
#include <random>

using namespace std::chrono_literals;

class TestMoveNode : public rclcpp::Node{
public:
    TestMoveNode():  Node("test_move_node")
    {
        // declare parameters so rqt param plugin can modify them
        auto vx_descriptor = rcl_interfaces::msg::ParameterDescriptor();
        vx_descriptor.description = "Velocity in X direction";
        vx_descriptor.floating_point_range.resize(1);
        vx_descriptor.floating_point_range[0].from_value = -1.0;
        vx_descriptor.floating_point_range[0].to_value = 1.0;
        this->declare_parameter<double>("vx", 0.0, vx_descriptor);

        auto vy_descriptor = rcl_interfaces::msg::ParameterDescriptor();
        vy_descriptor.description = "Velocity in Y direction";
        vy_descriptor.floating_point_range.resize(1);
        vy_descriptor.floating_point_range[0].from_value = -1.0;
        vy_descriptor.floating_point_range[0].to_value = 1.0;
        this->declare_parameter<double>("vy", 0.0, vy_descriptor);

        auto vz_descriptor = rcl_interfaces::msg::ParameterDescriptor();
        vz_descriptor.description = "Velocity in Z direction";
        vz_descriptor.floating_point_range.resize(1);
        vz_descriptor.floating_point_range[0].from_value = -1.0;
        vz_descriptor.floating_point_range[0].to_value = 1.0;
        this->declare_parameter<double>("vz", 0.0, vz_descriptor);


        this->declare_parameter<int>("step_type", 0);

        // Jump parameters
        this->declare_parameter<double>("v0", 2.0);
        this->declare_parameter<double>("v0_dir", 0.0);
        this->declare_parameter<double>("ready_jump_height", 0.13);
        this->declare_parameter<double>("finished_jump_height", 0.31);
        this->declare_parameter<double>("fly_height", 0.15);
        this->declare_parameter<double>("touch_height", 0.20);
        this->declare_parameter<double>("t1", 2.0);
        this->declare_parameter<double>("t2", 0.1);
        this->declare_parameter<double>("t3", 0.12);

        auto trigger_desc = rcl_interfaces::msg::ParameterDescriptor();
        trigger_desc.description = "Trigger jump command (触发跳跃命令)";
        this->declare_parameter<bool>("trigger_jump", false, trigger_desc);

        // create publishers
        pub_ = this->create_publisher<robot_interfaces::msg::MoveCmd>("robot_move_cmd", 10);
        jump_pub_ = this->create_publisher<robot_interfaces::msg::JumpCmd>("jump_cmd", 10);

        // set up parameter change callback: publish a new MoveCmd whenever parameters change
        param_cb_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params){
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto & p : params) {
                    if (p.get_name() == "vx") {
                        if (p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER) {
                            vx_ = static_cast<float>(p.as_double());
                        } else {
                            result.successful = false;
                            result.reason = "vx must be a number";
                            return result;
                        }
                    } else if (p.get_name() == "vy") {
                        if (p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER) {
                            vy_ = static_cast<float>(p.as_double());
                        } else {
                            result.successful = false;
                            result.reason = "vy must be a number";
                            return result;
                        }
                    } else if (p.get_name() == "vz") {
                        if (p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER) {
                            vz_ = static_cast<float>(p.as_double());
                        } else {
                            result.successful = false;
                            result.reason = "vz must be a number";
                            return result;
                        }
                    } else if (p.get_name() == "step_type") {
                        if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
                            step_type_ = static_cast<uint32_t>(p.as_int());
                        } else if (p.get_type() == rclcpp::PARAMETER_DOUBLE) {
                            // allow integers provided as double
                            step_type_ = static_cast<uint32_t>(p.as_double());
                        } else {
                            result.successful = false;
                            result.reason = "step_type must be an integer";
                            return result;
                        }
                    } else if (p.get_name() == "trigger_jump") {
                        bool trigger = p.as_bool();
                        if (trigger && !last_trigger_) {
                            // Rising edge detected, publish jump command
                            publish_jump_cmd();
                        }
                        last_trigger_ = trigger;
                    }
                }
                return result;
            }
        );

        // publish initial message based on default parameters
        vx_ = static_cast<float>(this->get_parameter("vx").as_double());
        vy_ = static_cast<float>(this->get_parameter("vy").as_double());
        vz_ = static_cast<float>(this->get_parameter("vz").as_double());
        step_type_ = static_cast<uint32_t>(this->get_parameter("step_type").as_int());
        //update_cmd_by_mode();
        publish_move_cmd();

        update_timer=this->create_wall_timer(100ms ,[this](){
            //update_cmd_by_mode();
            publish_move_cmd();
        });
    }

private:

    rclcpp::Time start_time_;
    bool running_ = false;

    void update_cmd_by_mode()
    {
        // 只有 step_type == 2 才执行
        if (step_type_ != 2)
        {
            running_ = false;
            vx_ = vy_ = vz_ = 0.0f;
            return;
        }

        if (!running_)
        {
            start_time_ = this->now();
            running_ = true;
        }

        double t = (this->now() - start_time_).seconds();

        if (t >= 120.0)
        {
            step_type_ = 1;
            vx_ = vy_ = vz_ = 0.0f;
            running_ = false;
            return;
        }

        int segment = static_cast<int>(t / 10.0);  // 0~11
        double phase_t = fmod(t, 10.0) / 10.0;     // 0~1

        int axis = segment / 4;   // 0=vx,1=vy,2=vz
        int phase = segment % 4;  // 0~3

        float amp = (axis == 1) ? 0.5f : 1.0f;

        float value = 0.0f;

        switch (phase)
        {
        case 0: value =  amp * phase_t; break;           // 0 → +
        case 1: value =  amp * (1 - phase_t); break;     // + → 0
        case 2: value = -amp * phase_t; break;           // 0 → -
        case 3: value = -amp * (1 - phase_t); break;     // - → 0
        }

        // 清零全部
        vx_ = vy_ = vz_ = 0.0f;

        // 只激活一个轴
        if (axis == 0) vx_ = value;
        if (axis == 1) vy_ = value;
        if (axis == 2) vz_ = value;
    }

    void publish_move_cmd()
    {
        robot_interfaces::msg::MoveCmd msg;
        msg.step_mode = step_type_;
        msg.wheel_vel = 0.0f; // unused by this test node
        msg.vx = vx_;
        msg.vy = vy_;
        msg.vz = vz_;
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published MoveCmd: step_type=%u vx=%.3f vy=%.3f vz=%.3f",
                    step_type_, vx_, vy_, vz_);
    }

    void publish_jump_cmd()
    {
        robot_interfaces::msg::JumpCmd msg;
        msg.stamp = this->now();
        msg.v0 = this->get_parameter("v0").as_double();
        msg.v0_dir=this->get_parameter("v0_dir").as_double();
        msg.ready_jump_height = this->get_parameter("ready_jump_height").as_double();
        msg.finished_jump_height = this->get_parameter("finished_jump_height").as_double();
        msg.fly_height = this->get_parameter("fly_height").as_double();
        msg.touch_height = this->get_parameter("touch_height").as_double();
        msg.t1 = this->get_parameter("t1").as_double();
        msg.t2 =this->get_parameter("t2").as_double();
        msg.t3 = this->get_parameter("t3").as_double();
        jump_pub_->publish(msg);
    }

    rclcpp::Publisher<robot_interfaces::msg::MoveCmd>::SharedPtr pub_;
    rclcpp::Publisher<robot_interfaces::msg::JumpCmd>::SharedPtr jump_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    rclcpp::TimerBase::SharedPtr update_timer;

    float vx_{0.0f}, vy_{0.0f}, vz_{0.0f};
    uint32_t step_type_{0};
    
    // Jump parameters
    bool last_trigger_{false};
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TestMoveNode>());
    rclcpp::shutdown();
    return 0;
}
