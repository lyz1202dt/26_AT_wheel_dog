#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <robot_interfaces/msg/move_cmd.hpp>
#include <robot_interfaces/msg/jump_cmd.hpp>
#include <memory>
#include <chrono>

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
        auto ready_height_desc = rcl_interfaces::msg::ParameterDescriptor();
        ready_height_desc.description = "Ready jump height (准备跳跃时的狗身高度)";
        ready_height_desc.floating_point_range.resize(1);
        ready_height_desc.floating_point_range[0].from_value = 0.0;
        ready_height_desc.floating_point_range[0].to_value = 0.5;
        this->declare_parameter<double>("ready_jump_height", 0.15, ready_height_desc);

        auto finished_height_desc = rcl_interfaces::msg::ParameterDescriptor();
        finished_height_desc.description = "Finished jump height (起跳动作完成时的狗身高度)";
        finished_height_desc.floating_point_range.resize(1);
        finished_height_desc.floating_point_range[0].from_value = 0.0;
        finished_height_desc.floating_point_range[0].to_value = 0.5;
        this->declare_parameter<double>("finished_jump_height", 0.25, finished_height_desc);

        auto fly_height_desc = rcl_interfaces::msg::ParameterDescriptor();
        fly_height_desc.description = "Fly height (飞行过程中足端到狗身的高度)";
        fly_height_desc.floating_point_range.resize(1);
        fly_height_desc.floating_point_range[0].from_value = 0.0;
        fly_height_desc.floating_point_range[0].to_value = 0.5;
        this->declare_parameter<double>("fly_height", 0.30, fly_height_desc);

        auto touch_height_desc = rcl_interfaces::msg::ParameterDescriptor();
        touch_height_desc.description = "Touch height (落地前用于缓冲的高度)";
        touch_height_desc.floating_point_range.resize(1);
        touch_height_desc.floating_point_range[0].from_value = 0.0;
        touch_height_desc.floating_point_range[0].to_value = 0.5;
        this->declare_parameter<double>("touch_height", 0.20, touch_height_desc);

        auto t1_desc = rcl_interfaces::msg::ParameterDescriptor();
        t1_desc.description = "Time t1 (从ready到finished的时间)";
        t1_desc.floating_point_range.resize(1);
        t1_desc.floating_point_range[0].from_value = 0.0;
        t1_desc.floating_point_range[0].to_value = 5.0;
        this->declare_parameter<double>("t1", 0.5, t1_desc);

        auto t2_desc = rcl_interfaces::msg::ParameterDescriptor();
        t2_desc.description = "Time t2 (从finished到fly的时间)";
        t2_desc.floating_point_range.resize(1);
        t2_desc.floating_point_range[0].from_value = 0.0;
        t2_desc.floating_point_range[0].to_value = 5.0;
        this->declare_parameter<double>("t2", 0.3, t2_desc);

        auto t3_desc = rcl_interfaces::msg::ParameterDescriptor();
        t3_desc.description = "Time t3 (从fly到touch的时间)";
        t3_desc.floating_point_range.resize(1);
        t3_desc.floating_point_range[0].from_value = 0.0;
        t3_desc.floating_point_range[0].to_value = 5.0;
        this->declare_parameter<double>("t3", 0.4, t3_desc);

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
                    } else if (p.get_name() == "ready_jump_height") {
                        ready_jump_height_ = p.as_double();
                    } else if (p.get_name() == "finished_jump_height") {
                        finished_jump_height_ = p.as_double();
                    } else if (p.get_name() == "fly_height") {
                        fly_height_ = p.as_double();
                    } else if (p.get_name() == "touch_height") {
                        touch_height_ = p.as_double();
                    } else if (p.get_name() == "t1") {
                        t1_ = p.as_double();
                    } else if (p.get_name() == "t2") {
                        t2_ = p.as_double();
                    } else if (p.get_name() == "t3") {
                        t3_ = p.as_double();
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
        publish_move_cmd();

        update_timer=this->create_wall_timer(100ms ,[this](){
            publish_move_cmd();
        });
    }

private:
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
        msg.v0 = 0.0;  // Initial velocity
        msg.ready_jump_height = ready_jump_height_;
        msg.finished_jump_height = finished_jump_height_;
        msg.fly_height = fly_height_;
        msg.touch_height = touch_height_;
        msg.t1 = t1_;
        msg.t2 = t2_;
        msg.t3 = t3_;
        
        jump_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), 
                    "Published JumpCmd: ready=%.3f finished=%.3f fly=%.3f touch=%.3f t1=%.3f t2=%.3f t3=%.3f",
                    ready_jump_height_, finished_jump_height_, fly_height_, touch_height_, 
                    t1_, t2_, t3_);
    }

    rclcpp::Publisher<robot_interfaces::msg::MoveCmd>::SharedPtr pub_;
    rclcpp::Publisher<robot_interfaces::msg::JumpCmd>::SharedPtr jump_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    rclcpp::TimerBase::SharedPtr update_timer;

    float vx_{0.0f}, vy_{0.0f}, vz_{0.0f};
    uint32_t step_type_{0};
    
    // Jump parameters
    double ready_jump_height_{0.15};
    double finished_jump_height_{0.25};
    double fly_height_{0.30};
    double touch_height_{0.20};
    double t1_{0.5};
    double t2_{0.3};
    double t3_{0.4};
    bool last_trigger_{false};
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TestMoveNode>());
    rclcpp::shutdown();
    return 0;
}
