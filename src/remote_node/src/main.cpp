#include <rclcpp/rclcpp.hpp>
#include "remote_node/remote_node.hpp"
#include <memory>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RemoteNode>();
    
    // 注意：如需从参数覆盖串口配置，可在此处添加参数处理
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}