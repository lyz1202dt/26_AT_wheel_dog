#include <rclcpp/rclcpp.hpp>
#include "remote_node/remote_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteNode>();
    
    RCLCPP_INFO(node->get_logger(), "遥控器驱动节点已启动，等待接收遥控器数据...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}