#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "imu_driver/imu_driver.hpp"


int main(int argc,char**argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("imu_driver_node");
    IMUDriver driver(node,"/dev/ttyUSB0", 921600);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
