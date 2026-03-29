#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "imu_driver/imu_driver.hpp"


int main(int argc,char**argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("imu_driver_node");
    IMUDriver driver(
        node,
        "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0003-if00-port0",
        921600);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
