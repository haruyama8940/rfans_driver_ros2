#include <rclcpp/rclcpp.hpp>
#include "rfans_driver/rfans_driver.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rfans_driver");
    // auto nh = std::make_shared<rclcpp::Node>("~");
    auto driver = std::make_shared<rfans_driver::Rfans_Driver>(node);

    while (rclcpp::ok() && driver->spinOnce())
    {
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}