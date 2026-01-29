#include <rclcpp/rclcpp.hpp>
#include "turtle_ekf/ekf_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtle_ekf::EKFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}