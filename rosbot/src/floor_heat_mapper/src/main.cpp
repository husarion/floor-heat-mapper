#include "floor_heat_mapper/heatmap_publisher.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<floor_heat_mapper::HeatmapPublisher>());
    rclcpp::shutdown();
    return 0;
}