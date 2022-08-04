#include "floor_heat_mapper/floor_heat_mapper.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<floor_heat_mapper::FloorHeatMapper>());
    rclcpp::shutdown();
    return 0;
}