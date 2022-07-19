#pragma once
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace floor_heat_mapper {

constexpr char NODE_NAME[] = "Map_Publisher";
constexpr char MAP_TOPIC_NAME[] = "floor_heatmap";
constexpr char MAP_FRAME_NAME[] = "floor_heatmap";

class MapPublisher : public rclcpp::Node {
   public:
    MapPublisher();

   private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid map_msg_;
    std::vector<int8_t> map_data_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_pub_;
};
}  // namespace floor_heat_mapper