#include "floor_heat_mapper/map_publisher.hpp"

namespace floor_heat_mapper {
MapPublisher::MapPublisher()
    : rclcpp::Node(NODE_NAME) {
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(MAP_TOPIC_NAME, 10);
    tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&MapPublisher::timer_callback, this));

    map_msg_.info.resolution = 0.1;
    map_msg_.info.width = 16;
    map_msg_.info.height = 12;
    map_msg_.data.resize(map_msg_.info.width * map_msg_.info.height);
    RCLCPP_INFO(get_logger(), "Map Publisher, ready to publish.");
    for (auto i = 0u; i < 12 * 16; ++i){
        map_msg_.data[i] = -1;
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = MAP_FRAME_NAME;
    t.child_frame_id = "map";
    tf_pub_->sendTransform(t);
}

void MapPublisher::timer_callback(){
    for (auto i = 0u; i < 12 * 8; ++i) {
        map_msg_.data[i] = i;
    }
    map_msg_.header.stamp = now();
    map_msg_.header.frame_id = MAP_FRAME_NAME;

    map_pub_->publish(map_msg_);
}
}  // namespace floor_heat_mapper