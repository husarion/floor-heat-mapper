#pragma once
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace floor_heat_mapper {

constexpr char NODE_NAME[] = "Map_Publisher";
constexpr char MAP_TOPIC_NAME[] = "floor_heatmap";
constexpr char MAP_FRAME_NAME[] = "map";
constexpr char ODOM_FRAME_NAME[] = "odom";
constexpr char BASE_LINK_FRAME_NAME[] = "base_link";

constexpr char HEATMAP_FRAME_NAME[] = "floor_heatmap_frame";
constexpr char THERMAL_CAMERA_FRAME_NAME[] = "thermal_camera_frame";

constexpr float THERMAL_CAMERA_X_FOV_HALF_ANGLE = 0.9599310885968813;
constexpr float THERMAL_CAMERA_Y_FOV_HALF_ANGLE = 0.6544984694978736;

constexpr float THERMAL_CAMERA_FRAME_X = 0.225;
constexpr float THERMAL_CAMERA_FRAME_Z = 0.178;

constexpr uint8_t IMAGE_WIDTH = 16;
constexpr uint8_t IMAGE_HEIGHT = 8;

class MapPublisher : public rclcpp::Node {
   public:
    MapPublisher();

   private:
    void initialize_heatmap_origin_from_base_link();
    void create_heatmap_to_map_tf();
    void create_thermal_camera_to_base_link_tf();
    void create_heatmap_settings();
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid heatmap_msg_;
    std::vector<int8_t> heatmap_data_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr heatmap_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
};
}  // namespace floor_heat_mapper