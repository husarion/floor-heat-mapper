#pragma once
#include <tf2/LinearMath/Matrix3x3.h>
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
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace floor_heat_mapper {

constexpr char NODE_NAME[] = "Map_Publisher";
constexpr char HEATMAP_TOPIC_NAME[] = "floor_heatmap";
constexpr char MAP_TOPIC_NAME[] = "map";
constexpr char THERMAL_CAMERA_TOPIC_NAME[] = "thermal_camera";

constexpr char MAP_FRAME_NAME[] = "map";
constexpr char ODOM_FRAME_NAME[] = "odom";
constexpr char BASE_LINK_FRAME_NAME[] = "base_link";
constexpr char HEATMAP_OFFSET_FRAME_NAME[] = "heatmap_offset";
constexpr char HEATMAP_FRAME_NAME[] = "floor_heatmap_frame";
constexpr char THERMAL_CAMERA_FRAME_NAME[] = "thermal_camera_frame";

constexpr float THERMAL_CAMERA_X_FOV_HALF_ANGLE = 0.9599310885968813;
constexpr float THERMAL_CAMERA_Y_FOV_HALF_ANGLE = 0.6544984694978736;

constexpr float THERMAL_CAMERA_FRAME_X = 0.225;
constexpr float THERMAL_CAMERA_FRAME_Z = 0.178;

constexpr uint8_t IMAGE_WIDTH = 16;
constexpr uint8_t IMAGE_HEIGHT = 12;

class HeatmapPublisher : public rclcpp::Node {
   public:
    HeatmapPublisher();

   private:
    void create_heatmap_to_map_tf();
    void create_thermal_camera_to_base_link_tf();
    void create_heatmap_origin_to_thermal_camera_tf();
    void calculate_heatmap_resolution();
    void update_heatmap();
    void map_callback(const nav_msgs::msg::OccupancyGrid map_msg);
    void thermal_camera_callback(const sensor_msgs::msg::Image image_msg);
    void sync_heatmap_info_with_map(const nav_msgs::msg::OccupancyGrid map_msg);
    void timer_callback();
    void take_heatmap_offset_to_map_transform();

    geometry_msgs::msg::Quaternion rotate_z_axis_by_angle(geometry_msgs::msg::Quaternion quaternion, double angle);

    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid heatmap_msg_;
    geometry_msgs::msg::TransformStamped thermal_camera_to_map_transform_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr heatmap_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermal_camera_image_sub_;

    cv::Mat single_thermal_image_;
    // cv::Mat heatmap_image_;
};
}  // namespace floor_heat_mapper