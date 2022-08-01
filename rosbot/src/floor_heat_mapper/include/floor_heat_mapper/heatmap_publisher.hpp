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
#include <limits>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "cv_bridge/cv_bridge.h"

namespace floor_heat_mapper {

constexpr const char* NODE_NAME = "FloorHeatMapper";
constexpr char HEATMAP_TOPIC_NAME[] = "floor_heatmap";
constexpr char MAP_TOPIC_NAME[] = "map";
constexpr char THERMAL_CAMERA_TOPIC_NAME[] = "thermal_camera";
constexpr char MARKERS_TOPIC_NAME[] = "heatpoints";
constexpr const char* TAKE_THERMAL_IMAGE_SERVICE_NAME = "take_thermal_image";

constexpr char MAP_FRAME_NAME[] = "map";
constexpr char ODOM_FRAME_NAME[] = "odom";
constexpr char BASE_LINK_FRAME_NAME[] = "base_link";
constexpr char THERMAL_CAMERA_FRAME_NAME[] = "thermal_camera_frame";

constexpr float THERMAL_CAMERA_X_FOV_HALF_ANGLE = 0.9599310885968813;
constexpr float THERMAL_CAMERA_Y_FOV_HALF_ANGLE = 0.6544984694978736;

constexpr float THERMAL_CAMERA_FRAME_X = 0.225;
constexpr float THERMAL_CAMERA_FRAME_Z = 0.178;

constexpr uint8_t IMAGE_WIDTH = 16;
constexpr uint8_t IMAGE_HEIGHT = 12;

constexpr double MAX_FLOOR_NORMALIZED_TEMPERATURE = 30.0;
constexpr double MIN_FLOOR_NORMALIZED_TEMPERATURE = 15.0;
constexpr double THERMAL_IMAGE_TEMPERATURE_SCALE = 0.1;
constexpr double COLOR_OCCUPACY_MAP_SCALE = 192.0;

class FloorHeatMapper : public rclcpp::Node {
   public:
    FloorHeatMapper();
    static void set_thermal_image_trigger();

   private:
    void create_pubs_subs_and_timers();
    void create_thermal_camera_to_base_link_tf();
    void calculate_heatmap_resolution();
    void update_heatmap();
    void map_callback(const nav_msgs::msg::OccupancyGrid map_msg);
    void thermal_camera_callback(const sensor_msgs::msg::Image image_msg);
    void sync_heatmap_info_with_map(const nav_msgs::msg::OccupancyGrid map_msg);
    void timer_callback();
    void take_thermal_camera_to_map_transform();
    void create_heatpoints();
    void mark_min_max_temperatures(cv::Mat image);
    void handle_parameters();
    bool merge_single_thermal_image_and_heatmap();
    static void take_thermal_image_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    cv::Mat create_image_from_heatmap();
    cv::Mat rotate_image(cv::Mat image);
    cv::Mat create_mask(cv::Mat image);
    cv::Mat normalize_and_check_min_max_temperatures(cv::Mat image);

    geometry_msgs::msg::Quaternion rotate_z_axis_by_angle(const geometry_msgs::msg::Quaternion& quaternion, const double angle) const;
    geometry_msgs::msg::Vector3 take_vector_to_image_center(const cv::Mat& image) const;

    bool sync_mode_ = false;
    bool async_mode_ = true;
    inline static bool heatmap_synced_ = false;
    inline static bool trigger_thermal_photo_ = false;
    inline static bool thermal_image_synced_ = false;

    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid heatmap_msg_;
    geometry_msgs::msg::TransformStamped thermal_camera_to_map_transform_;
    geometry_msgs::msg::Point* hottest_point_;
    geometry_msgs::msg::Point* coldest_point_;
    visualization_msgs::msg::Marker heatpoints_cube_marker_;
    cv_bridge::CvImagePtr cv_bridge_with_single_thermal_image_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr heatmap_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr heatpoints_marker_pub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermal_camera_image_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr take_thermal_image_srv_;

    std::size_t markers_count_ = 0;
    bool is_map_synced_{false};
    double hottest_temperature_ = std::numeric_limits<double>::min();
    double coldest_temperature_ = std::numeric_limits<double>::max();
};
}  // namespace floor_heat_mapper