#include "floor_heat_mapper/heatmap_publisher.hpp"

namespace floor_heat_mapper {
HeatmapPublisher::HeatmapPublisher()
    : rclcpp::Node(NODE_NAME) {
    static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    heatmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(HEATMAP_TOPIC_NAME, 10);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(MAP_TOPIC_NAME, 10, std::bind(&HeatmapPublisher::map_callback, this, _1));
    timer_ = this->create_wall_timer(100ms, std::bind(&HeatmapPublisher::update_heatmap, this));

    calculate_heatmap_resolution();
    create_thermal_camera_to_base_link_tf();
    create_heatmap_origin_to_thermal_camera_tf();
    create_heatmap_to_map_tf();
    RCLCPP_INFO(get_logger(), "Map constructed!");
}


void HeatmapPublisher::create_thermal_camera_to_base_link_tf() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = BASE_LINK_FRAME_NAME;
    t.child_frame_id = THERMAL_CAMERA_FRAME_NAME;
    t.transform.translation.x = THERMAL_CAMERA_FRAME_X;
    t.transform.translation.z = THERMAL_CAMERA_FRAME_Z;
    static_tf_pub_->sendTransform(t);
}

void HeatmapPublisher::create_heatmap_origin_to_thermal_camera_tf() {
    calculate_heatmap_resolution();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = THERMAL_CAMERA_FRAME_NAME;
    t.child_frame_id = HEATMAP_OFFSET_FRAME_NAME;
    t.transform.translation.x = -heatmap_msg_.info.resolution * IMAGE_HEIGHT/2;
    t.transform.translation.y = heatmap_msg_.info.resolution * IMAGE_WIDTH/2;
    t.transform.translation.z = -THERMAL_CAMERA_FRAME_Z;
    static_tf_pub_->sendTransform(t);
}

void HeatmapPublisher::create_heatmap_to_map_tf() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = HEATMAP_FRAME_NAME;
    t.child_frame_id = MAP_FRAME_NAME;
    static_tf_pub_->sendTransform(t);
}

void HeatmapPublisher::calculate_heatmap_resolution() {
    heatmap_msg_.info.resolution = tan(THERMAL_CAMERA_X_FOV_HALF_ANGLE) * THERMAL_CAMERA_FRAME_Z / 2 / IMAGE_WIDTH;
}

void HeatmapPublisher::update_heatmap() {
    heatmap_msg_.header.stamp = now();
    heatmap_msg_.header.frame_id = HEATMAP_FRAME_NAME;
    heatmap_pub_->publish(heatmap_msg_);
}

geometry_msgs::msg::Quaternion HeatmapPublisher::rotate_z_axis_by_angle(geometry_msgs::msg::Quaternion quaternion, double angle) {
    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(quaternion, q_orig);
    q_rot.setRPY(0.0, 0.0, angle);
    q_new = q_rot * q_orig;
    q_new.normalize();
    return tf2::toMsg(q_new);
}

void HeatmapPublisher::map_callback(const nav_msgs::msg::OccupancyGrid map_msg) {
    sync_heatmap_info_with_map(map_msg);
}

void HeatmapPublisher::sync_heatmap_info_with_map(const nav_msgs::msg::OccupancyGrid map_msg) {
    double real_size_x = map_msg.info.width * map_msg.info.resolution;
    double real_size_y = map_msg.info.height * map_msg.info.resolution;
    RCLCPP_INFO(get_logger(), "Map size x: %lf, y:%lf", real_size_x, real_size_y);
    heatmap_msg_.info.width = (real_size_x / heatmap_msg_.info.resolution);
    heatmap_msg_.info.height = (real_size_y / heatmap_msg_.info.resolution);
    heatmap_msg_.info.origin = map_msg.info.origin;
    heatmap_msg_.data.resize(heatmap_msg_.info.width * heatmap_msg_.info.height);
}

}  // namespace floor_heat_mapper