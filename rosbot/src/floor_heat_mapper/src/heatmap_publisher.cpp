#include "floor_heat_mapper/heatmap_publisher.hpp"

namespace floor_heat_mapper {
HeatmapPublisher::HeatmapPublisher()
    : rclcpp::Node(NODE_NAME), single_thermal_image_(IMAGE_WIDTH, IMAGE_HEIGHT, CV_8UC1) {
    static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    heatmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(HEATMAP_TOPIC_NAME, 10);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(MAP_TOPIC_NAME, 10, std::bind(&HeatmapPublisher::map_callback, this, _1));
    thermal_camera_image_sub_ = create_subscription<sensor_msgs::msg::Image>(THERMAL_CAMERA_TOPIC_NAME, 10, std::bind(&HeatmapPublisher::thermal_camera_callback, this, _1));
    timer_ = this->create_wall_timer(100ms, std::bind(&HeatmapPublisher::timer_callback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
    t.transform.translation.x = -heatmap_msg_.info.resolution * IMAGE_HEIGHT / 2;
    t.transform.translation.y = heatmap_msg_.info.resolution * IMAGE_WIDTH / 2;
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

void HeatmapPublisher::timer_callback() {
    take_heatmap_offset_to_map_transform();
    update_heatmap();
}

void HeatmapPublisher::take_heatmap_offset_to_map_transform() {
    try {
        thermal_camera_to_map_transform_ = tf_buffer_->lookupTransform(
            HEATMAP_OFFSET_FRAME_NAME, HEATMAP_FRAME_NAME,
            tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
        RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s",
                    MAP_FRAME_NAME, THERMAL_CAMERA_FRAME_NAME, ex.what());
        return;
    }
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
    auto old_heatmap_x  = heatmap_msg_.info.width;
    auto old_heatmap_y = heatmap_msg_.info.height;

    heatmap_msg_.info.width = (real_size_x / heatmap_msg_.info.resolution);
    heatmap_msg_.info.height = (real_size_y / heatmap_msg_.info.resolution);
    heatmap_msg_.info.origin = map_msg.info.origin;
    heatmap_msg_.data.resize(heatmap_msg_.info.width * heatmap_msg_.info.height);

    for (auto i = old_heatmap_x * old_heatmap_y; i < heatmap_msg_.info.width * heatmap_msg_.info.height; ++i){
        heatmap_msg_.data[i] = -1;
    }
}

void HeatmapPublisher::thermal_camera_callback(const sensor_msgs::msg::Image image_msg) {
    // single_thermal_image_ = std::make_shared<cv::Mat>(IMAGE_WIDTH, IMAGE_HEIGHT, CV_8UC3, image_msg.data);
    if (heatmap_msg_.info.width == 0 or heatmap_msg_.info.height == 0) {
        RCLCPP_INFO(get_logger(), "Skipping creating image...");
        return;
    }

    RCLCPP_INFO(get_logger(), "Creating image...");

    for (auto i = 0u; i < IMAGE_WIDTH; i++) {
        for (auto j = 0u; j < IMAGE_HEIGHT; j++) {
            single_thermal_image_.data[i * IMAGE_HEIGHT + j] = image_msg.data[j * IMAGE_WIDTH + i];
        }
    }

    cv::Mat heatmap_image(heatmap_msg_.info.height, heatmap_msg_.info.width, CV_8UC1);
    RCLCPP_INFO(get_logger(), "Heatmap image size: x: %d, y: %d", heatmap_msg_.info.width, heatmap_msg_.info.height);
    RCLCPP_INFO(get_logger(), "Copy thermal image...");

    single_thermal_image_.copyTo(heatmap_image(cv::Rect(0, 0, single_thermal_image_.cols, single_thermal_image_.rows)));
    for (auto i = 0u; i < heatmap_msg_.info.height * heatmap_msg_.info.width; ++i) {
        if (heatmap_image.data[i] == 255){
            heatmap_msg_.data[i] = -1;
        }
        else{
            heatmap_msg_.data[i] = heatmap_image.data[i] * 100 / 256;
        }
    }
}

}  // namespace floor_heat_mapper