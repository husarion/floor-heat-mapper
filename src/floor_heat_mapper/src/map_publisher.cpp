#include "floor_heat_mapper/map_publisher.hpp"

namespace floor_heat_mapper {
MapPublisher::MapPublisher()
    : rclcpp::Node(NODE_NAME) {
    heatmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(MAP_TOPIC_NAME, 10);
    static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(100ms, std::bind(&MapPublisher::timer_callback, this));


    create_heatmap_to_map_tf();
    create_heatmap_settings();
    create_thermal_camera_to_base_link_tf();
    RCLCPP_INFO(get_logger(), "Map constructed!");
}

void MapPublisher::initialize_heatmap_origin_from_base_link() {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    geometry_msgs::msg::TransformStamped map_to_thermal_camera_tf;

    bool found_transformation = false;
    do {
        try {
            map_to_thermal_camera_tf = tf_buffer_->lookupTransform(MAP_FRAME_NAME, THERMAL_CAMERA_FRAME_NAME, tf2::TimePointZero);
            found_transformation = true;
        } catch (tf2::TransformException& ex) {
            RCLCPP_INFO(get_logger(), "Could not find transform base_link to map: %s", ex.what());
        }
    } while (not found_transformation);

    heatmap_msg_.info.origin.position.x = map_to_thermal_camera_tf.transform.translation.x;
    heatmap_msg_.info.origin.position.y = map_to_thermal_camera_tf.transform.translation.y;
    heatmap_msg_.info.origin.orientation = map_to_thermal_camera_tf.transform.rotation;

    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(map_to_thermal_camera_tf.transform.rotation, q_orig);
    q_rot.setRPY(0.0, 0.0, -M_PI / 2);
    q_new = q_rot * q_orig;
    q_new.normalize();
    heatmap_msg_.info.origin.orientation = tf2::toMsg(q_new);
}

void MapPublisher::create_thermal_camera_to_base_link_tf() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = BASE_LINK_FRAME_NAME;
    t.child_frame_id = THERMAL_CAMERA_FRAME_NAME;
    t.transform.translation.x = THERMAL_CAMERA_FRAME_X;
    t.transform.translation.z = THERMAL_CAMERA_FRAME_Z;
    static_tf_pub_->sendTransform(t);
}

void MapPublisher::create_heatmap_to_map_tf() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = HEATMAP_FRAME_NAME;
    t.child_frame_id = MAP_FRAME_NAME;
    static_tf_pub_->sendTransform(t);
}

void MapPublisher::create_heatmap_settings() {
    heatmap_msg_.info.resolution = tan(THERMAL_CAMERA_X_FOV_HALF_ANGLE) * THERMAL_CAMERA_FRAME_Z / 2 / IMAGE_WIDTH;
    heatmap_msg_.info.width = IMAGE_WIDTH;
    heatmap_msg_.info.height = IMAGE_HEIGHT;
    heatmap_msg_.data.resize(heatmap_msg_.info.width * heatmap_msg_.info.height);

    initialize_heatmap_origin_from_base_link();
    RCLCPP_INFO(get_logger(), "Map Publisher, ready to publish. Resolution: %f", heatmap_msg_.info.resolution);
}

void MapPublisher::timer_callback() {
    heatmap_msg_.header.stamp = now();
    heatmap_msg_.header.frame_id = HEATMAP_FRAME_NAME;
    heatmap_pub_->publish(heatmap_msg_);
}
}  // namespace floor_heat_mapper