#include "floor_heat_mapper/heatmap_publisher.hpp"

namespace floor_heat_mapper {
FloorHeatMapper::FloorHeatMapper()
    : rclcpp::Node(NODE_NAME), single_thermal_image_(IMAGE_WIDTH, IMAGE_HEIGHT, CV_16UC1) {
    static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    heatmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(HEATMAP_TOPIC_NAME, 10);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(MAP_TOPIC_NAME, 10, std::bind(&FloorHeatMapper::map_callback, this, _1));
    thermal_camera_image_sub_ = create_subscription<sensor_msgs::msg::Image>(THERMAL_CAMERA_TOPIC_NAME, 10, std::bind(&FloorHeatMapper::thermal_camera_callback, this, _1));
    timer_ = this->create_wall_timer(100ms, std::bind(&FloorHeatMapper::timer_callback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    calculate_heatmap_resolution();
    create_thermal_camera_to_base_link_tf();
    RCLCPP_INFO(get_logger(), "Configured mapping, wainting for /%s and /%s...", THERMAL_CAMERA_TOPIC_NAME, MAP_TOPIC_NAME);
}

void FloorHeatMapper::create_thermal_camera_to_base_link_tf() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = BASE_LINK_FRAME_NAME;
    t.child_frame_id = THERMAL_CAMERA_FRAME_NAME;
    t.transform.translation.x = THERMAL_CAMERA_FRAME_X;
    t.transform.translation.z = THERMAL_CAMERA_FRAME_Z;
    static_tf_pub_->sendTransform(t);
}

void FloorHeatMapper::calculate_heatmap_resolution() {
    heatmap_msg_.info.resolution = tan(THERMAL_CAMERA_X_FOV_HALF_ANGLE) * THERMAL_CAMERA_FRAME_Z / 2 / IMAGE_WIDTH;
}

void FloorHeatMapper::timer_callback() {
    take_thermal_camera_to_map_transform();
    update_heatmap();
}

void FloorHeatMapper::take_thermal_camera_to_map_transform() {
    try {
        thermal_camera_to_map_transform_ = tf_buffer_->lookupTransform(
            MAP_FRAME_NAME, THERMAL_CAMERA_FRAME_NAME,
            tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
        return;
    }
}

void FloorHeatMapper::update_heatmap() {
    heatmap_msg_.header.stamp = now();
    heatmap_msg_.header.frame_id = MAP_FRAME_NAME;
    heatmap_pub_->publish(heatmap_msg_);
}

geometry_msgs::msg::Quaternion FloorHeatMapper::rotate_z_axis_by_angle(const geometry_msgs::msg::Quaternion& quaternion, double angle) const {
    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(quaternion, q_orig);
    q_rot.setRPY(0.0, 0.0, angle);
    q_new = q_rot * q_orig;
    q_new.normalize();
    return tf2::toMsg(q_new);
}

geometry_msgs::msg::Vector3 FloorHeatMapper::take_vector_to_image_center(const cv::Mat& image) const {
    tf2::Quaternion q_orig, q_rot, q_new;
    q_orig.setRPY(0, 0, 0);
    tf2::convert(thermal_camera_to_map_transform_.transform.rotation, q_rot);
    q_new = q_rot * q_orig;
    q_new.normalize();
    tf2::Matrix3x3 m(q_new);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    geometry_msgs::msg::Vector3 ros_vec;
    ros_vec.y = (image.rows - 1) * heatmap_msg_.info.resolution / 2.0;
    ros_vec.x = (image.cols - 1) * heatmap_msg_.info.resolution / 2.0;
    tf2::Vector3 tf2_vec;
    tf2::convert(ros_vec, tf2_vec);
    tf2_vec.rotate({0, 0, 1}, yaw);
    return tf2::toMsg(tf2_vec);
}

void FloorHeatMapper::map_callback(const nav_msgs::msg::OccupancyGrid map_msg) {
    sync_heatmap_info_with_map(map_msg);
}

void FloorHeatMapper::sync_heatmap_info_with_map(const nav_msgs::msg::OccupancyGrid map_msg) {
    RCLCPP_INFO(get_logger(), "Sync map info");
    double real_size_x = map_msg.info.width * map_msg.info.resolution;
    double real_size_y = map_msg.info.height * map_msg.info.resolution;
    RCLCPP_INFO(get_logger(), "Map size x: %lf, y:%lf", real_size_x, real_size_y);
    auto old_heatmap_x = heatmap_msg_.info.width;
    auto old_heatmap_y = heatmap_msg_.info.height;
    auto old_width = heatmap_msg_.info.width;
    auto old_height = heatmap_msg_.info.height;

    heatmap_msg_.info.width = (real_size_x / heatmap_msg_.info.resolution);
    heatmap_msg_.info.height = (real_size_y / heatmap_msg_.info.resolution);
    heatmap_msg_.info.origin = map_msg.info.origin;

    if (old_heatmap_x * old_heatmap_y != heatmap_msg_.info.width * heatmap_msg_.info.height) {
        RCLCPP_INFO(get_logger(), "Resizeing map...");
        auto last_heat_map = heatmap_msg_;
        heatmap_msg_.data.resize(heatmap_msg_.info.width * heatmap_msg_.info.height);
        for (auto i = 0u; i < old_width; i++) {
            for (auto j = 0u; j < old_height; j++) {
                heatmap_msg_.data[i * IMAGE_HEIGHT + j] = last_heat_map.data[j * IMAGE_WIDTH + i];
            }
        }
        for (auto i = old_heatmap_x * old_heatmap_y; i < heatmap_msg_.info.width * heatmap_msg_.info.height; ++i) {
            heatmap_msg_.data[i] = -1;
        }
    }
    RCLCPP_INFO(get_logger(), "Map constructed!");
}

cv::Mat FloorHeatMapper::create_image_from_heatmap() {
    cv::Mat heatmap_image(heatmap_msg_.info.height, heatmap_msg_.info.width, CV_8UC1);
    for (auto i = 0u; i < heatmap_msg_.info.height * heatmap_msg_.info.width; ++i) {
        heatmap_image.data[i] = heatmap_msg_.data[i];
    }
    return heatmap_image;
}

cv::Mat FloorHeatMapper::rotate_image(cv::Mat image) {
    tf2::Quaternion q_orig, q_rot, q_new;
    q_orig.setRPY(0, 0, 0);
    tf2::convert(thermal_camera_to_map_transform_.transform.rotation, q_rot);
    q_new = q_rot * q_orig;
    q_new.normalize();
    tf2::Matrix3x3 m(q_new);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw *= -1 * 180 / M_PI;
    yaw += 90;
    RCLCPP_INFO(get_logger(), "Image rotate angle : %f", yaw);

    // Mirror y image
    cv::Mat mirrored;
    cv::flip(image, mirrored, 0);

    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), mirrored.size(), yaw).boundingRect2f();

    // Rotate cropped image
    cv::Mat cropped_image;
    cv::Rect crop_region(1, 1, mirrored.cols - 2, mirrored.rows - 2);
    cropped_image = mirrored(crop_region);

    cv::Point2f center_small((cropped_image.cols - 1) / 2.0, (cropped_image.rows - 1) / 2.0);
    cv::Mat rot_small = cv::getRotationMatrix2D(center_small, yaw, 1.0);
    rot_small.at<double>(0, 2) += bbox.width / 2.0 - cropped_image.cols / 2.0;
    rot_small.at<double>(1, 2) += bbox.height / 2.0 - cropped_image.rows / 2.0;

    cv::Mat small_rotated;
    cv::warpAffine(cropped_image, small_rotated, rot_small, bbox.size());

    // Rotate image
    cv::Mat without_noices;
    cv::Point2f center((mirrored.cols - 1) / 2.0, (mirrored.rows - 1) / 2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, yaw, 1.0);
    rot.at<double>(0, 2) += bbox.width / 2.0 - mirrored.cols / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - mirrored.rows / 2.0;
    cv::warpAffine(mirrored, without_noices, rot, bbox.size());

    // Compare crooped image and remove noices from edges
    for (auto i = 0u; i < without_noices.total(); ++i) {
        if (small_rotated.data[i] == 0) {
            without_noices.data[i] = 255;
        }
    }

    return without_noices;
}

cv::Mat FloorHeatMapper::create_mask(cv::Mat image) {
    cv::Mat mask = image.clone();
    for (auto i = 0u; i < mask.total(); ++i) {
        mask.data[i] = mask.data[i] ? 0 : 100;
    }
    return mask;
}

cv::Mat FloorHeatMapper::normalize_image_for_temperatures(cv::Mat image) {
    cv::Mat normalized = image.clone();

    for (auto i = 0u; i < normalized.rows; i++) {
        for (auto j = 0u; j < normalized.cols; j++) {
            auto pixel = normalized.at<uint16_t>(i, j);
            pixel = (static_cast<double>(pixel) - 150.0)* 150 / 200;
            normalized.at<uint16_t>(i, j) = pixel;
        }
    }
    std::cout << normalized << std::endl;
    return normalized;
}

void FloorHeatMapper::thermal_camera_callback(const sensor_msgs::msg::Image image_msg) {
    if (heatmap_msg_.info.width == 0 or heatmap_msg_.info.height == 0) {
        RCLCPP_INFO(get_logger(), "Waiting for /%s topic...", MAP_TOPIC_NAME);
        return;
    }
    if (heatmap_msg_.info.origin.position.x == 0 or heatmap_msg_.info.origin.position.y == 0) {
        RCLCPP_INFO(get_logger(), "Waiting for /%s topic...", MAP_TOPIC_NAME);

        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO16);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    auto heatmap_image = create_image_from_heatmap();
    cv::Mat normalized_image = normalize_image_for_temperatures(cv_ptr->image);

    normalized_image.convertTo(normalized_image, CV_8UC1, 1);
    auto rotated_single_thermal_image_ = rotate_image(normalized_image);

    auto to_image_center_vector = take_vector_to_image_center(rotated_single_thermal_image_);

    double real_position_dx = thermal_camera_to_map_transform_.transform.translation.x - heatmap_msg_.info.origin.position.x;
    double real_position_dy = thermal_camera_to_map_transform_.transform.translation.y - heatmap_msg_.info.origin.position.y;
    real_position_dx -= to_image_center_vector.x;
    real_position_dy -= to_image_center_vector.y;

    uint image_position_x = real_position_dx / heatmap_msg_.info.resolution;
    uint image_position_y = real_position_dy / heatmap_msg_.info.resolution;

    auto mask = create_mask(rotated_single_thermal_image_);

    rotated_single_thermal_image_.copyTo(heatmap_image(cv::Rect(image_position_x, image_position_y, rotated_single_thermal_image_.cols, rotated_single_thermal_image_.rows)), heatmap_image(cv::Rect(image_position_x, image_position_y, mask.cols, mask.rows)) == 255);
    for (auto i = 0u; i < heatmap_msg_.info.height * heatmap_msg_.info.width; ++i) {
        if (heatmap_image.data[i] == 255) {
            heatmap_msg_.data[i] = -1;
        } else if (heatmap_image.data[i] > 1) {
            heatmap_msg_.data[i] = (heatmap_image.data[i] + heatmap_msg_.data[i]) / 2;
        } else {
            heatmap_msg_.data[i] = heatmap_image.data[i];
        }
    }
}

}  // namespace floor_heat_mapper