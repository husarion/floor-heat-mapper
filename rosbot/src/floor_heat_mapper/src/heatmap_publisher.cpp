#include "floor_heat_mapper/heatmap_publisher.hpp"

namespace floor_heat_mapper {
FloorHeatMapper::FloorHeatMapper()
    : rclcpp::Node(NODE_NAME) {
    handle_parameters();
    create_pubs_subs_and_timers();
    calculate_heatmap_resolution();
    create_thermal_camera_to_base_link_tf();
    create_heatpoints();
    RCLCPP_INFO(get_logger(), "Configured mapping, wainting for /%s and /%s...", THERMAL_CAMERA_TOPIC_NAME, MAP_TOPIC_NAME);
}

void FloorHeatMapper::create_pubs_subs_and_timers() {
    using namespace std::chrono_literals;
    using std::placeholders::_1;

    static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    heatmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(HEATMAP_TOPIC_NAME, 10);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(MAP_TOPIC_NAME, 10, std::bind(&FloorHeatMapper::map_callback, this, _1));
    thermal_camera_image_sub_ = create_subscription<sensor_msgs::msg::Image>(THERMAL_CAMERA_TOPIC_NAME, 10, std::bind(&FloorHeatMapper::thermal_camera_callback, this, _1));
    heatpoints_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(HEATPOINTS_TOPIC_NAME, 10);
    goal_poses_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(GOAL_POSES_TOPIC_NAME, 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&FloorHeatMapper::timer_callback, this));

    if (sync_mode_) {
        take_thermal_image_srv_ = create_service<std_srvs::srv::Trigger>(TAKE_THERMAL_IMAGE_SERVICE_NAME, FloorHeatMapper::take_thermal_image_callback);
    }
}

void FloorHeatMapper::handle_parameters() {
    declare_parameter<bool>("async", true);
    declare_parameter<bool>("sync", false);

    auto async_param = get_parameter("async");
    auto sync_param = get_parameter("sync");

    async_mode_ = async_param.get_value<bool>();
    sync_mode_ = sync_param.get_value<bool>();

    RCLCPP_INFO(get_logger(), "Set parameters: async: %s\tsync: %s...", async_mode_ ? "true" : "false", sync_mode_ ? "true" : "false");
    if ((not async_mode_) and (not sync_mode_)) {
        throw rclcpp::exceptions::InvalidParametersException("[FloorHeatMapper] There is no sync even async mode parameter.");
    }
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

    if (async_mode_ or (sync_mode_ and trigger_thermal_photo_)) {
        merge_single_thermal_image_and_heatmap();
        trigger_thermal_photo_ = false;
    }

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
    if (heatmap_synced_) {
        return;
    }

    RCLCPP_INFO(get_logger(), "Sync map info...");
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
    RCLCPP_INFO(get_logger(), "Heatmap constructed!");
    heatmap_synced_ = true;
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

cv::Mat FloorHeatMapper::normalize_and_check_min_max_temperatures(cv::Mat image) {
    cv::Mat normalized = image.clone();

    for (auto i = 0u; i < normalized.rows; ++i) {
        for (auto j = 0u; j < normalized.cols; ++j) {
            auto pixel = static_cast<double>(normalized.at<uint16_t>(i, j)) * THERMAL_IMAGE_TEMPERATURE_SCALE;

            hottest_temperature_ = hottest_temperature_ < pixel ? pixel : hottest_temperature_;
            coldest_temperature_ = coldest_temperature_ > pixel ? pixel : coldest_temperature_;

            auto x = pixel;
            pixel = pixel > MAX_FLOOR_NORMALIZED_TEMPERATURE ? MAX_FLOOR_NORMALIZED_TEMPERATURE : pixel;
            pixel = pixel < MIN_FLOOR_NORMALIZED_TEMPERATURE ? MIN_FLOOR_NORMALIZED_TEMPERATURE : pixel;
            pixel -= MIN_FLOOR_NORMALIZED_TEMPERATURE - 1.0;
            pixel /= MAX_FLOOR_NORMALIZED_TEMPERATURE - MIN_FLOOR_NORMALIZED_TEMPERATURE + 1.0;
            pixel *= COLOR_OCCUPACY_MAP_SCALE;

            normalized.at<uint16_t>(i, j) = static_cast<uint16_t>(pixel);
            if (hottest_temperature_ == x) {
                RCLCPP_INFO(get_logger(), "Temperatures:\tmax: %f,\t min: %f", hottest_temperature_, coldest_temperature_);
            }
        }
    }
    return normalized;
}

void FloorHeatMapper::thermal_camera_callback(const sensor_msgs::msg::Image image_msg) {
    cv_bridge_with_single_thermal_image_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO16);
    thermal_image_synced_ = true;
}

bool FloorHeatMapper::merge_single_thermal_image_and_heatmap() {
    if (not heatmap_synced_) {
        RCLCPP_INFO(get_logger(), "Waiting for synchronization with /%s topic...", MAP_TOPIC_NAME);
        return false;
    }
    if (not thermal_image_synced_) {
        RCLCPP_INFO(get_logger(), "Waiting for thermal image from /%s topic...", THERMAL_CAMERA_TOPIC_NAME);
        return false;
    }

    auto normalized_image = normalize_and_check_min_max_temperatures(cv_bridge_with_single_thermal_image_->image);
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

    auto heatmap_image = create_image_from_heatmap();
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
    mark_min_max_temperatures(heatmap_image);
    return true;
}

void FloorHeatMapper::create_heatpoints() {
    heatpoints_marker_.header.stamp = now();
    heatpoints_marker_.header.frame_id = MAP_FRAME_NAME;
    heatpoints_marker_.type = visualization_msgs::msg::Marker::POINTS;
    heatpoints_marker_.action = visualization_msgs::msg::Marker::ADD;
    heatpoints_marker_.scale.x = 0.1;
    heatpoints_marker_.scale.y = 0.1;
    heatpoints_marker_.scale.z = 0.1;
    heatpoints_marker_.pose.position.z = 0.2;
    heatpoints_marker_.ns = "points";
    heatpoints_marker_.color.a = 1.0;

    heatpoints_marker_.frame_locked = true;
    heatpoints_marker_.points.resize(2);
    heatpoints_marker_.colors.resize(2);
    heatpoints_marker_.colors[0].a = 1.0;
    heatpoints_marker_.colors[0].r = 0.8;
    heatpoints_marker_.colors[1].a = 1.0;
    heatpoints_marker_.colors[1].b = 1.0;

    hottest_point_ = &heatpoints_marker_.points[0];
    coldest_point_ = &heatpoints_marker_.points[1];
}

void FloorHeatMapper::mark_min_max_temperatures(cv::Mat image) {
    uint8_t max_value = std::numeric_limits<uint8_t>::min();
    uint8_t min_value = std::numeric_limits<uint8_t>::max();
    for (auto i = 0u; i < image.rows; ++i) {
        for (auto j = 0u; j < image.cols; ++j) {
            auto pixel = image.at<uint8_t>(i, j);
            if (pixel < min_value and pixel != 0) {
                min_value = pixel;
                coldest_point_->x = static_cast<double>(j) * heatmap_msg_.info.resolution + heatmap_msg_.info.origin.position.x;
                coldest_point_->y = static_cast<double>(i) * heatmap_msg_.info.resolution + heatmap_msg_.info.origin.position.y;
            } else if (pixel > max_value and pixel != 255 and pixel < 100) {
                max_value = pixel;
                hottest_point_->x = static_cast<double>(j) * heatmap_msg_.info.resolution + heatmap_msg_.info.origin.position.x;
                hottest_point_->y = static_cast<double>(i) * heatmap_msg_.info.resolution + heatmap_msg_.info.origin.position.y;
            }
        }
    }
    heatpoints_marker_pub_->publish(heatpoints_marker_);
}

void FloorHeatMapper::take_thermal_image_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    trigger_thermal_photo_ = true;
    response->success = heatmap_synced_ and thermal_image_synced_;
    response->message = response->success ? "Successfully took photo." : "Heatmap isn't synced with /map or /thermal_image.";
}
}  // namespace floor_heat_mapper