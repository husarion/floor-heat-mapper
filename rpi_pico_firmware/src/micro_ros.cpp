#include "micro_ros.hpp"

constexpr uint8_t LED_PIN = 25;
rcl_publisher_t image_pub;
static sensor_msgs__msg__Image image_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

static float image_data_temperature[IMAGE_HEIGHT * IMAGE_WIDTH] = {0.0};
static uint16_t image_data_ros[IMAGE_HEIGHT * IMAGE_WIDTH] = {0};
extern void setup();
extern int main();

static enum states state;

void error_loop() {
    while (true) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        if (rmw_uros_ping_agent(100, 1) == RCL_RET_OK) {
            setup();
            while (true) {
                main();
            }
        }
    }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        thermal_camera_read_data(image_data_temperature);
        fill_image_msg_with_thermal_camera();
        rcl_publish(&image_pub, &image_msg, NULL);
    }
}

bool micro_ros_init() {
    allocator_init();
    node_init();
    publishers_init();
    timer_init();
    fill_image_msg_constants();
    return true;
}

void micro_ros_deinit() {
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&image_pub, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

void micro_ros_spin() {
    delay(100);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

void allocator_init() {
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
}

void node_init() {
    rclc_node_init_default(&node, NODE_NAME, "", &support);
}

void publishers_init() {
    rclc_publisher_init_default(
        &image_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
        PUBLISHER_NAME);
}

void timer_init() {
    const unsigned int timer_timeout = 1000;
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback);

    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);
}

void fill_image_msg_constants() {
    image_msg.header.frame_id.data = (char*)(THERMAL_CAMERA_FRAME_NAME);
    image_msg.width = IMAGE_WIDTH;
    image_msg.height = IMAGE_HEIGHT;
    image_msg.encoding.data = "mono16";
    image_msg.is_bigendian = false;
    image_msg.step = 32;
    image_msg.data.data = (uint8_t*)(image_data_ros);
    image_msg.data.size = sizeof(image_data_ros);
}

void fill_image_msg_with_thermal_camera() {
    struct timespec currnet_timestamp = get_time_stamp();
    image_msg.header.stamp.nanosec = currnet_timestamp.tv_nsec;
    image_msg.header.stamp.sec = currnet_timestamp.tv_sec;
    mirror_thermal_image();
    for (uint8_t i = 0; i < IMAGE_HEIGHT * IMAGE_WIDTH; ++i) {
        if (image_data_temperature[i] < 0.0) {
            image_data_temperature[i] = 0.0;
        }
        image_data_ros[i] = image_data_temperature[i] * 10;
    }
}

timespec get_time_stamp() {
    struct timespec currnet_timestamp = {0};
    rmw_uros_sync_session(1000);
    if (rmw_uros_epoch_synchronized()) {
        currnet_timestamp.tv_sec = rmw_uros_epoch_millis() / 1000;
        currnet_timestamp.tv_nsec = rmw_uros_epoch_nanos();
    }
    return currnet_timestamp;
}

void mirror_thermal_image() {
    for (int i = 0; i < IMAGE_HEIGHT / 2; ++i) {
        for (int j = 0; j < IMAGE_WIDTH; ++j) {
            auto saved_value = image_data_temperature[j + i * IMAGE_WIDTH];
            image_data_temperature[j + i * IMAGE_WIDTH] = image_data_temperature[j + ((IMAGE_HEIGHT - 1 - i) * IMAGE_WIDTH)];
            image_data_temperature[j + ((IMAGE_HEIGHT - 1 - i) * IMAGE_WIDTH)] = saved_value;
        }
    }
}

void handle_micro_ros_with_reconnection(){
    switch (state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(2000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == micro_ros_init()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                micro_ros_deinit();
            };
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(2000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) {
                micro_ros_spin();
            }
            break;
        case AGENT_DISCONNECTED:
            micro_ros_deinit();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
}