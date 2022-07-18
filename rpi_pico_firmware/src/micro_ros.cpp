#include "micro_ros.hpp"

constexpr uint8_t LED_PIN = 25;
rcl_publisher_t image_pub;
rcl_publisher_t info_pub;
sensor_msgs__msg__Image image_msg;
std_msgs__msg__UInt16MultiArray info_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

uint16_t image_data_ros[IMAGE_HEIGHT * IMAGE_WIDTH];
float image_data_temperature[IMAGE_HEIGHT * IMAGE_WIDTH];
uint16_t info_buffer[BUFFER_LENGHT];

void error_loop() {
    while (true) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));

        RCSOFTCHECK(rcl_publish(&image_pub, &image_msg, NULL));

        // thermal_camera_read_data(image_data_temperature);
    }
}

void micro_ros_init() {
    serial_init();
    allocator_init();
    node_init();
    publishers_init();
    timer_init();
    fill_image_msg_constants();
}

void micro_ros_deinit() {
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&image_pub, &node);
    rclc_support_fini(&support);
    rcl_node_fini(&node);
    rclc_executor_fini(&executor);
}

void micro_ros_spin() {
    while (true) {
        delay(100);
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    }
}

void serial_init() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);
}

void allocator_init() {
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
}

void node_init() {
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
}

void publishers_init() {
    RCCHECK(rclc_publisher_init_default(
        &image_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
        PUBLISHER_NAME));
}

void timer_init() {
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void fill_image_msg_constants() {
    image_msg.header.frame_id.data = (char*)THERMAL_CAMERA_FRAME_NAME;
    image_msg.width = IMAGE_WIDTH;
    image_msg.height = IMAGE_HEIGHT;
    image_msg.encoding.data = "mono16";
    image_msg.is_bigendian = false;
    image_msg.step = 32;  // TODO: co to?
    image_msg.data.capacity = IMAGE_WIDTH * IMAGE_HEIGHT;
    // image_msg.data.data = (uint8_t*)image_data_ros;
    // image_msg.data.size = sizeof(image_data_ros);
}

void convert_temperatures_to_image_msg() {
    for (uint8_t i = 0; i < IMAGE_HEIGHT * IMAGE_WIDTH; ++i) {
        if (image_data_temperature[(int(i / 16) * 16 + 15) - (i - int(i / 16) * 16)] < 0) {
            image_data_ros[int(i / 16) * 32 + i] = 0;
        } else {
            image_data_ros[int(i / 16) * 32 + i] = (uint16_t)(image_data_temperature[(int(i / 16) * 16 + 15) - (i - int(i / 16) * 16)] * 100);  // magic
        }
    }
}