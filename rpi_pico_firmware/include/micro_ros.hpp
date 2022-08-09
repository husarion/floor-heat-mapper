#pragma once
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/image.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include <std_msgs/msg/u_int8.h>

#include "thermal_camera.hpp"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

#define EXECUTE_EVERY_N_MS(MS, X)          \
    do {                                   \
        static volatile int64_t init = -1; \
        if (init == -1) {                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS) {    \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

constexpr char NODE_NAME[] = "Thermal_Camera";
constexpr char PUBLISHER_NAME[] = "thermal_camera";
constexpr size_t ROS_DOMAIN_ID = 126;

constexpr char THERMAL_CAMERA_FRAME_NAME[] = "thermal_camera_frame";
constexpr uint8_t BUFFER_LENGHT = 10;

void error_loop();
void timer_callback(rcl_timer_t* timer, int64_t last_call_time);

bool micro_ros_init();
void micro_ros_deinit();
void micro_ros_spin();

void serial_init();
void led_init();
void allocator_init();
void node_init();
void publishers_init();
void timer_init();

void fill_image_msg_with_thermal_camera();
void fill_image_msg_constants();
void mirror_thermal_image();
void handle_micro_ros_with_reconnection();

float&
take_pixel_from(int x, int y);
timespec get_time_stamp();
