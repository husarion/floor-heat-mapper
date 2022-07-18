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

#define RCCHECK(fn)                    \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            error_loop();              \
        }                              \
    }
#define RCSOFTCHECK(fn)                \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
        }                              \
    }

constexpr char NODE_NAME[] = "Thermal_Camera";
constexpr char PUBLISHER_NAME[] = "thermal_camera";


constexpr char THERMAL_CAMERA_FRAME_NAME[] = "thermal_camera_frame";
constexpr uint8_t BUFFER_LENGHT = 10;
constexpr uint8_t IMAGE_WIDTH = 16;
constexpr uint8_t IMAGE_HEIGHT = 12;

void error_loop();
void timer_callback(rcl_timer_t* timer, int64_t last_call_time);

void micro_ros_init();
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
void filter_nan_values();
timespec get_time_stamp();
