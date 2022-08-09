#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
constexpr uint8_t LED_PIN = 25;

#include "micro_ros.hpp"
#include "thermal_camera.hpp"

void led_init();
void serial_init();