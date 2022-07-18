#pragma once

#include <Arduino.h>
#include <Wire.h>
constexpr uint8_t LED_PIN = 25;

#include "micro_ros.hpp"
#include "thermal_camera.hpp"

void led_init();