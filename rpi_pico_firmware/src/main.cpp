#include "main.hpp"


void setup() {
    led_init();
    thermal_camera_init();
    thermal_camera_calibration();
    micro_ros_init();
}

void loop() {
    micro_ros_spin();
}

void led_init() {
    pinMode(LED_PIN, OUTPUT);
}