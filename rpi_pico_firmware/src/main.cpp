#include "main.hpp"
static enum states state;
void setup() {
    led_init();
    thermal_camera_init();
    thermal_camera_calibration();
    serial_init();
}

void loop() {
    handle_micro_ros_with_reconnection();

    if (state == AGENT_CONNECTED and millis() % 250 == 0) {
        digitalWrite(LED_PIN, 1);
    } else {
        digitalWrite(LED_PIN, 0);
    }
}

void led_init() {
    pinMode(LED_PIN, OUTPUT);
}

void serial_init() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);
}