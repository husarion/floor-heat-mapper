#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <MLX90641_API.h>
#include <MLX9064X_I2C_Driver.h>

constexpr uint32_t I2C_FREQUENCY = 400000;
constexpr uint8_t THERMAL_CAMERA_I2C_ADDRESS = 0x33;
constexpr uint8_t TA_SHIFT = 8;
constexpr uint8_t IMAGE_WIDTH = 16;
constexpr uint8_t IMAGE_HEIGHT = 12;

void thermal_camera_init();
void thermal_camera_check_eeprom_errors();
void thermal_camera_read_data(float read_data[]);
void thermal_camera_filter_nan(float read_data[]);
void thermal_camera_calibration();