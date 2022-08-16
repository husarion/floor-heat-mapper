#include "thermal_camera.hpp"
static uint16_t MLX_eeprom[832];
constexpr uint8_t LED_PIN = 25;
paramsMLX90641 thermal_camera_params;
float calibration_array[IMAGE_WIDTH * IMAGE_HEIGHT] = {0};

void thermal_camera_init() {
    Wire.begin();
    Wire.setClock(I2C_FREQUENCY);
    MLX9064x_I2CInit();
    delay(1000);
    MLX9064x_I2CWrite(THERMAL_CAMERA_I2C_ADDRESS, 0x800D, 0x0901);

    thermal_camera_check_eeprom_errors();
    MLX90641_SetRefreshRate(THERMAL_CAMERA_I2C_ADDRESS, 0x05);
    Wire.setClock(2*I2C_FREQUENCY);
}

void thermal_camera_check_eeprom_errors(){
    int8_t ret_code = MLX90641_DumpEE(THERMAL_CAMERA_I2C_ADDRESS, MLX_eeprom);
    if (ret_code != 0) {
        while (1) {
            switch (ret_code) {
                case -1:
                    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                    delay(1000);
                    break;
                case -9:
                    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                    delay(500);
                    break;
                case -10:
                    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                    delay(50);
                    break;
                default:
                    digitalWrite(LED_PIN, LOW);
                    break;
            }
        }
    }

    digitalWrite(LED_PIN, HIGH);
    MLX90641_ExtractParameters(MLX_eeprom, &thermal_camera_params);
}

void thermal_camera_read_data(float read_data[]){
    uint16_t mlx90640Frame[300];
    int status = MLX90641_GetFrameData(THERMAL_CAMERA_I2C_ADDRESS, mlx90640Frame);
    float vdd = MLX90641_GetVdd(mlx90640Frame, &thermal_camera_params);
    float ambient = MLX90641_GetTa(mlx90640Frame, &thermal_camera_params);
    float emissivity = MLX90641_GetEmissivity(&thermal_camera_params);
    MLX90641_CalculateTo(mlx90640Frame, &thermal_camera_params, emissivity, ambient, read_data);
    thermal_camera_filter_nan(read_data);
    for (auto i = 0u; i < IMAGE_WIDTH * IMAGE_HEIGHT; ++i) {
        read_data[i] -= calibration_array[i];
    }
}

void thermal_camera_filter_nan(float read_data[]) {
    for (auto i = 0u; i < IMAGE_WIDTH * IMAGE_HEIGHT; ++i)
        if (read_data[i] != read_data[i]) {
            auto sum = read_data[i - 1] + read_data[i + 1];
            sum += read_data[i - IMAGE_WIDTH] + read_data[i - IMAGE_WIDTH - 1] + read_data[i - IMAGE_WIDTH + 1];
            sum += read_data[i + IMAGE_WIDTH] + read_data[i + IMAGE_WIDTH - 1] + read_data[i + IMAGE_WIDTH + 1];
            read_data[i] = sum / 8;
        }
}

void thermal_camera_calibration(){
    thermal_camera_read_data(calibration_array);
    float average = 0.0;
    for (auto i = 0u; i < IMAGE_WIDTH * IMAGE_HEIGHT; ++i) {
        average += calibration_array[i];
    }

    average /= IMAGE_WIDTH * IMAGE_HEIGHT;

    for (auto i = 0u; i < IMAGE_WIDTH * IMAGE_HEIGHT; ++i) {
        calibration_array[i] -= average;
    }
}