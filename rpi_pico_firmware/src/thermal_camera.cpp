#include "thermal_camera.hpp"
static uint16_t MLX_eeprom[832];
constexpr uint8_t LED_PIN = 25;
paramsMLX90641 thermal_camera_params;


void thermal_camera_init(){
    Wire.begin();
    Wire.setClock(I2C_FREQUENCY);
    MLX9064x_I2CInit();
    delay(1000);
    MLX9064x_I2CWrite(THERMAL_CAMERA_I2C_ADDRESS, 0x800D, 0x0901);

    thermal_camera_check_eeprom_errors();
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
}

void thermal_camera_read_data(float read_data[]){
    uint16_t mlx90640Frame[834];
    int status = MLX90641_GetFrameData(THERMAL_CAMERA_I2C_ADDRESS, mlx90640Frame);
    float vdd = MLX90641_GetVdd(mlx90640Frame, &thermal_camera_params);
    float Ta = MLX90641_GetTa(mlx90640Frame, &thermal_camera_params);

    float tr = Ta - TA_SHIFT;
    float emissivity = 0.95;

    MLX90641_CalculateTo(mlx90640Frame, &thermal_camera_params, emissivity, tr, read_data);
}