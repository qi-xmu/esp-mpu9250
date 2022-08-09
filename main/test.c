#include "dmp/inv_mpu.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_init.h"
#include "mpu/mpu9250.h"
#include <time.h>

// static const char *TAG = "TEST";

void test_temp() {
    // 测试温度
    for (int i = 0; i < 5; i++) {
        vTaskDelay(1000 / portTICK_RATE_MS);
        uint16_t res = MPU_Get_Temp();
        ESP_LOGD("TEST: temp", "Temperature is [%f] ", res / 100.00);
    }
}

void test_time() {
    // 测试时间
    uint32_t res = esp_log_timestamp();
    ESP_LOGD("TEST:time", "Timestamp is [%d] ", res);
}

void test_mpu() {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    for (int i = 0; i < 3; i++) {
        MPU_Get_Gyroscope(&gx, &gy, &gz);
        MPU_Get_Accelerometer(&ax, &ay, &az);

        printf("ax, ay, az = %d %d %d\n", ax, ay, az);
        printf("Gx, Gy, Gz = %d %d %d\n", gx, gy, gz);

        // uint8_t re = mpu_dmp_get_data(&pitch, &roll, &yaw);
        // printf("--- %d\n", re);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}