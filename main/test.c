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

int test_mpu() {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    if (mpu9250_init())
        return -1;
    for (int i = 0; i < 3000000; i++) {
        MPU_Get_Gyroscope(&gx, &gy, &gz);
        MPU_Get_Accelerometer(&ax, &ay, &az);

        printf("ax, ay, az = %7.4lfg %7.4lfg %7.4lfg\n", ax / (32768.0f / 8),
               ay / (32768.0f / 8), az / (32768.0f / 8));
        printf("Gx, Gy, Gz = %7.4lfd %7.4lfd %7.4lfd\n", gx / (32768.0f / 1000),
               gy / (32768.0f / 1000), gz / (32768.0f / 1000));

        // uint8_t re = mpu_dmp_get_data(&pitch, &roll, &yaw);
        // printf("--- %d\n", re);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return 0;
}