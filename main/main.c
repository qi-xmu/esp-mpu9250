#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#include "i2c_init.h"
#include "mpu/mpu9250.h"

static const char *TAG = "MAIN";

#include "test.c"

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    uint8_t i = mpu9250_init();
    ESP_LOGW(TAG, "[%d] Hello world!", i);
    
    // test_temp(); // 测试读取温度
    // test_time(); // 测试时间函数

    ESP_LOGW(TAG, "Restart!\n");
    // esp_restart();
}
