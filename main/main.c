#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#include "dmp/inv_mpu.h"
#include "dmp/inv_mpu_dmp_motion_driver.h"

#include "i2c_init.h"
#include "mpu/mpu9250.h"

static const char *TAG = "MAIN";

#include "test.c"

void app_main(void) {
    float pitch, roll, yaw;

    // unsigned long step_count = 0;
    dmp_set_pedometer_step_count(0);

    ESP_ERROR_CHECK(i2c_master_init());
    uint8_t i = mpu_dmp_init();
    ESP_LOGW(TAG, "[%d] Hello world!", i);

    // test_temp(); // 测试读取温度
    // test_time(); // 测试时间函数
    // test_mpu(); // 测试mpu读取

    uint8_t re = mpu_dmp_get_data(&pitch, &roll, &yaw);
    printf("--- %d\n", re);

    printf("pitch   = %f\n", pitch);
    printf("roll    = %f\n", roll);
    printf("yaw     = %f\n", yaw);

    // while (1) {
    //     ESP_LOGW("STEP","%ld", step_count);
    //     dmp_get_pedometer_step_count(&step_count);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    ESP_LOGW(TAG, "Restart!\n");
    // esp_restart();
}
