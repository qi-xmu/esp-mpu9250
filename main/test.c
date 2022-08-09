#include <time.h>

void test_temp() {
    // 测试温度
    TAG = "TEST_TEMP";
    for (int i = 0; i < 5; i++) {
        vTaskDelay(1000 / portTICK_RATE_MS);
        uint16_t res = MPU_Get_Temp();
        ESP_LOGD(TAG, "Temperature is [%f] ", res / 100.00);
    }
}

void test_time() {
    // 测试时间
    TAG = "TEST_TIME";
    uint32_t res = esp_log_timestamp();
    ESP_LOGD(TAG, "Timestamp is [%d] ", res);
}