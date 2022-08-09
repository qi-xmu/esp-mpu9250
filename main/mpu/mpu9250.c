#include "mpu9250.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "MPU9250";

uint8_t mpu9250_init() {
    uint8_t res = 0;

    MPU_Write_Byte(MPU9250_PWR_MGMT_1, 0x80);
    vTaskDelay(100 / portTICK_RATE_MS);
    MPU_Write_Byte(MPU9250_PWR_MGMT_1, 0x00);
    // 设置量程
    MPU_Set_Gyro_FSR(2); // 3
    MPU_Set_Accel_FSR(2);
    MPU_Set_Rate(50);

    MPU_Write_Byte(MPU9250_INT_ENABLE, 0X00);  //关闭所有中断
    MPU_Write_Byte(MPU9250_USER_CTRL, 0X00);   // I2C主模式关闭
    MPU_Write_Byte(MPU9250_FIFO_EN, 0X00);     //关闭FIFO
    MPU_Write_Byte(MPU9250_INT_PIN_CFG, 0X80); // INT引脚低电平有效
    MPU_Read_Byte(MPU9250_WHO_AM_I, &res);     // 设备识别码

    ESP_LOGW(TAG, "who am i is 0x%x", res);

    if (res == MPU9250_WHO_AM_I_RESULT) {
        MPU_Write_Byte(MPU9250_PWR_MGMT_1, 0X01); // 设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU9250_PWR_MGMT_2, 0X00); //加速度与陀螺仪都工作
        MPU_Set_Rate(50);                         //设置采样率为50Hz
    } else
        return 1;
    return 0;
}

/**
 * @brief Write a byte to MPU through I2C
 *        写入i2c数据
 *
 * @param reg parameter is a register of MPU
 * @param data parameter will be written to the register of MPU
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Write_Byte(mpu_reg reg, uint8_t data) {
    esp_err_t error;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    error = i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, 1);
    if (error != ESP_OK)
        return 1;

    error = i2c_master_write_byte(cmd, reg, 1);
    if (error != ESP_OK)
        return 1;

    error = i2c_master_write_byte(cmd, data, 1);
    if (error != ESP_OK)
        return 1;

    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);

    return 0;
}

/**
 * @brief Write a buffer to MPU through I2C
 *        写入i2c缓冲
 *
 * @param saddr parameter is slave_addr
 * @param reg parameter is a register of MPU
 * @param data parameter is a buffer which will be written to a register of MPU
 * @param len parameter is the length of data
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Write_Len(uint8_t saddr, mpu_reg reg, uint8_t len, uint8_t *data) {
    esp_err_t error;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    error = i2c_master_write_byte(cmd, (saddr << 1) | I2C_MASTER_WRITE, 1);
    if (error != ESP_OK)
        return 1;

    error = i2c_master_write_byte(cmd, reg, 1);
    if (error != ESP_OK)
        return 1;

    error = i2c_master_write(cmd, data, len, 1);
    if (error != ESP_OK)
        return 1;

    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);

    return 0;
}

/**
 * @brief Read a byte from MPU through I2C
 *        读取i2c数据
 *
 * @param reg parameter is a register of MPU
 * @param res the data read will be stored in this parameter
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Read_Byte(mpu_reg reg, uint8_t *res) {
    esp_err_t error;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    error = i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, 1);
    if (error != ESP_OK)
        return 1;

    error = i2c_master_write_byte(cmd, reg, 1);
    if (error != ESP_OK)
        return 1;

    i2c_master_start(cmd);
    error = i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, 1);
    if (error != ESP_OK)
        return 1;

    error = i2c_master_read_byte(cmd, res, I2C_MASTER_LAST_NACK);
    if (error != ESP_OK)
        return 1;

    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);
    return 0;
}

/**
 * @brief Read a buffer from MPU through I2C
 *        读取i2c缓冲
 *
 * @param saddr parameter is slave_addr
 * @param reg parameter is a register of MPU
 * @param buf parameter is a buf witch will store the data
 * @param len parameter is the length of buf
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Read_Len(uint8_t saddr, mpu_reg reg, uint8_t len, uint8_t *buf) {
    esp_err_t error;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    error = i2c_master_write_byte(cmd, (saddr << 1) | I2C_MASTER_WRITE, 1);
    if (error != ESP_OK)
        return 1;

    error = i2c_master_write_byte(cmd, reg, 1);
    if (error != ESP_OK)
        return 1;

    i2c_master_start(cmd);
    error = i2c_master_write_byte(cmd, (saddr << 1) | I2C_MASTER_READ, 1);
    if (error != ESP_OK)
        return 1;

    error = i2c_master_read(cmd, buf, len, I2C_MASTER_LAST_NACK);
    if (error != ESP_OK)
        return 1;

    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);
    return 0;
}

/**
 * @brief Set the Gyroscope full-scale range of ±250, ±500, ±1000, and
 * ±2000°/sec (dps) 设置陀螺仪的量程
 *
 * @param fsr the number of register, it could be 0, 1, 2, 3
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Set_Gyro_FSR(uint8_t fsr) {
    return MPU_Write_Byte(MPU9250_GYRO_CONFIG, fsr << 3);
}

/**
 * @brief Set the Accelerometer full-scale range of ±2g, ±4g, ±8g, and ±16g
 *        设置加速度量程
 *
 * @param fsr the number of register, it could be 0, 1, 2, 3
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Set_Accel_FSR(uint8_t fsr) {
    return MPU_Write_Byte(MPU9250_ACCEL_CONFIG, fsr << 3);
}

/**
 * @brief Set the band of low pass filter
 *        设置低通滤波的带宽
 *
 * @param lps parameter is the band of low pass filter
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Set_LPF(uint16_t lpf) {
    uint8_t data = 0;
    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return MPU_Write_Byte(MPU9250_CONFIG, data);
}

/**
 * @brief Set the Sample rate of Gyroscope, Accelerometer, DMP, etc.
 *        设置采样速率
 *
 * @param rate parameter is the sample rate of Gyroscope, Accelerometer, DMP,
 * etc.
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Set_Rate(uint16_t rate) {
    uint8_t data;
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;
    data = 1000 / rate - 1;
    data = MPU_Write_Byte(MPU9250_SMPLRT_DIV, data);
    return MPU_Set_LPF(rate /
                       2); /*!< set low pass filter the half of the rate */
}

/**
 * @brief Get the temperature of the MPU
 *        获取温度值
 *
 * @return
 *     - temp is the temperature of the MPU
 *     - 1 is Error
 */
int16_t MPU_Get_Temp() {
    uint8_t buf[2];
    int16_t raw;
    float temp;
    uint8_t res = MPU_Read_Len(MPU_ADDR, MPU9250_TEMP_OUT_H, 2, buf);
    if (res != 0)
        return 1;
    raw = ((uint16_t)(buf[0] << 8)) | buf[1];
    temp = 21 + raw / 333.87;
    return temp * 100;
}

/**
 * @brief Get the Gyroscope data of the MPU
 *        获取角加速度
 *
 * @param gx parameter is the x axis data of Gyroscope
 * @param gy parameter is the y axis data of Gyroscope
 * @param gz parameter is the z axis data of Gyroscope
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Get_Gyroscope(int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU9250_GYRO_XOUT_H, 6, buf);
    if (res == 0) {
        *gx = ((uint16_t)buf[0] << 8) | buf[1];
        *gy = ((uint16_t)buf[2] << 8) | buf[3];
        *gz = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
}

/**
 * @brief Get the Accelerometer data of the MPU
 *        获取加速度
 *
 * @param ax parameter is the x axis data of Accelerometer
 * @param ay parameter is the y axis data of Accelerometer
 * @param az parameter is the z axis data of Accelerometer
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Get_Accelerometer(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU9250_ACCEL_XOUT_H, 6, buf);
    if (res == 0) {
        *ax = ((uint16_t)buf[0] << 8) | buf[1];
        *ay = ((uint16_t)buf[2] << 8) | buf[3];
        *az = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
}

/**
 * @brief delay some millionseconds
 *
 * @param num_ms
 * @return * void
 */
void _delay_ms(unsigned long num_ms) {
    vTaskDelay(num_ms / portTICK_PERIOD_MS);
}

/**
 * @brief get system time
 *
 * @param time parameter is return time in milliseconds.
 * @return * void
 */
void _get_ms(unsigned long *time) { *time = esp_log_timestamp(); }