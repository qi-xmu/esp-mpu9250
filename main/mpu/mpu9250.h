#pragma once
#include "mpu9250_map.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include <time.h>

uint8_t mpu9250_init();

uint8_t MPU_Write_Byte(mpu_reg reg, uint8_t data);
uint8_t MPU_Write_Len(uint8_t saddr, mpu_reg reg, uint8_t *data, uint8_t len);

uint8_t MPU_Read_Byte(mpu_reg reg, uint8_t *res);
uint8_t MPU_Read_Len(uint8_t saddr, mpu_reg reg, uint8_t *buf, uint8_t len);

uint8_t MPU_Set_Gyro_FSR(uint8_t fsr);
uint8_t MPU_Set_Accel_FSR(uint8_t fsr);
uint8_t MPU_Set_Rate(uint16_t rate);

int16_t MPU_Get_Temp();

/* 其他 */

void _get_ms(unsigned long *time); 

void _delay_ms(unsigned long num_ms);