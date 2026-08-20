#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"

extern float angle_acc;   // 声明（告诉编译器这俩变量在别处定义了）
extern float angle_gyro;   // 声明（告诉编译器这俩变量在别处定义了）
extern float angle;   // 声明（告诉编译器这俩变量在别处定义了）
extern int16_t z_angle;   // 声明（告诉编译器这俩变量在别处定义了）
extern float angle_z;

typedef struct {
    int16_t ax; // m/s²
    int16_t ay; // m/s²
    int16_t az; // m/s²
    int16_t gx; // °/s
    int16_t gy; // °/s
    int16_t gz; // °/s
} MPU6050_RealData_t;



void mpu_init(void);
void mpu_read(uint8_t reg, uint8_t *data, uint16_t len);
void MPU6050_Read_RealData(I2C_HandleTypeDef *hi2c, MPU6050_RealData_t *real);
void mpu_get_actangle(void);
void mpu_get_gz(void);
void navigate(void);
#endif // MPU6050_H