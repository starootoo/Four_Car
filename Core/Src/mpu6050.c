#include "mpu6050.h"
#include "i2c.h"
#include <math.h>
#include <stdint.h>
// MPU6050 register addresses
#define MPU6050_ADDRESS ( 0x68 << 1) // I2C address of MPU6050

// 加速度计零偏（原始值）
static int16_t accel_offset_x = -504;
static int16_t accel_offset_y = 770;
static int16_t accel_offset_z = 3038;

// 陀螺仪零偏（原始值）
static int16_t gyro_offset_x = 0;
static int16_t gyro_offset_y = 0;
static int16_t gyro_offset_z = 0;





void mpu_send(uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, buffer, 2, 20);
}

void mpu_read(uint8_t reg, uint8_t *data, uint16_t len)
{
   
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &reg, 1, 20);
    
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDRESS, data, len, 20);
    
}


void MPU6050_Calibrate(void)
{
    const int samples = 300;
    //int32_t sum_gx = 0, sum_gy = 0 sum_gz = 0;
    int32_t sum_gz = 0;
    uint8_t raw[2];

    for (int i = 0; i < samples; i++)
    {
        // 读陀螺仪 0x43 ~ 0x48
        mpu_read(0x47, raw, 2);
        //sum_gx += (int16_t)(raw[0] << 8 | raw[1]);
        //sum_gy += (int16_t)(raw[2] << 8 | raw[3]);
        sum_gz += (int16_t)(raw[0] << 8 | raw[1]);

        
    }
    // 陀螺仪零偏：静止时期望 0
    //gyro_offset_x = (int16_t)(sum_gx / samples);
    //gyro_offset_y = (int16_t)(sum_gy / samples);
    gyro_offset_z = (int16_t)(sum_gz / samples);
}




void mpu_init()
{   
    
    // Wake up the MPU6050
    mpu_send(0x6b, 0x80);// Reset the device
    HAL_Delay(100);
    mpu_send(0x6b, 0x00);// Wake up the device
    mpu_send(0x1b, 0x18);// Set gyroscope range to ±2000°/s
    mpu_send(0x1c, 0x00);// Set accelerometer range to ±2g
    // 在 main() 的初始化部分，MPU6050 已校准、小车静止放置后：
    HAL_Delay(20);
    // 执行零点校准（小车必须静止！）
    MPU6050_Calibrate();
    // mpu_get_actangle();                         // 第一次调用，此时 angle_acc 已更新
    // angle_gyro = angle_acc;                     // 陀螺仪积分起点 = 加速度计当前角度
    // angle = angle_acc;                          // 融合角度也初始化为当前角度

    
}

void MPU6050_Read_RealData(I2C_HandleTypeDef *hi2c, MPU6050_RealData_t *real) {
    uint8_t raw[14];
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDRESS, 0x3b, 
                     I2C_MEMADD_SIZE_8BIT, raw, 14, 20);
    

    int16_t ax_raw = (int16_t)(raw[0] << 8 | raw[1]);
    int16_t ay_raw = (int16_t)(raw[2] << 8 | raw[3]);
    int16_t az_raw = (int16_t)(raw[4] << 8 | raw[5]);
    int16_t gx_raw = (int16_t)(raw[8] << 8 | raw[9]);
    int16_t gy_raw = (int16_t)(raw[10] << 8 | raw[11]);
    int16_t gz_raw = (int16_t)(raw[12] << 8 | raw[13]);


     // 括号里拼出原始整数，紧接着就除以灵敏度，结果直接存进 float 成员
    real->ax = (ax_raw - accel_offset_x)/ 16.707f;//cm/s²,
    real->ay = (ay_raw - accel_offset_y)/ 16.707f;
    real->az = (az_raw - accel_offset_z)/ 16.707f;
    // 跳过温度 raw[6..7]
    real->gx = (gx_raw - gyro_offset_x)/ 1.6384f;//°/s,0~900
    real->gy = -(gy_raw - gyro_offset_y)/ 1.6384f;
    real->gz = (gz_raw - gyro_offset_z)/ 1.6384f;
}

void mpu_get_actangle(){
    MPU6050_RealData_t real_data;
    MPU6050_Read_RealData(&hi2c1, &real_data);
    float a=0.96f;
    angle_acc=atan2f(real_data.ax, real_data.az)*1800/M_PI;
    angle_gyro=real_data.gy*0.01f+angle;
    angle=a*angle_gyro+(1-a)*angle_acc;
    //z_angle=real_data.gz;
}


void mpu_get_gz(){
    uint8_t raw1[2];
    mpu_read(0x47,raw1,2);
    int16_t gz_raw1=(int16_t)(raw1[0] << 8 | raw1[1]);
    z_angle=(gz_raw1 - gyro_offset_z)/ 1.6384f;//°/s,0~900
}

//4. 将小车放置在一个网格图的原点坐标上，随机输入一个坐标，小车能够通过陀螺仪导
//航至该坐标点上。小车不可走直角路线（如先向上直行，再向右直行），必须在行驶过
//程中依托陀螺仪不断修正方向，直到对准坐标点，最终导航到该坐标点。
float angle_range=0;
float angle_z=0;//全局变量，航向角度0~900
void navigate(){
    mpu_get_gz();
    if(z_angle<-angle_range || z_angle>angle_range){
        angle_z+=z_angle*0.01f;//逆时针为正方向，顺时针为负方向,周期10ms
    }

    
}






