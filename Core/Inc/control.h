#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"
#include <stdint.h>


extern float left_speed_output, right_speed_output;   // 声明（告诉编译器这俩变量在别处定义了）
extern float aver_speed_output;   // 声明（告诉编译器这俩变量在别处定义了）
extern volatile uint32_t count_period;  // 声明（告诉编译器这俩变量在别处定义了）
extern float z_angle_output;
extern float track_output;
extern volatile float distance;
extern volatile uint8_t x_target;
extern volatile uint8_t y_target;



extern float dis_turn;
// extern float L_output, R_output,angle_z;
// extern float z_angle_output;
// extern float left_speed_output;
// extern float right_speed_output;

typedef enum {
    locate_turn = 0,
    locate_normal,
    locate_stop
} locate_dir_t;//定位枚举

typedef enum {
    track_xunxian = 0,
    track_distance,
    track_turn,
    track_stop
} track_dir_t;//方形循迹枚举

typedef enum {
    follow = 0,
    encoder_distance,
    encoder_turn,
    encoder_stop
} encoder_dir_t;//编码器枚举





void control_init(void);
void calc_speed(void);
void calc_distance(void);
void locate_line(locate_dir_t *id,uint8_t x,uint8_t y,uint8_t cm);
void track_rectangle1(track_dir_t *id);//mpu转弯循迹
void track_encoder(encoder_dir_t *id);//编码器加灰度实现转弯









#endif // CONTROL_H