#ifndef __ENCODER_H__
#define __ENCODER_H__

extern float speed_lpm;   // 声明（告诉编译器这俩变量在别处定义了）
extern float speed_rpm;



typedef struct {
    float left;
    float right;
} wheel_speed_t;



void encoder_init(void);
wheel_speed_t encoder_GetSpeeds(void);


#endif
