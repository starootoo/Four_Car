#ifndef __HCSR04_H__
#define __HCSR04_H__

#include "main.h"
#include "delay.h"

// 引脚配置（根据实际硬件修改）
#define HCSR04_TRIG_PORT    GPIOA
#define HCSR04_TRIG_PIN     GPIO_PIN_15
#define HCSR04_ECHO_PORT    GPIOB
#define HCSR04_ECHO_PIN     GPIO_PIN_12

// 超时时间（微秒），约40厘米量程
#define HCSR04_TIMEOUT_US   2500

void hcsr04_Init(void);
uint16_t hcsr04_read(void);      // 返回距离，单位：毫米
float hcsr04_read_cm(void);      // 返回距离，单位：厘米

//目标距离200mm，实际距离超声波读取，误差为正，需要减速，误差为负，需要加速




#endif // __HCSR04_H__