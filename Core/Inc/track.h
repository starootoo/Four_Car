#ifndef __TRACK_H__
#define __TRACK_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif











//灰度传感器，白色亮灯输出高电平，黑色灭灯输出低电平
//调节时，双手抓两侧，顺时针旋转，在白色区域调节到灯恰好亮
//
extern bool lost_line;
void track_init(void);
uint8_t sensor_read(void);
int16_t huidu_get_offset(void);








#endif

#ifdef __cplusplus
}
#endif