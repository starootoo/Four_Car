#include "encoder.h"
#include "tim.h"
#include <math.h>



void encoder_init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    
}



// Ensure function name matches header declaration
wheel_speed_t encoder_GetSpeeds(void) {//M法测速
    wheel_speed_t spd;
    spd.left  = (int)speed_lpm;//最大带轮子空载速度1099mm/s
    spd.right = (int)speed_rpm;
    return spd;
}



