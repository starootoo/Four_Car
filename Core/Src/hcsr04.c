#include "hcsr04.h"

void hcsr04_Init(void)
{
    // 确保delay已初始化
    delay_init();

    // Trig引脚初始化为低电平
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
}
 
uint16_t hcsr04_read(void)
{
    uint32_t start_time;
    uint32_t timeout_counter;

    // 发送10微秒以上的Trig信号触发测距
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
    delay_us(20);
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);

    // 等待Echo上升沿（带超时保护）
    timeout_counter = 0;
    while (HAL_GPIO_ReadPin(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_RESET)
    {
        delay_us(1);
        timeout_counter++;
        if (timeout_counter > HCSR04_TIMEOUT_US)
        {
            return 0; // 超时返回0
        }
    }

    // 记录Echo高电平开始时间
    start_time = DWT->CYCCNT;

    // 等待Echo下降沿（测量高电平持续时间）
    timeout_counter = 0;
    while (HAL_GPIO_ReadPin(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_SET)
    {
        timeout_counter++;
        if (timeout_counter > HCSR04_TIMEOUT_US)
        {
            return 0; // 超时返回0
        }
    }

    // 计算高电平持续时间（微秒）
    uint32_t cycles = DWT->CYCCNT - start_time;
    uint32_t time_us = cycles / (HAL_RCC_GetHCLKFreq() / 1000000);

    // 计算距离
    // 声速：340m/s = 0.34mm/微秒
    // 距离(mm) = 时间(微秒) × 声速(mm/微秒) ÷ 2 = time_us × 0.34 ÷ 2 = time_us × 34 / 200
    uint16_t distance_mm = (uint16_t)(time_us * 34 / 200);

    return distance_mm;
}

float hcsr04_read_cm(void)
{
    return (float)hcsr04_read() / 10.0f;
}





