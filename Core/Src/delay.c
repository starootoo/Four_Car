#include "delay.h"

static uint32_t fac_us = 0; // 每微秒所需的时钟周期数

void delay_init(void)
{
    // 获取HCLK时钟频率
    fac_us = HAL_RCC_GetHCLKFreq() / 1000000; // 每微秒的时钟周期数

    // 启用DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // 复位CYCCNT计数器
    DWT->CYCCNT = 0;

    // 启用CYCCNT
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t nus)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = nus * fac_us;

    while ((DWT->CYCCNT - start) < cycles)
    {
        // 等待
    }
}