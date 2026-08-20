#include "track.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>




// 权重数组：索引0对应最左传感器，索引7对应最右
const int16_t weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
bool lost_line = false;
bool finish = false;
/**
 * @brief  读取8路灰度并计算加权偏差
 * @retval int16_t 偏差值：负=偏左，正=偏右，0=居中
 */
 
void track_init(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
}
uint8_t sensor_read(void)
{   uint8_t raw = 0;
    raw |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) << 0;
    raw |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) << 1;
    raw |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) << 2;
    raw |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)  << 3;
    raw |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)  << 4;
    raw |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) << 5;
    raw |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) << 6;
    raw |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) << 7;
    
    //raw ^= 0xFF;//0xFF是11111111，与操作后，1变0，0变1,这里异或取反，使1变0，0变1

    return raw;
}



int16_t huidu_get_offset(void)//返回横向距离偏差，用pid控制，偏差为正，线在右，右转，偏差为负，线在左，左转
{
    uint8_t res = sensor_read();
    static uint8_t line_count = 0;
    static uint8_t finish_count = 0;
    static int16_t last_offset = 0;
    int16_t offset = 0;
    uint8_t cnt = 0;
    //终点检测，全黑时为终点
    if (res == 0xFF) {
        finish_count++;
        if (finish_count >= 10) {       // 连续10次全黑，约100ms
            finish = true;              // 设置终点标志（全局变量）
            finish_count = 10;          // 防止溢出
        }
        // 全黑时直接返回0（保持直行），跳过加权平均
        last_offset = 0;
        return 0;
    } else {
        finish_count = 0;               // 一旦不是全黑，立即清零终点计数器
    }

    // 计算加权平均偏差
    for (uint8_t i = 0; i < 8; i++) {
        if (res & (1 << i)) {
            offset += weight[i];
            cnt++;
        }
    }
    if (cnt > 0) {
        // 检测到线，更新有效偏差
        offset = offset / cnt;   // 加权平均，得到连续的位置偏差
        last_offset = offset;
        line_count = 0;
        lost_line = false;
        return offset;
    } else {
        // 全白，丢线处理
        line_count++;
        if (line_count >= 5) {
            lost_line = true;
            line_count = 5;  // 防止溢出
            // 返回最后有效偏差，保持搜索方向
            return last_offset;
        } else {
            // 短暂丢线，也返回最后有效偏差，避免误判
            return last_offset;
        }
    }
}
















