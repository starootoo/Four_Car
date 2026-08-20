/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <stdint.h>
#include <stdio.h>
#include "motor.h"
#include "hcsr04.h"
#include <string.h>
#include <sys/_intsup.h>
#include "encoder.h"
#include "pid.h"
#include "mpu6050.h"
#include "control.h"
#include "track.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char Receivedata[50];
int16_t x_error=0;
int16_t y_error=0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart == &huart2){

    sscanf(Receivedata, "$%hd,%hd*", &x_error, &y_error);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)Receivedata, Size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)Receivedata, sizeof(Receivedata));
  }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
pid_t pid_left, pid_right,pid_angle,pid_track; // 声明全局变量
char mpu_data[80];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(20); 
  OLED_Init();
  motor_init();
  encoder_init();
  mpu_init();
  track_init();
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)Receivedata, sizeof(Receivedata));
  pid_init(&pid_left,0.6f, 0.65f, 0.0f, 0.0f, -1000.0f, 1000.0f);//800时，1.9%误差，误差与速度成正比
  pid_init(&pid_right, 0.6f, 0.65f, 0.0f,100.0f, -1000.0f, 1000.0f);
  //20ms,单独调，Kp=1.4,Kd=0.0f  比较完美。10ms,kp=0.6~0.8,ki=0.6~0.72,kd=0.0f
  pid_init(&pid_angle, 0.9f, 0.7f, 1.5f,0.0f, -800.0f, 800.0f);//转向环pid,目标值是期望速度差，十倍关系,+为期望右转
  //50ms,单独调，Kp=3.8,Kd=2.0f  比较完美。，10ms,单独转向环kp1.0,ki0.0f,kd0.2f
  //50ms,并联速度环kp8.0,ki1.0f,kd1.4f，10ms并联速度环0.9f, 0.7f, 1.5f
  
  pid_init(&pid_track,50.0f, 0.0f, 5.0f, 0.0f, -800.0f, 800.0f);//循迹环pid,目标值是车头前横向距离差

  
  
  char message[50];
  char message1[50];
  char message2[50];
  char message3[50];
  control_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   

    // if(count_period-last_30ms>=20){//每20ms计算一次速度环
    //   last_30ms= count_period;  // Store the current count_period value
    //   last_encoder_countl=now_encoder_countl;
    //   now_encoder_countl= __HAL_TIM_GET_COUNTER(&htim2);
    //   last_encoder_countr=now_encoder_countr;
    //   now_encoder_countr= __HAL_TIM_GET_COUNTER(&htim4);
    //     /* 计算实际速度，单位为 mm/s. 使用 M_PI 常量 */
    //   speed_lpm = (float)((int32_t)now_encoder_countl - (int32_t)last_encoder_countl) * 65.0f * (float)M_PI / 728.0f / 0.03f;
    //   speed_rpm = (float)((int32_t)now_encoder_countr - (int32_t)last_encoder_countr) * 65.0f * (float)M_PI / 728.0f / 0.03f;
    //   left_speed_output=pid_calc(&pid_left, speed_lpm);
    //   right_speed_output=pid_calc(&pid_right, speed_rpm);
    //   L_output=left_speed_output+z_angle_output;
    //   R_output=right_speed_output-z_angle_output;
    //   motor_setlefrightspeed((int16_t)left_speed_output, (int16_t)right_speed_output);

    // }
    // if(count_period-last_50ms>=50){//每50ms计算一次转向环和循迹环
    //   last_50ms= count_period;  // Store the current count_period value
    //   speed_difference = speed_lpm - speed_rpm;//差速转向环，不依赖起始位置，粗略转向
    //   z_angle_output = pid_calc(&pid_angle, speed_difference);
      

    //   // navigate();
    //   // z_angle_output = pid_calc(&pid_angle, angle_z);//差角转向环,精确角度转向

    //  }
      
      
      sprintf(message, "speedl: %d", (int)speed_lpm);
      sprintf(message1, "speedr: %d", (int)speed_rpm);
      sprintf(message2, "dis: %d", (int)distance);
      sprintf(message3, "dis_turn: %d", (int)dis_turn);
      OLED_NewFrame();
      OLED_PrintASCIIString(0, 0, message, &afont8x6, OLED_COLOR_NORMAL);
      OLED_PrintASCIIString(0, 16, message1, &afont8x6, OLED_COLOR_NORMAL);
      OLED_PrintASCIIString(0, 32, message2, &afont8x6, OLED_COLOR_NORMAL);
      OLED_PrintASCIIString(0, 48, message3, &afont8x6, OLED_COLOR_NORMAL);
      OLED_ShowFrame();
      // if(lost_line){
      //   motor_stop(MOTOR_LEFT);
      //   motor_stop(MOTOR_RIGHT);
      //   HAL_TIM_Base_Stop_IT(&htim1);

      // }

     
     


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
