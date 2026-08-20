#include "control.h"
#include "encoder.h"
#include "pid.h"
#include "motor.h"
#include "mpu6050.h"
#include <math.h>
#include <stdint.h>
#include "track.h"

#define PI 3.1415926f

/* TIM handles declared elsewhere */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern pid_t pid_left, pid_right,pid_angle,pid_track; // 声明全局变量

volatile uint32_t count_period= 0;

uint32_t  last_20ms = 0,last_30ms = 0,last_50ms = 0;
uint32_t  last_10ms = 0,last_40ms = 0,last_60ms = 0;
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
uint8_t run_flag = 1;  // 声明并初始化运行标志
//编码器测速度参数

uint16_t last_encoder_countl=0;
uint16_t now_encoder_countl=0;
uint16_t last_encoder_countr=0;
uint16_t now_encoder_countr=0;
float  speed_lpm = 0.0f;   // 声明并初始化全局变量
float  speed_rpm = 0.0f;   // 声明并初始化全局
float base_speed =44.0f;   // 声明并初始化全局变量

float left_speed_output = 0.0f;   // 声明并初始化全局变量
float right_speed_output = 0.0f;   // 声明并初始化全局
float speed_difference = 0.0f;   // 声明并初始化全局变量
float z_angle_output = 0.0f;   // 声明并初始化全局变量
float L_output = 0.0f;   // 声明并初始化全局变量
float R_output = 0.0f;   // 声明并初始化全局
float track_output = 0.0f;
int16_t track_offset = 0;
//转向环速度参数
int16_t z_angle = 0;

uint8_t mpu_raw_data[14];
MPU6050_RealData_t real_data;
encoder_dir_t encoder_mode = 0;

void control_init(void)
{
  x_target=0;
  y_target=0;
  speed_lpm=0.0f;
  speed_rpm=0.0f;
  distance = 0;
  // 同步编码器计数值
  now_encoder_countl = __HAL_TIM_GET_COUNTER(&htim2);
  now_encoder_countr = __HAL_TIM_GET_COUNTER(&htim4);
  last_encoder_countl = now_encoder_countl;
  last_encoder_countr = now_encoder_countr;
  HAL_TIM_Base_Start_IT(&htim1);
}
uint32_t m=0;
volatile float distance = 0;
locate_dir_t turn_mode = 0;
track_dir_t track_mode = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if(htim == &htim1){ 
    count_period++;
    if(count_period-last_10ms>=10){//每10ms计算一次速度环和转向环
      last_10ms= count_period;  // Store the current count_period value
      
      if(count_period-m>=3000){
        m=count_period;
        pid_angle.target_value += 900;
      }
      //转向环
      navigate();
      //pid_angle.target_value = track_offset;
      z_angle_output = pid_calc(&pid_angle, angle_z);//差角转向环,精确角度转向

      //速度环
      calc_speed();
      //calc_distance();
      left_speed_output=pid_calc(&pid_left, speed_lpm);
      right_speed_output=pid_calc(&pid_right, speed_rpm);
      

      //循迹环
      track_offset = huidu_get_offset();//循迹
      track_output=pid_calc(&pid_track, track_offset);
      //track_rectangle1(&track_mode);//方形循迹
      //track_encoder(&encoder_mode);//编码器加灰度实现转弯
      // L_output=left_speed_output-z_angle_output;
      // R_output=right_speed_output+z_angle_output;


      //电机控制
      //motor_setlefrightspeed((int16_t)L_output, (int16_t)R_output);
      
    }
    if(count_period-last_30ms>=30){//每30ms计算一次循迹环
      last_30ms= count_period;

      // track_offset = huidu_get_offset();//循迹
      // z_angle_output=pid_calc(&pid_track, track_offset);
    }
    if(count_period-last_50ms>=50){
      last_50ms= count_period;  // Store the current count_period value
     }
  }
}



void calc_speed(void){
  last_encoder_countl=now_encoder_countl;
  last_encoder_countr=now_encoder_countr;
  now_encoder_countl= __HAL_TIM_GET_COUNTER(&htim2);
  now_encoder_countr= __HAL_TIM_GET_COUNTER(&htim4);
  
  /* 计算实际速度，单位为 mm/s. 使用 M_PI 常量 */
  speed_lpm = (float)((int32_t)now_encoder_countl - (int32_t)last_encoder_countl) * 65.0f * (float)M_PI / 728.0f / 0.01f;
  speed_rpm = (float)((int32_t)now_encoder_countr - (int32_t)last_encoder_countr) * 65.0f * (float)M_PI / 728.0f / 0.01f;
}
void calc_distance(void){
  static float average_speed;
  average_speed = (speed_lpm+speed_rpm)/2.0f;
  distance += average_speed*0.001f;//每10ms计算一次距离，单位为cm
  if(distance<0){
    distance=0;
  }
}


void locate_line(locate_dir_t *id,uint8_t x,uint8_t y,uint8_t cm){//10ms,单独转向环kp1.0,ki0.0f,kd0.2f
  float s = sqrt( (x*x) + (y*y) ) * cm;
  float angle=atan2f(y, x)*1800/M_PI;

  pid_angle.target_value = angle;
  switch(*id){
    case locate_turn:
      pid_left.target_value = 0;
      pid_right.target_value = 0;
      L_output = left_speed_output-z_angle_output;
      R_output = right_speed_output+z_angle_output;
      if(angle_z>angle-20&&angle_z<angle+20){
        *id=locate_normal;
      }
      break;
    case locate_normal:
      //pid_angle.target_value = angle;
      pid_left.target_value = base_speed;
      pid_right.target_value = base_speed;
      calc_distance();
      L_output = left_speed_output-z_angle_output;
      R_output = right_speed_output+z_angle_output;
      if(distance>=s){
        *id=locate_stop;
      }
      break;
    case locate_stop:
      L_output = 0;
      R_output = 0;
      break;
  }
}


int car_body = 6;//车身长度,cm,两圈10，三圈
uint8_t count_90=0;
void track_rectangle1(track_dir_t *id){//第二版
  uint8_t res = sensor_read();
  switch(*id){
    case track_xunxian:
      L_output=base_speed-track_output;
      R_output=base_speed+track_output;
      if((res & 0x0F) == 0x0F){
        count_90++;
        if(count_90>=3){
          count_90=0;
          *id=track_distance;
          distance=0;
          //pid_angle.target_value = angle_z+900.0f;
          pid_left.target_value = base_speed*5;
          pid_right.target_value = base_speed*5;
        }
      }
      else{
        count_90=0;
      }
      break;
    case track_distance:
      calc_distance();
      L_output = left_speed_output;
      R_output = right_speed_output;
      if(distance>=car_body){
        motor_brake(MOTOR_LEFT);
        motor_brake(MOTOR_RIGHT);
        *id=track_turn;
        pid_left.target_value = 0;
        pid_right.target_value = 0;//kd=1.5f,ki=0.25f,kp=0.3f
        pid_angle.Kd=1.5f;
        pid_angle.Ki=0.20f;
        pid_angle.Kp=0.5f;
        pid_angle.target_value = angle_z+850.0f;
      }
      break;
    case track_turn://10ms,单独转向环kp1.0,ki0.0f,kd0.2f
      L_output = -z_angle_output;
      R_output = z_angle_output;
      if(angle_z > pid_angle.target_value-100.0f && angle_z < pid_angle.target_value+100.0f ){
            *id=track_xunxian;
            pid_track.integral = 0.0f;      // 清积分
            pid_track.prev_error = 0.0f;    // 清上一次误差（避免D项冲击）
            track_output = 0.0f;        // 输出也清零
          
      }
      break;
    case track_stop:
    
      break;
  }
}
float dis_turn=0;
uint8_t turn_count=0;
//编码器加灰度实现转弯
void track_encoder(encoder_dir_t *id){
  uint8_t res = sensor_read();
  switch(*id){
    case follow:
      L_output=base_speed-track_output;
      R_output=base_speed+track_output;
      if((res & 0x0F) == 0x0F){
        count_90++;
        if(count_90>=3){
          count_90=0;
          *id=encoder_distance;
          distance=0;
          //pid_angle.target_value = angle_z+900.0f;
          pid_left.target_value = base_speed*10;
          pid_right.target_value = base_speed*10;
        }
      }
      else{
        count_90=0;
      }
      break;
    case encoder_distance:
      
      L_output = left_speed_output;
      R_output = right_speed_output;
      calc_distance();
      if(distance>=car_body){
        motor_brake(MOTOR_LEFT);
        motor_brake(MOTOR_RIGHT);
        *id=encoder_turn;
        pid_left.target_value = 0;
        pid_right.target_value = base_speed*5;
        dis_turn=0;
      }
      break;
    case encoder_turn:
      L_output = left_speed_output;
      R_output = right_speed_output;
      
      dis_turn += speed_rpm*0.001;//cm
      //turn_count++;
      // if(turn_count>=7){
      //   turn_count=0;
      //   *id=encoder_stop;
      // }
      if(dis_turn>=PI*5.75){
        *id=follow;
        
        pid_track.integral = 0.0f;      // 清积分
        pid_track.prev_error = 0.0f;    // 清上一次误差（避免D项冲击）
        track_output = 0.0f;        // 输出也清零
      }
      break;
    case encoder_stop:
      
      break;
  }
}



volatile uint8_t x_target=0;
volatile uint8_t y_target=0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == key1_Pin){
    x_target++;
  }
  if(GPIO_Pin == key2_Pin){
    x_target++;
  }
  if(GPIO_Pin == key3_Pin){
    y_target++;
  }
}