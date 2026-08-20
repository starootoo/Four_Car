#include "pid.h"
#include "motor.h"
#include "encoder.h"


void pid_init(pid_t *pid, float Kp, float Ki, float Kd, float target_value, float out_min, float out_max){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->target_value = target_value;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float pid_calc(pid_t *pid,  float measured){//位置式PID
    float error = pid->target_value - measured;

    // 比例
    float p_out = pid->Kp * error;

    // 积分
    pid->integral += error;
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    if (pid->integral < pid->out_min) pid->integral = pid->out_min;
    float i_out = pid->Ki * pid->integral;

    // 微分
    float d_out = pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;

    float output = p_out + i_out + d_out;

    // 输出限幅
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output/10; // 除以10是为了将输出范围缩小到0-100之间，适应电机控制的速度范围
}

void pid_con_speed(pid_t *pid_left,pid_t *pid_right){
    float left_speed = speed_lpm;
    float right_speed = speed_rpm;

    float left_output = pid_calc(pid_left, left_speed);
    float right_output = pid_calc(pid_right, right_speed);

    if(left_output > 0){
        motor_control(MOTOR_LEFT, MOTOR_FORWARD, (uint16_t)left_output);
    }
    else if(left_output < 0){
        motor_control(MOTOR_LEFT, MOTOR_BACKWARD, (uint16_t)(-left_output));
    }
    else{
        motor_control(MOTOR_LEFT, MOTOR_STOP, 0);
    }

    if(right_output > 0){
        motor_control(MOTOR_RIGHT, MOTOR_FORWARD, (uint16_t)right_output);
    }
    else if(right_output < 0){
        motor_control(MOTOR_RIGHT, MOTOR_BACKWARD, (uint16_t)(-right_output));
    }
    else{
        motor_control(MOTOR_RIGHT, MOTOR_STOP, 0);
    }
}





