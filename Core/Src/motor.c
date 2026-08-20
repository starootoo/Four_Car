#include "motor.h"
#include "tim.h"

#define MOTOR_MAX_SPEED     100


//左电机为A，右电机为B
void motor_init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);//左电机
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);//右电机

    motor_control(MOTOR_LEFT,  MOTOR_STOP, 0);
    motor_control(MOTOR_RIGHT, MOTOR_STOP, 0);
}

void motor_control(MotorId_t id, MotorDir_t dir, uint8_t speed)
{
    if (speed > MOTOR_MAX_SPEED) speed = MOTOR_MAX_SPEED;

    switch (id)
    {
    case MOTOR_LEFT:
        switch (dir)
        {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
            break;
        case MOTOR_BACKWARD:
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
            break;
        case MOTOR_BRAKE:
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
            break;
        case MOTOR_STOP:
        default:
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
            break;
        }
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
        break;

    case MOTOR_RIGHT:
        switch (dir)
        {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
            break;
        case MOTOR_BACKWARD:
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
            break;
        case MOTOR_BRAKE:
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
            break;
        case MOTOR_STOP:
        default:
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
            break;
        }
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
        break;

    default:
        break;
    }
}

void motor_stop(MotorId_t id)
{
    motor_control(id, MOTOR_STOP, 0);
}

void motor_brake(MotorId_t id)
{
    motor_control(id, MOTOR_BRAKE, 0);
}

void motor_setspeed(MotorId_t id, int16_t speed)
{
    MotorDir_t dir;

    if (speed > 0)
    {
        dir = MOTOR_FORWARD;
        if (speed > MOTOR_MAX_SPEED) speed = MOTOR_MAX_SPEED;
    }
    else if (speed < 0)
    {
        dir = MOTOR_BACKWARD;
        speed = -speed;
        if (speed > MOTOR_MAX_SPEED) speed = MOTOR_MAX_SPEED;
    }
    else
    {
        dir = MOTOR_STOP;
        speed = 0;
    }

    motor_control(id, dir, (uint8_t)speed);
}

void motor_setlefrightspeed(int16_t left_speed, int16_t right_speed)
{
    motor_setspeed(MOTOR_LEFT, left_speed);
    motor_setspeed(MOTOR_RIGHT, right_speed);
}