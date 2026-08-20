#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_NUM
} MotorId_t;

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_BRAKE
} MotorDir_t;

void motor_init(void);
void motor_control(MotorId_t id, MotorDir_t dir, uint8_t speed);
void motor_stop(MotorId_t id);
void motor_brake(MotorId_t id);
void motor_setspeed(MotorId_t id, int16_t speed);
void motor_setlefrightspeed(int16_t left_speed, int16_t right_speed);

#endif