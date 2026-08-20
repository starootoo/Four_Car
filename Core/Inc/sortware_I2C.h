#ifndef __SORTWARE_I2C_H__
#define __SORTWARE_I2C_H__
#include "main.h"
#include "delay.h"


void sortware_I2C_Init(void);
void sortware_I2C_Start(void);
void sortware_I2C_Stop(void);
void sortware_I2C_Send(uint8_t byte);
uint8_t sortware_I2C_Receive(void);
void sortware_I2C_SAck(uint8_t ack);
uint8_t sortware_I2C_RAck(void);









#endif
