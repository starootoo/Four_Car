#include "sortware_I2C.h"



void W_SCL(uint8_t state)
{
  HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, state);
  delay_us(10);
}

void W_SDA(uint8_t state)
{
  HAL_GPIO_WritePin(SDL_GPIO_Port, SDL_Pin, state);
  delay_us(10);
}

uint8_t R_SDA(void)
{
  uint8_t state;
  state = HAL_GPIO_ReadPin(SDL_GPIO_Port, SDL_Pin);
  delay_us(10);
  return state;
}



void sortware_I2C_Init(void)
{
  W_SCL(1);
  W_SDA(1);
}


void sortware_I2C_Start(void)
{
  W_SDA(1);
  W_SCL(1);
  W_SDA(0);
  W_SCL(0);
}

void sortware_I2C_Stop(void)
{
  W_SDA(0);
  W_SCL(1);
  W_SDA(1);

}

void sortware_I2C_Send(uint8_t byte)
{
  for(uint8_t i=0;i<8;i++)
  {
    W_SDA(byte & (0x80>>i));
    W_SCL(1);
    W_SCL(0);
  }
}


uint8_t sortware_I2C_Receive(void)
{
  uint8_t i, byte = 0x00;
  W_SDA(1);
  for(i=0;i<8;i++)
  {
    W_SCL(1);
    if(R_SDA() == 1){byte |= (0x80>>i);}
    W_SCL(0);
  }
  return byte;
}

void sortware_I2C_SAck(uint8_t ack)
{

    W_SDA(ack);
    W_SCL(1);
    W_SCL(0);

}

uint8_t sortware_I2C_RAck(void)
{
  uint8_t rack;
  W_SDA(1);
  W_SCL(1);
  rack=R_SDA();
  W_SCL(0);
  return rack;
}






