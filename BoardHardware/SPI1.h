#ifndef __SPI1_HARDWARE_H
#define __SPI1_HARDHARE_H

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
void SPI1_Init(void);
void SPI1_SetSpeed(u8 SpeedSet);
u8 SPI1_ReadWriteByte(u8 TxData);

void CSPin_Init(void);
void SPI1_CS1Pin_Init(void);
void SPI1_CS2Pin_Init(void);
void SPI1_CS3Pin_Init(void);


#endif
