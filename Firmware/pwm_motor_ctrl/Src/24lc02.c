
/*

 24lc02.c - описание функций работы с памятью

*/

#include "stm32f1xx_hal.h"
#include "delay.h"

extern I2C_HandleTypeDef hi2c1;

void MemWrite(unsigned char Addr,unsigned char Data)
{
	HAL_I2C_Mem_Write(&hi2c1,0x50 << 1,Addr,sizeof(Addr),&Data,sizeof(Data),50);	
	delay_ms(50);
}


unsigned char MemRead(unsigned char Addr)
{
	unsigned char Data;
	HAL_I2C_Mem_Read(&hi2c1,0x50 << 1,Addr,sizeof(Addr),&Data,sizeof(Data),50);
	return Data;
}

