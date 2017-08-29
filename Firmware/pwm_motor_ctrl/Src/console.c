/*

console.с - описание ф-ций работы с консолью UART

*/

#include "stm32f1xx_hal.h"
#include "delay.h"
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;

void ConsoleWrite(char *UARTBuff)
{
		HAL_GPIO_WritePin(RT_SWITCH_GPIO_Port,RT_SWITCH_Pin,GPIO_PIN_SET); // Запись в RS-485
		HAL_UART_Transmit(&huart1,(void *)UARTBuff,strlen(UARTBuff),10);	
}


int ConsoleRead(char *UARTBuff)
{
	char TmpBuff[255];
	int i=0;
	HAL_GPIO_WritePin(RT_SWITCH_GPIO_Port,RT_SWITCH_Pin,GPIO_PIN_RESET); // Чтение RS-485
	while(TmpBuff[i-1]!=0x0D)
	{
		HAL_UART_Receive(&huart1,(void *)&TmpBuff[i],sizeof(char),5000);
		HAL_GPIO_WritePin(RT_SWITCH_GPIO_Port,RT_SWITCH_Pin,GPIO_PIN_SET);
		delay_ms(1);
		HAL_UART_Transmit(&huart1,(void *)&TmpBuff[i],sizeof(char),50);
		HAL_GPIO_WritePin(RT_SWITCH_GPIO_Port,RT_SWITCH_Pin,GPIO_PIN_RESET);
		i++;
	}
	strcpy(UARTBuff,TmpBuff);
	ConsoleWrite("\r\n");
	return i;	
}

int ConsoleReadInteger()
{
	char TmpBuff[255];
	ConsoleRead(TmpBuff);
	return atoi(TmpBuff);
}


