/*

fet_switch.c - �������� ������� ���������� �������� �������

*/

#include "main.h"
#include "stm32f1xx_hal.h"
#include "delay.h"

#define STATE_FORWARD 	0x01
#define STATE_BACKWARD 	0x02
#define STATE_HARDSTOP	0x03
#define STATE_STOP 			0x00

extern TIM_HandleTypeDef htim2;
unsigned char DevState;
unsigned char PWMValue;


void FET_Stop(void)						// ������� (��������� ���������� � �������� ������)
{
	HAL_GPIO_WritePin(GATE1_GPIO_Port,GATE1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GATE2_GPIO_Port,GATE2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GATE3_GPIO_Port,GATE3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GATE4_GPIO_Port,GATE4_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);	
}	

void FET_ForwardOn(void)    	// �������� ������ �������� �������
{
	HAL_GPIO_WritePin(GATE1_GPIO_Port,GATE1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GATE4_GPIO_Port,GATE4_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
}


void FET_BackwardOn(void)   	// �������� �������� �������� �������
{
	HAL_GPIO_WritePin(GATE2_GPIO_Port,GATE2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GATE3_GPIO_Port,GATE3_Pin,GPIO_PIN_SET);	
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
}

void FET_HardStop(void)			// ������� ������� (����������� ����� Q3 � Q5, ������� ��������������)
{
	FET_Stop();
	HAL_GPIO_WritePin(GATE2_GPIO_Port,GATE2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GATE4_GPIO_Port,GATE4_Pin,GPIO_PIN_SET);
}

void FET_ForwardPWM(unsigned char pwm)		// ��������� ��������� � ������������ ����������� ��� pwm - ���������� � ���������
{
			
}

void FET_BackwardPWM(unsigned char pwm)	  // �������� �������� �� ���
{
	
}

void MagnetOn(void) 					// ��������� �������
{
	HAL_GPIO_WritePin(MAGNET_GPIO_Port,MAGNET_Pin,GPIO_PIN_SET);
}

void MagnetOff(void) 				// ���������� �������
{
	HAL_GPIO_WritePin(MAGNET_GPIO_Port,MAGNET_Pin,GPIO_PIN_RESET);
}

void FET_SoftStart(void)			// ������� ����. 
{
	for(int pwm=1;pwm<=100;pwm++)
	{
		switch(DevState)
		{
			case STATE_FORWARD:
				FET_ForwardOn();
				delay_us(pwm*100);
				FET_Stop();
			  delay_us(10000-pwm*100);
				break;
			case STATE_BACKWARD:
				FET_BackwardOn();
				delay_us(pwm*100);
				FET_Stop();
			  delay_us(10000-pwm*100);
				break;
			default:
				FET_Stop();
			break;
			
		}		
	}
}
void FET_SoftStop(void)			// ������� �������. 
{
	for(int pwm=1;pwm<=100;pwm++)
	{
		switch(DevState)
		{
			case STATE_FORWARD:
				FET_ForwardOn();
				delay_us(10000-pwm*100);
				FET_Stop();
			  delay_us(pwm*100);
				break;
			case STATE_BACKWARD:
				FET_BackwardOn();
				delay_us(10000-pwm*100);
				FET_Stop();
			  delay_us(pwm*100);
				break;
			default:
				FET_Stop();
			break;
			
		}		
	}
}

