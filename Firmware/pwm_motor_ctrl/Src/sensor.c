/*

sensor.c - �������� ������� ������ � ���������

*/

#include "stm32f1xx_hal.h"

unsigned char GetSensorState(void)
{
return ~GPIOB->IDR & 0x0F; // ������ ����� ��������� ��������;
}
