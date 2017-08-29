/*

sensor.c - Описание функций работы с датчиками

*/

#include "stm32f1xx_hal.h"

unsigned char GetSensorState(void)
{
return ~GPIOB->IDR & 0x0F; // Битвая маска состояния датчиков;
}
