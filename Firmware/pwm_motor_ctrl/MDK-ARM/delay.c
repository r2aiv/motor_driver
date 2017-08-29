#include "stm32f1xx_hal.h"

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

void delay_us(uint32_t us)
{
	int32_t us_count_tick =  us * (SystemCoreClock/1000000);
	//��������� ������������ �������
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        //�������� �������� �������� ��������
	DWT_CYCCNT  = 0;
        //��������� �������
	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk; 
	while(DWT_CYCCNT < us_count_tick);
        //������������� �������
	DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk;
	
}
//////////////////////////////
void delay_ms(uint32_t ms)
{
	int32_t ms_count_tick =  ms * (SystemCoreClock/1000);
	//��������� ������������ �������
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
         //�������� �������� �������� ��������
	DWT_CYCCNT  = 0;
        //��������� �������
	DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; 
	while(DWT_CYCCNT < ms_count_tick);
        //������������� �������
	DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk;
	
}
