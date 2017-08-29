/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "fet_switch.h"
#include "sensor.h"
#include "delay.h"
#include "24lc02.h"
#include "console.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

char UARTBuff[255];								// Буфер UART

#define STATE_FORWARD 	0x01			// Вперед
#define STATE_BACKWARD 	0x02			// Назад
#define STATE_HARDSTOP	0x03			// Тормоз
#define STATE_STOP 			0x00			// Остановка

#define I2C_MEM_ADDR 		0x50			// Адрес I2C микросхемы EEPROM

extern unsigned char DevState;		// Режим работы
extern unsigned char PWMValue;		// Текущее значение PWM
unsigned char SensorState;				// Состояние концевиков
unsigned char PrevSensorState=0;	// Предыдущее состояние концевиков
unsigned char PWMSetting=0; 			// Уставка PWM

HAL_StatusTypeDef	flash_ok = HAL_ERROR;

struct PWMSettings								// Структура для хранения настроек (хранится в EEPROM)
{
	unsigned int ForwardSpeed;			// Скорость подъема
	unsigned int BackwardSpeed;		// Скорость опускания
	unsigned int  StopDelay;				// Задержка при накладывании тормоза при опускании
	unsigned int  SoftStartDelay;		// Задержка для плавного пуска
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
	struct PWMSettings MySettings;
	
	// Уставки
	
	MySettings.ForwardSpeed=90;  				// Подъем (% заполнения ШИМ)
	MySettings.BackwardSpeed=20;				// Опускание (% заполнения ШИМ)
	MySettings.SoftStartDelay=20;				// Задержка приращения скорости нарастания плавного пуска
	MySettings.StopDelay=500;						// Задержка перед накладыванием тормоза при опускании

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
	
	SensorState=GetSensorState();
	DevState=STATE_STOP;
	
	HAL_GPIO_WritePin(RT_SWITCH_GPIO_Port,RT_SWITCH_Pin,GPIO_PIN_SET); // Запись в RS-485
	
	sprintf(UARTBuff,"Forward speed: %d\r\n",MySettings.ForwardSpeed);
	ConsoleWrite(UARTBuff);
	sprintf(UARTBuff,"Backward speed: %d\r\n",MySettings.BackwardSpeed);
	ConsoleWrite(UARTBuff);
	sprintf(UARTBuff,"Backward stop delay: %d\r\n",MySettings.StopDelay);
	ConsoleWrite(UARTBuff);
	
	HAL_TIM_Base_Start_IT(&htim2);  // Поехали (с) Ю. Гагарин

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
	
  while (1)
  {
		
// Маска датчиков: 	0x01 - прожектор 
//									0x02 - концевик верх
//									0x04 - концевик низ

		SensorState=GetSensorState();
		
		if((SensorState & 0x01) == 0x01) // Прожектор вверху
		{
			ConsoleWrite("Projector is UP!\r");
			//if((SensorState & 0x02)==0x02) break; // Лифт вверху - выходим
			if((SensorState & 0x04)==0x04) // Нижний концевик активен
			{
				ConsoleWrite("Lift is down, start moving UP\r\n");
				// TODO: подъем
				PWMValue=MySettings.ForwardSpeed;
				DevState=STATE_FORWARD; //Едем вверх
				while(PWMValue++ <= PWMSetting-2) delay_ms(MySettings.SoftStartDelay);		// Плавный пуск
				while((GetSensorState() & 0x04)==0x04); // Ждем схода с концевика
				ConsoleWrite("Down switch if OFF\r\n");
				while((GetSensorState() & 0x02)!=0x02); // Ждем прихода на концевик
				ConsoleWrite("Up switch is ON\r\n");
				MagnetOn();
				ConsoleWrite("Magnet is ON\r\n");
				DevState=STATE_HARDSTOP;				
			}		
		}					
		
		if((SensorState & 0x01) == 0x00) //Прожектор внизу			
		{
			PWMValue=MySettings.BackwardSpeed;
			ConsoleWrite("Projector is DOWN!\r");
			//if((SensorState & 0x04)==0x04) break; // Лифт внизу - выходим
			if((SensorState & 0x02)==0x02) // Верхний концевик активен
			{
				// TODO: Опускание
			
				PWMValue=1;
				PWMSetting=MySettings.BackwardSpeed;
				if(GetSensorState() & 0x02) //Лифт вверху 
				{
					ConsoleWrite("Lift is DOWN, moving UP\r\n");	
					ConsoleWrite("Magnet is OFF\r\n");
					MagnetOff();
					DevState=STATE_BACKWARD; //Едем вниз				
					while(PWMValue++ <= PWMSetting-2) delay_ms(MySettings.SoftStartDelay);	// Плавный пуск
					while((GetSensorState() & 0x02)==0x02); // Ждем схода с концевика
					ConsoleWrite("Up switch is OFF\r\n");
					delay_ms(MySettings.StopDelay); // Работаем 0.5с, и ставим на тормоз для опускания самоходом
					DevState=STATE_HARDSTOP;
					while((GetSensorState() & 0x04)!=0x04); // Ждем прихода на концевик
					ConsoleWrite("Down switch is ON\r\n");
					DevState=STATE_HARDSTOP;				
				}				
			}	
		}

		DevState=STATE_STOP;	
		PrevSensorState=SensorState;
		delay_ms(100);		
 } // loop ends
} // main() ends
  /* USER CODE END WHILE */
				

  /* USER CODE BEGIN 3 */

  
  /* USER CODE END 3 */



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GATE1_Pin|GATE2_Pin|GATE3_Pin|GATE4_Pin 
                          |MAGNET_Pin|RT_SWITCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|WP_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GATE1_Pin GATE2_Pin GATE3_Pin GATE4_Pin 
                           MAGNET_Pin RT_SWITCH_Pin */
  GPIO_InitStruct.Pin = GATE1_Pin|GATE2_Pin|GATE3_Pin|GATE4_Pin 
                          |MAGNET_Pin|RT_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSE_TTL0_Pin SENSE_TTL1_Pin SENSE_TTL2_Pin SENSE_TTL3_Pin */
  GPIO_InitStruct.Pin = SENSE_TTL0_Pin|SENSE_TTL1_Pin|SENSE_TTL2_Pin|SENSE_TTL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin WP_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|WP_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
