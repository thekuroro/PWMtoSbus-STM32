/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "Sbus.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
CH_Pwm_Val_t CH_Pwm_Val[MAX_RC_CH_NB];
uint8_t i;
uint16_t Sbus_CH[16]  	= {1025,1025,1025,1025,1025,1025,1025,1025,1025,1025,1025,1025,1025,1025,1025,1025};
TIM_IC_InitTypeDef sConfigIC;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // RC channels 1 to 3
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);

  // RC channels 4 to 7
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4);

  // RC channel 8
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);



  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	for(i=0;i<USED_RC_CH_NB;i++){
	  		if(CH_Pwm_Val[i].CH_Detected == CH_DETECTED){
	  			Sbus_CH[i] = ((CH_Pwm_Val[i].Delta / PWM_Ratio) - 880) / 0.625f;
	  			CH_Pwm_Val[i].CH_Detected = CH_NOT_DETECTED;
	  		}
	  		else
	  			Sbus_CH[i] = 0;
	  	}

	  SBUS_write(Sbus_CH);
	  HAL_Delay(7);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint8_t Ch_Idx;
	uint8_t Ch_Ct_Idx;

	/*****************************************
	 * 				CH 1 to 8 sampling
	 *****************************************/

	Ch_Idx 		= htim->Channel >> 1;

	switch(htim->Channel)
	{
		case HAL_TIM_ACTIVE_CHANNEL_1:
			Ch_Idx 		= 0;
			Ch_Ct_Idx 	= TIM_CHANNEL_1;
		break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
			Ch_Idx 		= 1;
			Ch_Ct_Idx 	= TIM_CHANNEL_2;
		break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
			Ch_Idx 		= 2;
			Ch_Ct_Idx 	= TIM_CHANNEL_3;
		break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
			Ch_Idx 		= 3;
			Ch_Ct_Idx 	= TIM_CHANNEL_4;
		break;
	}

	if(htim->Instance == TIM2)
		Ch_Idx += TIMER_1_CH_NB;
	else if(htim->Instance == TIM3)
		Ch_Idx += TIMER_1_CH_NB + TIMER_2_CH_NB;

	// Raising edge
		if(CH_Pwm_Val[Ch_Idx].Current_Edge == RAISING)
		{
			// Set Edge detect flag
			CH_Pwm_Val[Ch_Idx].CH_Detected =  CH_DETECTED;

			// Set IC polarity to Falling
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
			HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, Ch_Ct_Idx);
			CH_Pwm_Val[Ch_Idx].Current_Edge = FALLING;

			// Store raising edge timer value
			CH_Pwm_Val[Ch_Idx].Rising = HAL_TIM_ReadCapturedValue(htim,Ch_Ct_Idx);
		}
		// Falling edge
		else{
			// Set IC polarity to Raising
			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
			HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, Ch_Ct_Idx);
			CH_Pwm_Val[Ch_Idx].Current_Edge = RAISING;

			// Store falling edge timer value
			CH_Pwm_Val[Ch_Idx].Falling = HAL_TIM_ReadCapturedValue(htim,Ch_Ct_Idx);

			// Compute delta value between raising and falling edge
			if(CH_Pwm_Val[Ch_Idx].Rising < CH_Pwm_Val[Ch_Idx].Falling)
				CH_Pwm_Val[Ch_Idx].Delta = CH_Pwm_Val[Ch_Idx].Falling - CH_Pwm_Val[Ch_Idx].Rising;
			else
				CH_Pwm_Val[Ch_Idx].Delta = (TIMER_MAX_VAL - CH_Pwm_Val[Ch_Idx].Rising) + CH_Pwm_Val[Ch_Idx].Falling + 1;
		}

		// Start IC interrupt after polarity inversion
		HAL_TIM_IC_Start_IT(htim,Ch_Ct_Idx);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
