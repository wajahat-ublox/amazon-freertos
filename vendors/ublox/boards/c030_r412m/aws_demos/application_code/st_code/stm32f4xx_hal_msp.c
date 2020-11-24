/**
  ******************************************************************************
  * @file    Templates/Src/stm32f4xx_hal_msp.c
  * @author  MCD Application Team
  * @brief   HAL MSP module.
  *         
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]
    This file is generated automatically by STM32CubeMX and eventually modified 
    by the user

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#include "stm32f4xx_hal.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup HAL_MSP
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspInit(void)
{
  /* NOTE : This function is generated automatically by STM32CubeMX and eventually  
            modified by the user
   */ 
	    /* Structure storing the information of GPIO Port D. */

}

/**
  * @brief  DeInitializes the Global MSP.
  * @param  None  
  * @retval None
  */
void HAL_MspDeInit(void)
{
  /* NOTE : This function is generated automatically by STM32CubeMX and eventually  
            modified by the user
   */
}

void HAL_UART_MspInit( UART_HandleTypeDef * uartHandle )
{
		GPIO_InitTypeDef GPIO_InitStructure;

		if (uartHandle->Instance == USART1)
		{
				__USART1_CLK_ENABLE();
				__GPIOA_CLK_ENABLE();
			 
				GPIO_InitStructure.Pin = GPIO_PIN_9 | GPIO_PIN_10;
				GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
				GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
				GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
				GPIO_InitStructure.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
			
		}
		else if (uartHandle->Instance == USART2)
		{
				__USART2_CLK_ENABLE();
				__GPIOD_CLK_ENABLE();
			 
				GPIO_InitStructure.Pin = GPIO_PIN_5 | GPIO_PIN_6;
				GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
				GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
				GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
				GPIO_InitStructure.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
		} 
		else if (uartHandle->Instance == USART3)
		{				
				__USART3_CLK_ENABLE();
				__GPIOD_CLK_ENABLE();
			 
				GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_9;
				GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
				GPIO_InitStructure.Alternate = GPIO_AF7_USART3;
				GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
				GPIO_InitStructure.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
			
		} 
		else if (uartHandle->Instance == USART6)
		{
				__USART6_CLK_ENABLE();
				__GPIOD_CLK_ENABLE();
			 
				GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7;
				GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
				GPIO_InitStructure.Alternate = GPIO_AF8_USART6;
				GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
				GPIO_InitStructure.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
		}
}

void HAL_UART_MspDeInit( UART_HandleTypeDef * uartHandle )
{
    if (uartHandle->Instance == USART1)
		{
        __HAL_RCC_USART1_CLK_DISABLE();

        HAL_GPIO_DeInit( GPIOA, GPIO_PIN_9 | GPIO_PIN_10 );

        /* UART4 interrupt Deinit */
        HAL_NVIC_DisableIRQ( USART1_IRQn );
			
		}
		else if (uartHandle->Instance == USART2)
		{
        __HAL_RCC_USART2_CLK_DISABLE();

        HAL_GPIO_DeInit( GPIOD, GPIO_PIN_5 | GPIO_PIN_6 );

        /* UART4 interrupt Deinit */
        HAL_NVIC_DisableIRQ( USART2_IRQn );
		} 
		else if (uartHandle->Instance == USART3)
		{				
        __HAL_RCC_USART3_CLK_DISABLE();

        HAL_GPIO_DeInit( GPIOD, GPIO_PIN_8 | GPIO_PIN_9 );

        /* UART4 interrupt Deinit */
        HAL_NVIC_DisableIRQ( USART3_IRQn );
			
		} 
		else if (uartHandle->Instance == USART6)
		{
        __HAL_RCC_USART6_CLK_DISABLE();

        HAL_GPIO_DeInit( GPIOC, GPIO_PIN_6 | GPIO_PIN_7 );

        /* UART4 interrupt Deinit */
        HAL_NVIC_DisableIRQ( USART6_IRQn );
		}
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
