/**
 ******************************************************************************
 * File Name          : stm32f4xx_hal_msp.c
 * Description        : This file provides code for the MSP Initialization
 *                      and de-Initialization codes.
 ******************************************************************************
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
#include "stm32f4xx_hal.h"
#include "main.h";

extern void
Error_Handler(void);
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
 * Initializes the Global MSP.
 */
void
HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */
  /* USER CODE END MspInit 0 */

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

void
HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (hadc->Instance == ADC1)
    {
      /* USER CODE BEGIN ADC1_MspInit 0 */

      /* USER CODE END ADC1_MspInit 0 */
      /* Peripheral clock enable */
      __HAL_RCC_ADC1_CLK_ENABLE();

      /**ADC1 GPIO Configuration
       PA1     ------> ADC1_IN1
       */
      GPIO_InitStruct.Pin = GPIO_PIN_2;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      /* USER CODE BEGIN ADC1_MspInit 1 */
      /* USER CODE END ADC1_MspInit 1 */
    }

}

void
HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if (hadc->Instance == ADC1)
    {
      /* USER CODE BEGIN ADC1_MspDeInit 0 */

      /* USER CODE END ADC1_MspDeInit 0 */
      /* Peripheral clock disable */
      __HAL_RCC_ADC1_CLK_DISABLE();

      /**ADC1 GPIO Configuration
       PA1     ------> ADC1_IN1
       */
      HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    }
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */

}

void
HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (hdac->Instance == DAC)
    {
      /* USER CODE BEGIN DAC_MspInit 0 */

      /* USER CODE END DAC_MspInit 0 */
      /* Peripheral clock enable */
      __HAL_RCC_DAC_CLK_ENABLE();

      /**DAC GPIO Configuration
       PA4     ------> DAC_OUT1
       */
      GPIO_InitStruct.Pin = GPIO_PIN_4;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      /* USER CODE BEGIN DAC_MspInit 1 */

      /* USER CODE END DAC_MspInit 1 */
    }

}

void
HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
{

  if (hdac->Instance == DAC)
    {
      /* USER CODE BEGIN DAC_MspDeInit 0 */

      /* USER CODE END DAC_MspDeInit 0 */
      /* Peripheral clock disable */
      __HAL_RCC_DAC_CLK_DISABLE();

      /**DAC GPIO Configuration
       PA4     ------> DAC_OUT1
       */
      HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

    }
  /* USER CODE BEGIN DAC_MspDeInit 1 */

  /* USER CODE END DAC_MspDeInit 1 */

}

void
HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (hdcmi->Instance == DCMI)
    {
      /* USER CODE BEGIN DCMI_MspInit 0 */

      /* USER CODE END DCMI_MspInit 0 */
      /* Peripheral clock enable */
      __HAL_RCC_DCMI_CLK_ENABLE();

      /**DCMI GPIO Configuration
       PE4     ------> DCMI_D4
       PE5     ------> DCMI_D6
       PE6     ------> DCMI_D7
       PA6     ------> DCMI_PIXCK
       PC6     ------> DCMI_D0
       PC7     ------> DCMI_D1
       PC8     ------> DCMI_D2
       PC9     ------> DCMI_D3
       PB6     ------> DCMI_D5
       */
      GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
      HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

      GPIO_InitStruct.Pin = GPIO_PIN_6;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

      GPIO_InitStruct.Pin = GPIO_PIN_6;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      /* USER CODE BEGIN DCMI_MspInit 1 */

      /* USER CODE END DCMI_MspInit 1 */
    }

}

void
HAL_DCMI_MspDeInit(DCMI_HandleTypeDef* hdcmi)
{

  if (hdcmi->Instance == DCMI)
    {
      /* USER CODE BEGIN DCMI_MspDeInit 0 */

      /* USER CODE END DCMI_MspDeInit 0 */
      /* Peripheral clock disable */
      __HAL_RCC_DCMI_CLK_DISABLE();

      /**DCMI GPIO Configuration
       PE4     ------> DCMI_D4
       PE5     ------> DCMI_D6
       PE6     ------> DCMI_D7
       PA6     ------> DCMI_PIXCK
       PC6     ------> DCMI_D0
       PC7     ------> DCMI_D1
       PC8     ------> DCMI_D2
       PC9     ------> DCMI_D3
       PB6     ------> DCMI_D5
       */
      HAL_GPIO_DeInit(GPIOE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

      HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

      HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);

      HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    }
  /* USER CODE BEGIN DCMI_MspDeInit 1 */

  /* USER CODE END DCMI_MspDeInit 1 */

}

void
HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if (htim_base->Instance == TIM2)
    {
      /* USER CODE BEGIN TIM2_MspInit 0 */

      /* USER CODE END TIM2_MspInit 0 */
      /* Peripheral clock enable */
      __HAL_RCC_TIM2_CLK_ENABLE();
      /* USER CODE BEGIN TIM2_MspInit 1 */

      /* USER CODE END TIM2_MspInit 1 */
    }
  if (htim_base->Instance == TIM3)
      {
        /* USER CODE BEGIN TIM2_MspInit 0 */

        /* USER CODE END TIM2_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM3_CLK_ENABLE();
        /* USER CODE BEGIN TIM2_MspInit 1 */

        /* USER CODE END TIM2_MspInit 1 */
      }

}

void
HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (htim->Instance == TIM2)
    {
      /* USER CODE BEGIN TIM2_MspPostInit 0 */
      __GPIOA_CLK_ENABLE();
      /* USER CODE END TIM2_MspPostInit 0 */

      /**TIM2 GPIO Configuration
       PA3     ------> TIM2_CH4
       PA5     ------> TIM2_CH1
       */
      GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//      HAL_NVIC_EnableIRQ(TIM2_IRQn);
//      HAL_NVIC_SetPriority(TIM2_IRQn, 0, 2);

      /* USER CODE BEGIN TIM2_MspPostInit 1 */

      /* USER CODE END TIM2_MspPostInit 1 */
    }

}

void
HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if (htim_base->Instance == TIM2)
    {
      /* USER CODE BEGIN TIM2_MspDeInit 0 */

      /* USER CODE END TIM2_MspDeInit 0 */
      /* Peripheral clock disable */
      __HAL_RCC_TIM2_CLK_DISABLE();
    }
  if (htim_base->Instance == TIM3)
      {
        /* USER CODE BEGIN TIM2_MspDeInit 0 */

        /* USER CODE END TIM2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM3_CLK_DISABLE();
      }
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */

}

void
HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (huart->Instance == UART4)
    {
      /* USER CODE BEGIN UART4_MspInit 0 */

      /* USER CODE END UART4_MspInit 0 */
      /* Peripheral clock enable */
      __HAL_RCC_UART4_CLK_ENABLE();
      __GPIOA_CLK_ENABLE();

      /**UART4 GPIO Configuration
       PA0-WKUP     ------> UART4_TX
       PA1     ------> UART4_RX
       */
      GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      /* USER CODE BEGIN UART4_MspInit 1 */
      HAL_NVIC_EnableIRQ(UARTx_IRQn);
      HAL_NVIC_SetPriority(UARTx_IRQn, 0, 1);
      /* USER CODE END UART4_MspInit 1 */
    }

}

void
HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if (huart->Instance == UART4)
    {
      /* USER CODE BEGIN UART4_MspDeInit 0 */

      /* USER CODE END UART4_MspDeInit 0 */
      /* Peripheral clock disable */
      __HAL_RCC_UART4_CLK_DISABLE();

      /**UART4 GPIO Configuration
       PA0-WKUP     ------> UART4_TX
       PA1     ------> UART4_RX
       */
      HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);

    }
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
