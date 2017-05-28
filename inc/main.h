//#include "stm8s.h"
//#include "stm8s_uart2.h"
//#include "stm8s_it.h"
//#include "core_cm4.h"
//#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_it.h"

// UART def`s
UART_HandleTypeDef UartHandle;

#define UARTx                           UART4

#define UARTx_CLK_ENABLE()              __UART4_CLK_ENABLE()
#define UARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define UARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define UARTx_FORCE_RESET()             __UART4_FORCE_RESET()
#define UARTx_RELEASE_RESET()           __UART4_RELEASE_RESET()

/* Definition for UARTx Pins */
#define UARTx_TX_PIN                    GPIO_PIN_0
#define UARTx_TX_GPIO_PORT              GPIOA
#define UARTx_TX_AF						GPIO_AF8_UART4
#define UARTx_RX_PIN                    GPIO_PIN_1
#define UARTx_RX_GPIO_PORT              GPIOA
#define UARTx_RX_AF						GPIO_AF8_UART4

/* Definition for USARTx's NVIC */
#define UARTx_IRQn                      UART4_IRQn
#define UARTx_IRQHandler                UART4_IRQHandler

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      1024

/* GPIOD */
#define GPIOA_Clock_Enable()                      __GPIOA_CLK_ENABLE()
#define GPIOD_Clock_Enable()			  __GPIOD_CLK_ENABLE()
#define GPIOE_Clock_Enable()                      __GPIOE_CLK_ENABLE()

// TIM def`s
TIM_HandleTypeDef htim2;
TIM_OC_InitTypeDef sConfigOC;

//#define TIM1_IRQn                               TIM1_BRK_TIM9_IRQn
#define TIM2_IRQn                               TIM2_IRQn

//#define TIM1_IRQHandler                         TIM1_IRQHandler
#define TIM2_IRQHandler                         TIM2_IRQHandler

//#define TIM1_CLK_ENABLE() __HAL_RCC_TIM1_CLK_ENABLE()
#define TIM2_CLK_ENABLE() __HAL_RCC_TIM2_CLK_ENABLE()

//#define TIM1_PORT GPIOE
//#define TIM1_Pin GPIO_PIN_9

#define TIM2_PORT GPIOA
#define TIM2_Speed_Pin GPIO_PIN_15
#define TIM2_Rotation_Pin GPIO_PIN_3

static void
Error_Handler(void);
void
SystemClock_Config(void);
