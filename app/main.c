#include "main.h"
#include "esp8266.h"

#define uart_rx_buffer_size 1024
#define CCR1_Val  ((u16)500) // Configure channel 1 Pulse Width
#define CCR2_Val  ((u16)250) // Configure channel 2 Pulse Width
#define CCR3_Val  ((u16)750) // Configure channel 3 Pulse Width
ESP_State_TypeDef esp_state = ESP_Init;
Up_Down_State UpDown = Hold;
Left_Right_State LeftRight = CNT;
GPIO_InitTypeDef LED, Rotation_Left, Rotation_Right, Forward, Backward;
ADC_HandleTypeDef hadc1;
DAC_HandleTypeDef hdac;
DCMI_HandleTypeDef hdcmi;
volatile char uart_rx_buff[RXBUFFERSIZE];
volatile char uart_tx_buff[1024];
volatile char *uart_rx_buff_addr;
volatile bool UART_Ready = false;
bool RecStart = false;
uint16_t Speed = 0;
uint16_t Rotation = 0;

static void
MX_TIM2_Init(void);
void
Config_UART4(uint32_t BaudRate);
void
Config_ESP(void);
void
Config_PWM(uint16_t PWM_Val_Speed, uint16_t PWM_Val_Rotation);
void
GPIO_Config(void);
void
send_at_command(char *command, char *answer, uint8_t length);
void
Wait_Answer(char *answer);
void
Rotation_Movement(void);
void
flush_uart_buffer(void);
void
ReceiveCommand(void);

int
main(void)
{
  SystemInit();
  HAL_Init();
  SystemClock_Config();
  MX_TIM2_Init();
  uint32_t SysClk = HAL_RCC_GetSysClockFreq();
  GPIO_Config();
  Config_UART4(115200);
  while (1)
    {
      ReceiveCommand();
//      Error_Handler();
    }
}

void
Config_UART4(uint32_t BaudRate)
{
  UartHandle.Instance = UART4;

  UartHandle.Init.BaudRate = BaudRate;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&UartHandle) != HAL_OK)
    {
      Error_Handler();
    }
  uart_rx_buff_addr = uart_rx_buff;
}

void
send_at_command(char *command, char *answer, uint8_t length)
{
  UART_Ready = false;
  if (HAL_UART_Transmit_IT(&UartHandle, command, length) != HAL_OK)
    {
      Error_Handler();
    }
  while (!UART_Ready)
    ;
  Wait_Answer(answer);
}

void
Wait_Answer(char *answer)
{
  bool ret = true;
  HAL_StatusTypeDef UartRecErr;
  UART_Ready = false;
  if ((UartRecErr = HAL_UART_Receive_IT(&UartHandle, uart_rx_buff, RXBUFFERSIZE))
      != HAL_OK)
    {
      Error_Handler();
    }
  while (strstr(uart_rx_buff, answer) == 0)
    ;
  if (UartRecErr = HAL_UART_AbortReceive_IT(&UartHandle) != HAL_OK)
    Error_Handler();
  while (!UART_Ready)
    ;
  flush_uart_buffer();
}

void
flush_uart_buffer(void)
{
  for (uint16_t i = 0; i <= RXBUFFERSIZE; i++)
    uart_rx_buff[i] = 0;
  uart_rx_buff_addr = uart_rx_buff;
}

void
Config_ESP(void)
{
//  send_at_command(AT_RST, "ready", sizeof(AT_RST) - 1);
  send_at_command(ATE0, "OK\r\n", sizeof(ATE0) - 1);
  send_at_command(AT, "OK\r\n", sizeof(AT) - 1);
  send_at_command("AT+CIPSTAMAC?\r\n", "OK\r\n",
      sizeof("AT+CIPSTAMAC?\r\n") - 1);
  send_at_command(AT_CWJAP, "OK\r\n", sizeof(AT_CWJAP) - 1);
  send_at_command(AT_CWMODE_1, "OK\r\n", sizeof(AT_CWMODE_2) - 1);
//  send_at_command(AT_CIPAP, "OK\r\n", sizeof(AT_CIPAP) - 1);
//  send_at_command(AT_CWSAP, "OK\r\n", sizeof(AT_CWSAP) - 1);
  send_at_command(AT_CIPMUX1, "OK\r\n", sizeof(AT_CIPMUX1) - 1);
  send_at_command(AT_CIPSERVER, "OK\r\n", sizeof(AT_CIPSERVER) - 1);
  send_at_command(AT_CIPSTA, "OK\r\n", sizeof(AT_CIPSTA) - 1);
  send_at_command("AT+CIFSR\r\n", "OK\r\n", sizeof("AT+CIFSR\r\n") - 1);
  HAL_GPIO_WritePin(GPIOD, LED.Pin, SET);
  esp_state = ESP_Ready;
}

void
GPIO_Config(void)
{
  GPIOD_Clock_Enable();
  LED.Mode = GPIO_MODE_OUTPUT_PP;
  LED.Pin = GPIO_PIN_12;
  LED.Pull = GPIO_PULLDOWN;
  LED.Speed = GPIO_SPEED_LOW;

  Forward.Mode = GPIO_MODE_OUTPUT_PP;
  Forward.Pin = GPIO_PIN_11;
  Forward.Pull = GPIO_PULLDOWN;
  Forward.Speed = GPIO_SPEED_LOW;

  Backward.Mode = GPIO_MODE_OUTPUT_PP;
  Backward.Pin = GPIO_PIN_13;
  Backward.Pull = GPIO_PULLDOWN;
  Backward.Speed = GPIO_SPEED_LOW;

  Rotation_Left.Mode = GPIO_MODE_OUTPUT_PP;
  Rotation_Left.Pin = GPIO_PIN_14;
  Rotation_Left.Pull = GPIO_PULLDOWN;
  Rotation_Left.Speed = GPIO_SPEED_LOW;

  Rotation_Right.Mode = GPIO_MODE_OUTPUT_PP;
  Rotation_Right.Pin = GPIO_PIN_15;
  Rotation_Right.Pull = GPIO_PULLDOWN;
  Rotation_Right.Speed = GPIO_SPEED_LOW;

  HAL_GPIO_Init(GPIOD, &LED);
  HAL_GPIO_Init(GPIOD, &Forward);
  HAL_GPIO_Init(GPIOD, &Backward);
  HAL_GPIO_Init(GPIOD, &Rotation_Left);
  HAL_GPIO_Init(GPIOD, &Rotation_Right);

  HAL_GPIO_WritePin(GPIOD, LED.Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, Forward.Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, Backward.Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, Rotation_Left.Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, Rotation_Right.Pin, GPIO_PIN_RESET);

}

void
ReceiveCommand(void)
{
  switch (esp_state)
    {
  case ESP_Init:
    {
      Config_ESP();
      break;
    }
  case ESP_Ready:
    {
      Wait_Answer("CONNECT\r\n");
      HAL_GPIO_WritePin(GPIOD, LED.Pin, RESET);
      esp_state = ESP_CommandState;
      flush_uart_buffer();
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
      break;
    }
  case ESP_CommandState:
    {
      UART_Ready = false;
      if (HAL_UART_Receive_IT(&UartHandle, uart_rx_buff, 11) != HAL_OK)
        {
          Error_Handler();
        }
      while (!UART_Ready)
        ;
      uint8_t count = uart_rx_buff[9] - 48;
      UART_Ready = false;
      if (HAL_UART_Receive_IT(&UartHandle, uart_rx_buff + 11, count) != HAL_OK)
        {
          Error_Handler();
        }
      while (!UART_Ready)
        ;
      UART_Ready = false;
      if (HAL_UART_Receive_IT(&UartHandle, uart_rx_buff + 11 + count, 11)
          != HAL_OK)
        {
          Error_Handler();
        }
      while (!UART_Ready)
        ;
      uint8_t count2 = uart_rx_buff[11 + count + 9] - 48;
      //      flush_uart_buffer();
      UART_Ready = false;
      if (HAL_UART_Receive_IT(&UartHandle, uart_rx_buff + 22 + count, count2)
          != HAL_OK)
        {
          Error_Handler();
        }
      while (!UART_Ready)
        ;
      if (strstr(uart_rx_buff, "UP") != 0)
        {
          if (UpDown == UP)
            {
              if (Speed < 1000)
                {
                  Speed = Speed + 100;
                  UpDown = UP;
                }
            }
          else
            {
              Speed = 0;
              UpDown = UP;
            }
        }
      if (strstr(uart_rx_buff, "DOWN") != 0)
        {
          if (UpDown == Down)
            {
              if (Speed < 1000)
                {
                  Speed = Speed + 100;
                  UpDown = Down;
                }
            }
          else
            {
              Speed = 0;
              UpDown = Down;
            }
        }
      if (strstr(uart_rx_buff, "LEFT") != 0)
        {
          if (LeftRight == Left)
            {
              if (Rotation < 1000)
                Rotation = Rotation + 100;
              LeftRight = Left;
            }
          else
            {
              Rotation = 0;
              LeftRight = Left;
            }
        }
      if (strstr(uart_rx_buff, "RIGHT") != 0)
        {
          if (LeftRight == Right)
            {
              if (Rotation < 1000)
                Rotation = Rotation + 100;
              LeftRight = Right;
            }
          else
            {
              Rotation = 0;
              LeftRight = Right;
            }
        }
      if (strstr(uart_rx_buff, "HOLD") != 0)
        {
          Speed = 0;
          UpDown = Hold;
        }
      if (strstr(uart_rx_buff, "CNT") != 0)
        {
          Rotation = 0;
          LeftRight = CNT;
        }
      if (strstr(uart_rx_buff, "DISCONNECT") != 0)
        {
          esp_state = ESP_Ready;
        }
      flush_uart_buffer();
      char SendCmd[sizeof(AT_CIPSEND) + 5];
      strcpy(SendCmd, AT_CIPSEND);
      strcat(SendCmd, "0,8\r\n");
      UART_Ready = false;
      if (HAL_UART_Transmit_IT(&UartHandle, SendCmd, sizeof(SendCmd)) != HAL_OK)
        Error_Handler();
      while (!UART_Ready)
        ;
      Wait_Answer(">");
      UART_Ready = false;
      if (HAL_UART_Transmit_IT(&UartHandle, "Accept\r\n", sizeof("Accept\r\n"))
          != HAL_OK)
        Error_Handler();
      while (!UART_Ready)
        ;
      Wait_Answer("SEND OK\r\n");
      Rotation_Movement();
      break;
    }
    }
}

void
Config_PWM(uint16_t PWM_Val_Speed, uint16_t PWM_Val_Rotation)
{
  HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_ALL);
  sConfigOC.Pulse = PWM_Val_Speed;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  sConfigOC.Pulse = PWM_Val_Rotation;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
}

static void
Error_Handler(void)
{
  while (1)
    {
      HAL_GPIO_TogglePin(GPIOD, LED.Pin);
      HAL_Delay(100);
    }
}

void
HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Set transmission flag: transfer complete */
  UART_Ready = true;
}

void
HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Set transmission flag: transfer complete */
  UART_Ready = true;
}

void
HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
//  UNUSED(huart);
  UART_Ready = true;

  /* NOTE : This function should not be modified, when the callback is needed,
   the HAL_UART_AbortTransmitCpltCallback can be implemented in the user file.
   */
}

void
HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* NOTE: This function Should not be modified, when the callback is needed,
   the HAL_UART_ErrorCallback could be implemented in the user file
   */
  uint32_t UartError;
  UartError = huart->ErrorCode;
  Error_Handler();
}

void
Rotation_Movement(void)
{
  if (UpDown == UP)
    {
      HAL_GPIO_WritePin(GPIOD, Backward.Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, Forward.Pin, GPIO_PIN_SET);
    }
  if (UpDown == Down)
    {
      HAL_GPIO_WritePin(GPIOD, Forward.Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, Backward.Pin, GPIO_PIN_SET);
    }
  if (UpDown == Hold)
    {
      HAL_GPIO_WritePin(GPIOD, Forward.Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, Backward.Pin, GPIO_PIN_RESET);
    }
  if (LeftRight == Left)
    {
      HAL_GPIO_WritePin(GPIOD, Rotation_Right.Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, Rotation_Left.Pin, GPIO_PIN_SET);
    }
  if (LeftRight == Right)
    {
      HAL_GPIO_WritePin(GPIOD, Rotation_Left.Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, Rotation_Right.Pin, GPIO_PIN_SET);
    }
  if (LeftRight == CNT)
    {
      HAL_GPIO_WritePin(GPIOD, Rotation_Left.Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, Rotation_Right.Pin, GPIO_PIN_RESET);
    }
  Config_PWM(Speed, Rotation);
}

void UARTx_IRQHandler(void)
{
  HAL_UART_IRQHandler(&UartHandle);

}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

void
SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /**Configure the main internal regulator output voltage
   */__HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
      Error_Handler();
    }

  /**Configure the Systick interrupt time
   */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
   */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void
MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
      Error_Handler();
    }

  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }

}

/* DAC init function */
static void
MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

  /**DAC Initialization
   */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
    {
      Error_Handler();
    }

  /**DAC channel OUT1 config
   */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }

}

/* DCMI init function */
static void
MX_DCMI_Init(void)
{

  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_EMBEDDED;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.SyncroCode.FrameEndCode = 0;
  hdcmi.Init.SyncroCode.FrameStartCode = 0;
  hdcmi.Init.SyncroCode.LineStartCode = 0;
  hdcmi.Init.SyncroCode.LineEndCode = 0;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
    {
      Error_Handler();
    }

}

/* TIM2 init function */
static void
MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 21;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
      Error_Handler();
    }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
      Error_Handler();
    }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
      Error_Handler();
    }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
    {
      Error_Handler();
    }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
      Error_Handler();
    }

  HAL_TIM_MspPostInit(&htim2);

}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */

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

