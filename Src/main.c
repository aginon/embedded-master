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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>

#include <agn_buzzer.h>
#include <agn_errno.h>
#include <agn_gateway.h>
#include <agn_logger.h>
#include <agn_micros.h>
#include <agn_packet.h>
#include <agn_range_finder.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

DCMI_HandleTypeDef hdcmi;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ----------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_DCMI_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM10_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// run this code every x seconds
void AGN_CHECK_ERRORS() {
	if (getErrno() != 0) {
		char s[20];
		sprintf(s, "Error Code = 0x%02x", getErrno());
		AGN_LOG_ERROR(s);
		resetErrno();
	}
}

int buzzer_state = 0;
GPIO_PinState gwhb_state = 0;
const int gwhb_threshold = 3;
int gwhb_counter = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//AGN_LOG_DEBUG("TIM_INTERRUPT");
	if (htim->Instance == TIM7) {
		// TIM 7 set to 10 Hz
		HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
		AGN_CHECK_ERRORS();

	} else if (htim->Instance == TIM6){
		// TIM 6 set to 10kHz

		AGN_BUZZER_TICK();
	} else if (htim->Instance == TIM10) {
		// TIM 10 set to 0.2 Hz

		// Toggle Heartbeat OUT
		HAL_GPIO_TogglePin(GWHB_OUT_GPIO_Port, GWHB_OUT_Pin);

		// Check Heartbeat IN
		GPIO_PinState new_gwhb_state = HAL_GPIO_ReadPin(GWHB_IN_GPIO_Port, GWHB_IN_Pin);
		if (gwhb_state != new_gwhb_state) {
			// Heartbeat is normal
			gwhb_counter = 0;
		} else {
			// Heartbeat is malfunctioning
			gwhb_counter++;
			if (gwhb_counter >= gwhb_threshold) {
				setErrno(AGN_ERRNO_GATEWAY_HB_ERROR);
			}
		}

		gwhb_state = new_gwhb_state;

	} else {
		UNUSED(htim);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
/*
	char str[100];
	sprintf(str, "EXTI_INTERRUPT: 0x%04x", GPIO_Pin);
	AGN_LOG_DEBUG(str);
*/
	if (GPIO_Pin & RANGE1_ECHO_RS_Pin) {
		AGN_RANGE_RISING(1);
	}
	if (GPIO_Pin & RANGE1_ECHO_FL_Pin) {
		AGN_RANGE_FALLING(1);
	}


	if (GPIO_Pin & RANGE2_ECHO_RS_Pin) {
		AGN_RANGE_RISING(2);
	}
	if (GPIO_Pin & RANGE2_ECHO_FL_Pin) {
		AGN_RANGE_FALLING(2);
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin, 0); // LD4 (Green) is working indicator
  HAL_GPIO_WritePin(GPIOD, LD5_Pin, 0); // LD5 (Red) is error indicator
  HAL_GPIO_WritePin(GPIOD, LD6_Pin, 1); // LD6 (Blue)
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_DCMI_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_DAC_Init();
  MX_TIM10_Init();

  /* USER CODE BEGIN 2 */

  resetErrno();

  // Initialize Logger Huart
  AGN_LOG_INITIALIZE(&huart2);
  AGN_GATEWAY_INITIALIZE(&huart3);
  AGN_BUZZER_INITIALIZE(&hdac);
  AGN_RANGE_INITIALIZE();

  // Start Timer Interrupts
  // TODO Check for errors at this stage
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim10);

  // Start Digital-Analog Converter
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

  // Set Beep Sound
  AGN_BUZZER_SET_WAVEFORM(AGN_WAVEFORM_SQ);
  AGN_BUZZER_SET_FREQ(AGN_TONE_BB4 * 2);

  // Calibrate Rangefinders
  AGN_RANGE_SET_A(10, 1);
  AGN_RANGE_SET_A(10, 2);

  struct AGN_PACKET packet;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //const char * d = "hello\r\n";
	  //HAL_UART_Transmit(&huart2, d, strlen((char * )d), 1000);

	  if (getErrno() == AGN_ERRNO_GATEWAY_RECV_TIMEOUT) {
		  uint8_t dummy_bytes[AGN_PACKET_SIZE];
		  int connection = AGN_GATEWAY_RESYNC(dummy_bytes, AGN_PACKET_SIZE);
		  if (connection == 0) {
			  resetErrno();
		  }
	  }

	  packet.magic = 0xA0A0;

	  char mstr[200];
	  sprintf(mstr, "Before Trigger: %lu", agnProfile());
	  AGN_LOG_TRACE(mstr);
	  uint32_t _depth_1_sum = 0, _depth_2_sum = 0;
	  for (int i = 0; i < 8; i++) {
		  AGN_RANGE_TRIGGER(1);
		  _depth_1_sum += AGN_RANGE_GET(1);
		  AGN_RANGE_TRIGGER(2);
		  _depth_2_sum += AGN_RANGE_GET(2);
		  HAL_Delay(60);
	  }
	  packet.depth1 = _depth_1_sum / 8;
	  packet.depth2 = _depth_2_sum / 8;

	  uint8_t isDetectThief = (packet.depth1 + packet.depth2) < 540000;

	  sprintf(mstr, "After Trigger: %lu", agnProfile());
	  AGN_LOG_TRACE(mstr);

	  packet.status &= 0xF000;
	  packet.status |= (getErrno() & 0xFF);
	  packet.status |= (AGN_RANGE_IS_CONNECTED(2) << 8);
	  packet.status |= (AGN_RANGE_IS_CONNECTED(1) << 9);
	  packet.status |= (1 << 10);
	  packet.detection = 0x00;
	  packet.detection |= (packet.depth1 < 270000) << 3;
	  packet.detection |= (packet.depth2 < 270000) << 2;
	  packet.hex1  = 0x00 | (isDetectThief & 0x01);
	  packet.hex2  = 0x3;

	  sprintf(mstr, "After Get: %lu", agnProfile());
	  AGN_LOG_TRACE(mstr);
	  AGN_GATEWAY_SEND_PACKET(&packet);

	  sprintf(mstr, "After Send: %lu", agnProfile());
	  AGN_LOG_TRACE(mstr);

	  char rstr[200];
	  sprintf(rstr, "SENT Packet: mg=0x%04x, dp1=%lu, dp2=%lu, hx1=0x%01x, hx2=0x%01x, ic1=%d",
			  packet.magic, packet.depth1, packet.depth2, packet.hex1, packet.hex2, AGN_RANGE_IS_CONNECTED(1));
	  AGN_LOG_DEBUG(rstr);


	  AGN_GATEWAY_RECEIVE_PACKET(&packet);
	  sprintf(mstr, "After Receive: %lu", agnProfile());
	  AGN_LOG_TRACE(mstr);

	  char qstr[200];
	  sprintf(qstr, "RECV Packet: mg=0x%04x, dp1=%lu, dp2=%lu, hx1=0x%01x, hx2 = 0x%01x", packet.magic, packet.depth1, packet.depth2, packet.hex1, packet.hex2);
	  AGN_LOG_DEBUG(qstr);

	  // Handle Received Commands

	  if ((packet.mode == 0x01)) {
		  // On command 'ON'
		  AGN_LOG_DEBUG("SET ON");
		  AGN_BUZZER_SET_SWTICH(AGN_BUZZER_ON);
	  } else if ((packet.mode  == 0x00)) {
		  // On command 'OFF'
		  AGN_LOG_DEBUG("SET OFF");
		  AGN_BUZZER_SET_SWTICH(AGN_BUZZER_OFF);
	  } else if ((packet.mode == 0x02)) {
		  AGN_LOG_DEBUG("ENABLE");
		  if (isDetectThief) {
			  AGN_BUZZER_SET_SWTICH(AGN_BUZZER_ON);
 		  } else {
 			  AGN_BUZZER_SET_SWTICH(AGN_BUZZER_OFF);
 		  }
	  } else if ((packet.mode == 0x03)) {
		  AGN_LOG_DEBUG("DISABLE");
		  AGN_BUZZER_SET_SWTICH(AGN_BUZZER_OFF);
	  }

	  // Print Range Information
	  //char str[100];
	  //sprintf(str, "AGN_RANGE = %lu", AGN_RANGE_GET());
	  //AGN_LOG_DEBUG(str);

	  // Delay between measurements needs to be > 60ms
	  HAL_Delay(100);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 150;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DCMI init function */
static void MX_DCMI_Init(void)
{

  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 146;
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

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 99;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 75;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7500;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 4000;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 15000;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 50000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
     PC3   ------> I2S2_SD
     PA8   ------> RCC_MCO_1
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|GWHB_OUT_Pin|BUZZER_IO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin|RANGE1_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RANGE2_TRIG_GPIO_Port, RANGE2_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin GWHB_OUT_Pin BUZZER_IO_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|GWHB_OUT_Pin|BUZZER_IO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin RANGE2_TRIG_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|RANGE2_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GWHB_IN_Pin */
  GPIO_InitStruct.Pin = GWHB_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GWHB_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin RANGE1_TRIG_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin|RANGE1_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RANGE2_ECHO_FL_Pin */
  GPIO_InitStruct.Pin = RANGE2_ECHO_FL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RANGE2_ECHO_FL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RANGE2_ECHO_RS_Pin */
  GPIO_InitStruct.Pin = RANGE2_ECHO_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RANGE2_ECHO_RS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RANGE1_ECHO_RS_Pin */
  GPIO_InitStruct.Pin = RANGE1_ECHO_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RANGE1_ECHO_RS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RANGE1_ECHO_FL_Pin */
  GPIO_InitStruct.Pin = RANGE1_ECHO_FL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RANGE1_ECHO_FL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
  if (getErrno() == 0) {
	  setErrno(AGN_ERRNO_UNKNOWN);
  }
  HAL_GPIO_WritePin(GPIOD, LD6_Pin, 1);
  char message[1000];
  sprintf(message, "Unknown Error occurred at (%s:%d). ERRNO = %d.", file, line, getErrno());
  AGN_LOG_FATAL(message);
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
