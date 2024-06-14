/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wizchip_conf.h"
#include "socket.h"
#include "w5500_spi.h"
#include "ethernet.h"
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

W5500_HW_t w5500_hw;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t curr_bat = 0;
uint32_t curr_up = 0;
uint32_t curr_down = 0;
uint32_t volt_bat = 0;
uint8_t Ac_active = 0;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */
uint16_t adc_result[2];
uint8_t dato[5];
uint8_t dato2[5];
uint8_t re[50];
uint8_t re2[50];
GPIO_PinState sw;    // PB9
uint8_t eth = 72; // H
uint16_t offset_address;
uint8_t BSB;
uint8_t RWB;
uint8_t OM;
uint8_t DATA[240];
uint8_t control_phase;
uint8_t buffer[243] = { 0 };
uint8_t buffer_t[3];
uint8_t *p;
uint8_t buffer2[3000];

uint8_t sn[4];
uint8_t buffer3[3000];
uint8_t *rt;
GPIO_PinState NSHD;	 // PC0
GPIO_PinState S485_N232; // PC15
GPIO_PinState DE485_F232; // PC13
GPIO_PinState NRE485; // PA12
GPIO_PinState NTE485; // PA1
GPIO_PinState LB; // PA5

uint8_t c = 0;
uint16_t RxXferdif = 0;
uint8_t set1 = 0;
uint8_t set2 = 0;
uint8_t time1, time2, timeNow, timeDiff;

uint8_t gar[] = { 192, 168, 60, 1 };
uint8_t sub_r[] = { 255, 255, 255, 0 };
uint8_t shar[] = { 0x00, 0x08, 0xDC, 0x01, 0x02, 0x03 };
uint8_t sipr[] = { 192, 168, 60, 104 };
uint8_t mode = 0x012;
int8_t socket_mode = 0x01; //TCP
uint8_t S_MR = 0x01;
uint8_t S_CR_open = 0x01;
uint8_t S_CR_discon = 0x08;
uint8_t S_CR_con = 0x04;
uint8_t S_CR_recv = 0x40;
uint8_t S_SR = 0x13;
uint8_t S_PORT[2] = { 0x0B, 0xB8 }; //PUERTO 3000
uint8_t S_DPORT[2] = { 0x03, 0xE8 }; // PUERTO 1000
uint8_t S_MMS[2] = { 0x05, 0xB4 };
uint8_t S_RXBUF_SIZE = 0x01; // 1KB
uint8_t S_TXBUF_SIZE = 0x01; // 1KB
uint8_t S_CR_listen = 0x02;
uint8_t _PHYCFGR_RST = 0x1D;
uint8_t _PHYCFGR_NRST = 0X9D;
uint8_t read_reg = 1;
uint8_t S_DHAR[6] = { 0x08, 0xDC, 0x00, 0x01, 0x02, 0x0A };
uint8_t S_TTL = 0x40;

uint8_t s_MR; //0
uint8_t s_CR; //1
uint8_t s_IR; //2
uint8_t s_SR; //3
uint8_t s_PORT[2]; //4
uint8_t s_DHAR[6]; //6
uint8_t s_DIPR[4]; //C
uint8_t s_DPORT[2]; //10
uint8_t s_MSS[2]; //12
uint8_t s_TOS; //15
uint8_t s_TTL; //16
uint8_t s_RXBUF_SIZE; //1E
uint8_t s_TXBUF_SIZE; //1F
uint8_t s_TX_FS[2]; //20
uint8_t s_TX_RD[2]; //22
uint8_t s_TX_WR[2]; //24

uint8_t s_RX_WR[2]; //2A
uint8_t s_IMR; //2C
uint8_t s_FRAG[2]; //2D
uint8_t s_KPALVTR; //2F

uint8_t read_IR = 0;

uint8_t socket_status = 0; // 1 CON, 2 DISCN, 3 RECV, 4 TIMEOUT, 5 SENDOK
uint8_t tx_data[4] = { 84, 69, 83, 84 }; // TEST
uint8_t send_socket0 = 0x20;
uint16_t point = 0;
uint16_t q = 0;

uint8_t s_send_ok = 0; //0x10
uint8_t s_timeout = 0; //0x08
uint8_t s_recv = 0; //0x04
uint8_t s_discon = 0; //0x02
uint8_t s_con = 0; //0x01
uint16_t len_rx = 0;
uint16_t offset_read = 0;
uint8_t data_transmition[3000];
uint8_t eth_bufRX[1024];
uint8_t eth_lenRX;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

uint32_t Read_ADC_Channel(uint32_t channel) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t value = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	return value;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	uint8_t ip[]={0,0,0,0};
	init_w5500_hw(&w5500_hw, &hspi1, W5500_NSS_GPIO_Port, W5500_NSS_Pin, W5500_NRST_GPIO_Port, W5500_NRST_Pin);
	eth_read_reg(0,0x0F,ip,4);
	common_register_block(buffer, 0x0F, sipr, sizeof(sipr));
	eth_read_reg(0,0x0F,ip,4);


	//PHYSCIS configuration of the chip//
	eth_write_reg(COMMON_REG_OFFSET, PHYCFGR_RST_OFFSET,
			(uint8_t*) &_PHYCFGR_RST, sizeof(_PHYCFGR_RST));
	HAL_Delay(500);
	eth_write_reg(COMMON_REG_OFFSET, PHYCFGR_RST_OFFSET,
			(uint8_t*) &_PHYCFGR_NRST, sizeof(_PHYCFGR_NRST));
	HAL_Delay(200);



	/*Configuration of common registers*/
	//common_reg_config(buffer, mode, gar, sub_r, shar, sipr);

	common_register_block(buffer, 0x00, (uint8_t*) &mode, sizeof(mode));
	common_register_block(buffer, 0x01, gar, sizeof(gar));
	common_register_block(buffer, 0x05, sub_r, sizeof(sub_r));
	common_register_block(buffer, 0x09, shar, sizeof(shar));
	common_register_block(buffer, 0x0F, sipr, sizeof(sipr));

	socket_reg_config(buffer, S_MR, S_PORT, S_DHAR, S_DPORT, S_MMS, S_TTL,
			S_RXBUF_SIZE, S_TXBUF_SIZE, S_CR_open, S_CR_listen);

	memset(ip,0,4);
	eth_read_reg(0,0x0F,ip,4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		/*DHCP validation */
	    if (DHCP_run() == DHCP_RET_UPDATE) {
	       //Update network settings after getting IP
	       wizchip_getnetinfo(&net_info);
	       printf("IP Address assigned by DHCP: %d.%d.%d.%d\n", net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3]);
	    }
		/* Read analog values */
		curr_bat = Read_ADC_Channel(ADC_CHANNEL_0);  // PA0
		curr_up = Read_ADC_Channel(ADC_CHANNEL_1);   // PA1
		curr_down = Read_ADC_Channel(ADC_CHANNEL_3); // PA3
		volt_bat = Read_ADC_Channel(ADC_CHANNEL_4);  // PA4

		/* Read digital value */
		Ac_active = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2); // PA2

		/* Add your code to process the adcValue */
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
		HAL_Delay(100);

		// Send data via Modbus TCP/IP
		send_modbus_tcp_ip(0, 1, 1, 0, curr_bat, curr_up, curr_down, volt_bat, Ac_active);

		// LEER REGISTRO IR
		eth_read_reg(socket_0_register, S_IR_OFFSET, &s_IR, sizeof(s_IR));

		if (s_IR & Sn_IR_MASK) {
			uint8_t ir_reset;
			if (s_IR & Sn_CONNECT) {  // SOCK_ESTABLISHED
				socket_cmd_cfg(socket_0_register, S_CR_CONNECT);
				ir_reset = Sn_CONNECT;
			}
			if (s_IR & Sn_DISCONNECT) { //FIN/ACK
				socket_cmd_cfg(socket_0_register, S_CR_DISCONECT);
				socket_cmd_cfg(socket_0_register, S_CR_OPEN);
				socket_cmd_cfg(socket_0_register, S_CR_LISTEN);
				ir_reset = Sn_DISCONNECT;
			}

			if ((s_IR & Sn_RECEIVE)) {
				/*readDataFromEthernet*/
				eth_lenRX = read_socket_n_rx_buffer(socket_0_register,
						eth_bufRX);
				socket_cmd_cfg(socket_0_register, S_CR_RECV);
				ir_reset = Sn_RECEIVE;
			}
			eth_write_reg(socket_0_register, S_IR_OFFSET, (uint8_t*) &ir_reset,
					sizeof(ir_reset));

		}

	   // Reading the status of the socket
	   eth_read_reg(socket_0_register, S_SR_OFFSET, &s_SR, sizeof(s_SR));

       /* USER CODE END WHILE */

       /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, W5500_NSS_Pin|GPIO_PIN_4|USART1_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, W5500_NRST_Pin|NINT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : AC_ACTIV_Pin */
  GPIO_InitStruct.Pin = AC_ACTIV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AC_ACTIV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : W5500_NSS_Pin PB4 USART1_CTRL_Pin */
  GPIO_InitStruct.Pin = W5500_NSS_Pin|GPIO_PIN_4|USART1_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : W5500_NRST_Pin NINT_Pin */
  GPIO_InitStruct.Pin = W5500_NRST_Pin|NINT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
