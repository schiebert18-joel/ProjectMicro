/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "UNERprotocol.h"
#include "Utilities.h"
#include "ADC.h"
#include "Engines.h"
#include "MPU6050.h"
#include "Fonts.h"
#include "ssd1306_oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_CHANNELS_ADC 9
#define RINGBUFFER_Tx 	256
#define RINGBUFFER_Rx 	256

#define IS250US 		myFlags.individualFlags.bit1
#define ADCready		myFlags.individualFlags.bit2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
_bFlags myFlags;
_work w;
_sDato datosComSerie;
_sIrSensor sensorIR;
_sEng motorL,motorR;
_eEngState estado; /*!< Creo que lo puedo borrar*/
_sMPUxyz DatosMPU;

uint16_t adcBuffer[NUM_CHANNELS_ADC];
uint8_t commBufferRx[RINGBUFFER_Tx];
uint8_t commBufferTx[RINGBUFFER_Rx];
//uint8_t asd = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);

/*		MOTORES		*/
void MotorL_SetPWM(uint16_t dCycle);
void MotorR_SetPWM(uint16_t dCycle);
void MotorL_SetPIN(_eEngState estado);
void MotorR_SetPIN(_eEngState estado);

/*		  MPU		*/
void MPU6050_Init(I2C_HandleTypeDef *hi2c);

/* 		  I2C		*/
//uint8_t I2C_DMA_Transmit(uint16_t Dev_Address,uint16_t reg,uint8_t *p_Data, uint16_t _Size);
//uint8_t I2C_1_Abstract_Master_Transmit_Blocking(uint16_t Dev_Address, uint8_t *p_Data, uint16_t _Size, uint32_t _Timeout);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim -> Instance == TIM1){
		IS250US = TRUE;
	}
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&sensorIR.bufferADCvalue, 9);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//lamar a la funcion para hacer el filtrado de datos
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){

	DatosMPU.x.accel = (DatosMPU.bufData[0] << 8) | DatosMPU.bufData[1];
	DatosMPU.y.accel = (DatosMPU.bufData[2] << 8) | DatosMPU.bufData[3];
	DatosMPU.z.accel = (DatosMPU.bufData[4] << 8) | DatosMPU.bufData[5];

//	DatosMPU.temperature = (bufData[6] << 8) | bufData[7];

	DatosMPU.x.gyro = (DatosMPU.bufData[8] << 8) | DatosMPU.bufData[9];
	DatosMPU.y.gyro = (DatosMPU.bufData[10] << 8) | DatosMPU.bufData[11];
	DatosMPU.z.gyro = (DatosMPU.bufData[12] << 8) | DatosMPU.bufData[13];
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t time250us = 0, time10ms = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  CDC_Attach_Rx(&CommDatafromUSB);
  CommInitProtocol(&datosComSerie, &CommDecodeData, commBufferRx, commBufferTx);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  en_InitENG(&motorL, &MotorL_SetPWM, &MotorL_SetPIN, htim3.Instance->ARR); /*!< asigno a cada motor una direccio de memoria para manejarlo desde la lib */
  en_InitENG(&motorR, &MotorR_SetPWM, &MotorR_SetPIN, htim3.Instance->ARR); /*!< En donde, (htim3.Instance->ARR) es el valor maximo de PWM */

  MPU6050_Init(&hi2c1);

  SSD1306_Init(); 		/*! Inicializamos el OLED */
  HAL_Delay(100);
  SSD1306_Fill(BLACK);	/*! Limpiamos pantalla (rellenamos de negro) */
  SSD1306_GotoXY(0, 0); /*! Posicionamos cursor (esquina superior izquierda) */
  SSD1306_Puts("tkm Lu", &Font_7x10, WHITE); /*!  Escribimos un texto 	*/
  SSD1306_UpdateScreen(); /*! Enviamos el buffer a la pantalla */

//  Display_Set_I2C_Master_Transmit(&I2C_DMA_Transmit, &I2C_1_Abstract_Master_Transmit_Blocking);
//  SSD1306_Init();

  IS250US = FALSE;
  ADCready = TRUE;
  datosComSerie.Rx.indexRead = 0;
  datosComSerie.Rx.indexWrite =0;
  myFlags.allFlags = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  CommComunicationsTask(&datosComSerie);

	  if(IS250US){
		  time250us++;
		  IS250US =! IS250US;
		  if(time250us >= 40){
			  time10ms++;
			  time250us = 0;
			  if(time10ms == 10){
				  HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_REG, MemAddSize, (uint8_t*)&DatosMPU.bufData, 14);
				  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				  time10ms = 0;
			  }
		  }
	  }

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 9;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 23999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 59999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OutEngB_1_Pin|OutEngB_2_Pin|OutEngA_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OutEngA_1_GPIO_Port, OutEngA_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OutEngB_1_Pin OutEngB_2_Pin OutEngA_2_Pin */
  GPIO_InitStruct.Pin = OutEngB_1_Pin|OutEngB_2_Pin|OutEngA_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OutEngA_1_Pin */
  GPIO_InitStruct.Pin = OutEngA_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OutEngA_1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief: 	recibo la informacion enviada por puerto USB (lo enviado por QT), y guardo los bytes recibidos
 *  		en el buffer circular Rx.buffercomm[] de la estructura datosComSerie. Es decir:
 * 			UNER = 55 4E 45 52 // Nbytes= 02 // ':' = 3A // Alive= F0 // 0xC4 = checksum
 *
 * @param: *buf, el buffer que posee la informacion para recibirla en el buffer de recepcion
 * @param: length, largo del buffer
 */
void CommDatafromUSB(uint8_t *buf, uint16_t length){

  uint16_t i;

  for (i = 0; i < length; i++) {
	  datosComSerie.Rx.buffercomm[datosComSerie.Rx.indexWrite] = buf[i];
	  datosComSerie.Rx.indexWrite++;
  }

}

/**
 * @brief: si llegó informacion (CommDatafromUSB) entonces llamo a decodeheader para el analisis del protocolo "UNER"
 *
 */
void CommComunicationsTask(_sDato *datosCom){
	if(datosCom->Rx.indexRead!=datosCom->Rx.indexWrite ){ /*!< si Recepcion write =! Recepcion read => buffer lleno */
		CommDecodeHeader(datosCom);
		datosCom->Rx.indexRead=datosCom->Rx.indexWrite;
	}

	if(datosCom->Tx.indexRead!=datosCom->Tx.indexWrite ){
		if(datosCom->Tx.indexWrite > datosCom->Tx.indexRead){
				datosCom->bytesTosend = datosCom->Tx.indexWrite - datosCom->Tx.indexRead;
		}else{
			datosCom->bytesTosend =  sizeof(datosCom->Rx.buffercomm) - datosCom->Tx.indexRead;
		}
		if(CDC_Transmit_FS(&datosCom->Tx.buffercomm[datosCom->Tx.indexRead], datosCom->bytesTosend) == USBD_OK){ /*!< Paquete enviado hacia QT: 55 4E 45 52   01 	 3A     F0         0D          C8
							 																												    'U''N''E''R''Nbytes'':''ID:Alive''Payload: ACK''Cheksum'*/
			datosCom->Tx.indexRead += datosCom->bytesTosend;
		}
	}
}

/**
 * @brief Si el protocolo fue aceptado (CommDecodeHeader), entonces preparo la respuesta segun el ID recibido.
 *
 */
void CommDecodeData(_sDato *datosComLib){
    uint8_t bufAux[30], indiceAux=0,bytes=0;
    uint32_t velL = 0, velR = 0;

    switch (datosComLib->Rx.buffercomm[datosComLib->indexStart+2])/*!< ID EN LA POSICION 2, porque es donde se adjunta el byte que te dice "ALIVE, FIRMWARE, ETC" */
    {

	case ALIVE:
		bufAux[indiceAux++] = ALIVE;   	/*!< ID de respuesta */
		bufAux[indiceAux++] = 0x0D;    	/*!< Respuesta: ACK */
		bytes = 0x03;        			/*!< NBYTES = 3 (ID + Dato + Checksum) */
	break;

    case FIRMWARE:
		bufAux[indiceAux++]=FIRMWARE;
		bytes=0x02;
    break;

    case IR:
        bufAux[indiceAux++] = IR;
        w.u16[0] = adcBuffer[0];
        bufAux[indiceAux++] = w.u8[0];  // LSB
        bufAux[indiceAux++] = w.u8[1];  // MSB

        w.u16[0] = adcBuffer[1];
        bufAux[indiceAux++] = w.u8[0];  // LSB
        bufAux[indiceAux++] = w.u8[1];  // MSB

        w.u16[0] = adcBuffer[2];
        bufAux[indiceAux++] = w.u8[0];  // LSB
        bufAux[indiceAux++] = w.u8[1];  // MSB

        w.u16[0] = adcBuffer[3];
        bufAux[indiceAux++] = w.u8[0];  // LSB
        bufAux[indiceAux++] = w.u8[1];  // MSB

        w.u16[0] = adcBuffer[4];
        bufAux[indiceAux++] = w.u8[0];  // LSB
        bufAux[indiceAux++] = w.u8[1];  // MSB

        w.u16[0] = adcBuffer[5];
        bufAux[indiceAux++] = w.u8[0];  // LSB
        bufAux[indiceAux++] = w.u8[1];  // MSB

        w.u16[0] = adcBuffer[6];
        bufAux[indiceAux++] = w.u8[0];  // LSB
        bufAux[indiceAux++] = w.u8[1];  // MSB

        w.u16[0] = adcBuffer[7];
        bufAux[indiceAux++] = w.u8[0];  // LSB
        bufAux[indiceAux++] = w.u8[1];  // MSB

        bytes = 18; // 1 byte para ID, 2 bytes por canal + Cks

	break;

    case MOTOR:

        w.u8[0] = datosComLib->Rx.buffercomm[datosComLib->indexStart+3];
        w.u8[1] = datosComLib->Rx.buffercomm[datosComLib->indexStart+4];
        w.u8[2] = datosComLib->Rx.buffercomm[datosComLib->indexStart+5];
        w.u8[3] = datosComLib->Rx.buffercomm[datosComLib->indexStart+6];
        velL = w.i32;

        w.u8[0] = datosComLib->Rx.buffercomm[datosComLib->indexStart+7];
        w.u8[1] = datosComLib->Rx.buffercomm[datosComLib->indexStart+8];
        w.u8[2] = datosComLib->Rx.buffercomm[datosComLib->indexStart+9];
        w.u8[3] = datosComLib->Rx.buffercomm[datosComLib->indexStart+10];
        velR = w.i32;

        en_HandlerENG(&motorL, velL, 0);
        en_HandlerENG(&motorR, velR, 0);

//        bufAux[indiceAux++] = MOTOR;
//
//        w.i32 = velL;
//        bufAux[indiceAux++] = w.u8[0];
//        bufAux[indiceAux++] = w.u8[1];
//        bufAux[indiceAux++] = w.u8[2];
//        bufAux[indiceAux++] = w.u8[3];
//
//        w.i32 = velR;
//        bufAux[indiceAux++] = w.u8[0];
//        bufAux[indiceAux++] = w.u8[1];
//        bufAux[indiceAux++] = w.u8[2];
//        bufAux[indiceAux++] = w.u8[3];
//
//        bytes = 2 + 8; // ID + cks + velL + velR

	break;

    case MPUData:
    	bufAux[indiceAux++] = MPUData;
    	// Aceleracion
    	w.i16[0] = DatosMPU.x.accel;
    	bufAux[indiceAux++] = w.u8[0];
    	bufAux[indiceAux++] = w.u8[1];

    	w.i16[0] = DatosMPU.y.accel;
    	bufAux[indiceAux++] = w.u8[0];
    	bufAux[indiceAux++] = w.u8[1];

    	w.i16[0] = DatosMPU.z.accel;
    	bufAux[indiceAux++] = w.u8[0];
    	bufAux[indiceAux++] = w.u8[1];

    	// Giroscopio
    	w.i16[0] = DatosMPU.x.gyro;
    	bufAux[indiceAux++] = w.u8[0];
    	bufAux[indiceAux++] = w.u8[1];

    	w.i16[0] = DatosMPU.y.gyro;
    	bufAux[indiceAux++] = w.u8[0];
    	bufAux[indiceAux++] = w.u8[1];

    	w.i16[0] = DatosMPU.z.gyro;
    	bufAux[indiceAux++] = w.u8[0];
    	bufAux[indiceAux++] = w.u8[1];

    	bytes = 14;

    break;


    default:
        bufAux[indiceAux++]=0xFF;
        bytes=0x02;
    break;
    }

    CommSendInfo(datosComLib,bufAux,bytes);
}

/**
 * @brief Seteo los pines segun el estado de los motores (Adelante, atras, freno, libre)
 */
void MotorL_SetPIN(_eEngState estado){
	switch(motorL.estado){
		case BRAKE:
			HAL_GPIO_WritePin(OutEngA_1_GPIO_Port, OutEngA_1_Pin, 1);
			HAL_GPIO_WritePin(OutEngA_2_GPIO_Port, OutEngA_2_Pin, 1);
			break;

		case FRONT:
			HAL_GPIO_WritePin(OutEngA_1_GPIO_Port, OutEngA_1_Pin, 1);
			HAL_GPIO_WritePin(OutEngA_2_GPIO_Port, OutEngA_2_Pin, 0);
			break;

		case BACK:
			HAL_GPIO_WritePin(OutEngA_1_GPIO_Port, OutEngA_1_Pin, 0);
			HAL_GPIO_WritePin(OutEngA_2_GPIO_Port, OutEngA_2_Pin, 1);
			break;

		case FREE:
			HAL_GPIO_WritePin(OutEngA_1_GPIO_Port, OutEngA_1_Pin, 0);
			HAL_GPIO_WritePin(OutEngA_2_GPIO_Port, OutEngA_2_Pin, 0);
			break;
		default:
			break;
	}
}
void MotorR_SetPIN(_eEngState estado){
	switch(motorR.estado){
		case BRAKE:
			HAL_GPIO_WritePin(OutEngB_1_GPIO_Port, OutEngB_1_Pin, 1);
			HAL_GPIO_WritePin(OutEngB_2_GPIO_Port, OutEngB_2_Pin, 1);
			break;

		case FRONT:
			HAL_GPIO_WritePin(OutEngB_1_GPIO_Port, OutEngB_1_Pin, 1);
			HAL_GPIO_WritePin(OutEngB_2_GPIO_Port, OutEngB_2_Pin, 0);
			break;

		case BACK:
			HAL_GPIO_WritePin(OutEngB_1_GPIO_Port, OutEngB_1_Pin, 0);
			HAL_GPIO_WritePin(OutEngB_2_GPIO_Port, OutEngB_2_Pin, 1);
			break;

		case FREE:
			HAL_GPIO_WritePin(OutEngB_1_GPIO_Port, OutEngB_1_Pin, 0);
			HAL_GPIO_WritePin(OutEngB_2_GPIO_Port, OutEngB_2_Pin, 0);
			break;
		default:
			break;
	}
}

/**
 * @Brief Le doy la velocidad a los motores
 */
void MotorL_SetPWM(uint16_t dCycle){
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dCycle);
}
void MotorR_SetPWM(uint16_t dCycle){
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, dCycle);
}

/**
 * @brief Inicializo el MPU. Lo despierto y configuro el giroscopio y el acelerómetro mediante el tercer parametro (MemAddress).
 */
void MPU6050_Init(I2C_HandleTypeDef *hi2c){
    uint8_t data = 0;

    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I, MemAddSize, &data, 1, timeout_duration);
    if (data == WHO_AM_I_DEFAULT_VALUE){
        // Wake up MPU6050
        data = 0;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, POWER_MANAGEMENT_REG, MemAddSize, &data, 1, timeout_duration);

        // Set accelerometer range (+/- 2g = 0, 4g = 01, 8g = 10, 16g = 11)
        data = 0; /*!< 2g (default) */
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, MemAddSize, &data, 1, timeout_duration);

        // Set gyroscope range (+/- 250, 500, 1000, 2000 degree/s)
        data = 0; /*!< 250 degree/s (default) */
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, MemAddSize, &data, 1, timeout_duration);

        // Set Digital Low Pass Filter
		data = 4; /*! Frecuencia de muestreo: 1kHz / 1+4 = 200hZ -> Retardo de 8.5 ms */
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, DLPF_CONFIG_REG, MemAddSize, &data, 1, timeout_duration);
    }
}

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
  while (1)
  {
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
