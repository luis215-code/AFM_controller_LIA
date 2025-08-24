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
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "controller.h"
#include "commands.h"
//#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
void fu(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */


volatile uint8_t sweep_done = 0;
volatile uint8_t sweeping = 0;
extern USBD_HandleTypeDef hUsbDeviceFS;
volatile bool command_pending = false;
volatile bool start_ADC = false;


// How many wave cycles per DAC buffer

//#define DDS_BUFFER_SIZE 30


// --- Constants ---
float X_buffer[DDS_BUFFER_SIZE];
float Y_buffer[DDS_BUFFER_SIZE];
uint16_t sine_lut[LUT_SIZE];
uint16_t dac_buffer[DDS_BUFFER_SIZE];
uint16_t adc_buffer[DDS_BUFFER_SIZE];
uint16_t LIA_phi_acc_idx_buff[DDS_BUFFER_SIZE];
uint16_t X_Y_buffer_index = 0;
volatile bool usb_ready_to_transmit = false;
volatile bool sweep_step_request = false;
float X = 0.0f, Y = 0.0f;
float alpha = 0.001f;  // EMA filter coefficient
volatile float dc_offset = 0.0f;


uint8_t USB_recieve_buffer[USB_BUFFER_LENGTH];
char *connected_message = "AFM controller connected\r\n";



// Precompute sine and cosine tables
float sin_lut_lockin[LUT_SIZE];
float cos_lut_lockin[LUT_SIZE];


volatile uint32_t phase_acc = 0;
uint32_t delta_phase = 0;



uint16_t sine_lut_fixed[LUT_SIZE_FIXED];





float current_frequency = 32140.0f;  // Desired output frequency





float start_freq = 32000;
float end_freq = 32300;
float freq_spacing = 0.1;
float amplitude = 0.5;

void set_dds_frequency(float freq_hz)
{
	current_frequency = freq_hz;
	// Calculate phase increment for desired frequency
	delta_phase = (uint32_t)((current_frequency * (float)(1ULL << 32)) / DAC_SAMPLE_RATE);
}


void generate_sine_lut_fixed(float amplitude)
{
    for (int i = 0; i < LUT_SIZE_FIXED; ++i) {
        float angle = 2.0f * M_PI * i / LUT_SIZE_FIXED;
        float sine = sinf(angle); // -1 to +1
        float scaled = ((sine * amplitude * 0.5f) + 0.5f) * DAC_MAX;
        sine_lut_fixed[i] = (uint16_t)(scaled + 0.5f);
    }
}


// --- Generate sine LUT ---
void init_sine_lut(float amplitude)
{
    if (amplitude < 0.0f) amplitude = 0.0f;
    if (amplitude > 1.0f) amplitude = 1.0f;

    for (int i = 0; i < LUT_SIZE; i++) {
        float angle = ((float)i / LUT_SIZE) * 2.0f * M_PI;
        float sine_val = sinf(angle);
        float cos_val = cosf(angle);
        float scaled = ((sine_val * amplitude * 0.5f) + 0.5f) * DAC_MAX;

        if (scaled < 0) scaled = 0;
        if (scaled > DAC_MAX) scaled = DAC_MAX;

        sin_lut_lockin[i] = sine_val;
        cos_lut_lockin[i] = cos_val;
        sine_lut[i] = (uint16_t)scaled;
    }
}

// --- Fill half of the DMA buffer ---
void fill_buffer(int start, int end)
{
    for (int i = start; i < end; i++) {
        phase_acc += delta_phase;
        uint16_t index = phase_acc >> (32 - LUT_BITS); // Top 10 bits for LUT index
        dac_buffer[i] = sine_lut[index];
        LIA_phi_acc_idx_buff[i] = index;
    }
}

void calculate_LIA_quads(int start, int end)
{
	float sum = 0.0f;
	float sum_X = 0.0f;
	float sum_Y = 0.0f;

	for (int i = 0; i < DDS_BUFFER_SIZE; i++) {
	        sum += (float)adc_buffer[i];
	    }
	    dc_offset = sum / DDS_BUFFER_SIZE;
    for (int i = start; i < end; i++) {
//    	X_buffer[i] = (adc_buffer[i]-dc_offset)*sin_lut_lockin[LIA_phi_acc_idx_buff[i]];
//    	Y_buffer[i] = (adc_buffer[i]-dc_offset)*cos_lut_lockin[LIA_phi_acc_idx_buff[i]];
    	sum_X += (adc_buffer[i]-dc_offset)*sin_lut_lockin[LIA_phi_acc_idx_buff[i]];
    	sum_Y += (adc_buffer[i]-dc_offset)*cos_lut_lockin[LIA_phi_acc_idx_buff[i]];
    }
    X = sum_X/(float)(end-start);
    Y = sum_Y/(float)(end-start);

}


/* Provide the wrapper so old call sites keep working */
void handle_usb_command(const char* cmd)
{
    if (!cmd) return;
    Commands_ProcessLine(cmd);
}




void Send_XY_Over_USB_Raw(float x, float y)
{
    // Only send if USB is ready
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        return;
    }

//    uint8_t buffer[8];  // 4 bytes per float
//
//    // Copy the raw bytes of each float into the buffer
//    memcpy(&buffer[0], &x, sizeof(float));
//    memcpy(&buffer[4], &y, sizeof(float));
//
//    // Transmit the raw buffer over USB
//    CDC_Transmit_FS(buffer, sizeof(buffer));
//    printf("%u, %u\r\n", (uint16_t)X, (uint16_t)Y);
    printf("%.1f, %.1f\r\n", (double)x, (double)y);
}




// fixed LUT version
//void fill_buffer(int start, int end)
//{
//    for (int i = start; i < end; i++) {
//        dac_buffer[i] = sine_lut_fixed[i];
//    }
//}


// --- DMA callback handlers ---

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{

    fill_buffer(0, DDS_BUFFER_SIZE / 2);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
    fill_buffer(DDS_BUFFER_SIZE / 2, DDS_BUFFER_SIZE);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

}

//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
//{
//    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
////    calculate_LIA_quads(0, DDS_BUFFER_SIZE/2);
//}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//	calculate_LIA_quads(0, DDS_BUFFER_SIZE);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
    usb_ready_to_transmit = true;
    sweep_step_request = true;
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


    // Example: send X,Y periodically if ready
//    if (usb_ready_to_transmit) {
//        Send_XY_Over_USB_Raw(X, Y);
//        usb_ready_to_transmit = false;
//    }
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  g_controller.init(&g_controller);
  //HAL_TIM_Base_Start(&htim2);          // ensure TIM2 is running so TRGO (update) happens

  //HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)sine_lut, SINE_RESOLUTION, DAC_ALIGN_12B_R);
  //HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  // Initialize sine LUT at 85% amplitude
  init_sine_lut(amplitude);

  // Set DDS frequency
  set_dds_frequency(current_frequency);

  // Fill the entire buffer initially
  fill_buffer(0, DDS_BUFFER_SIZE);

  // starting DAC and ADC DMAs

  HAL_TIM_Base_Start(&htim6);

  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dac_buffer, DDS_BUFFER_SIZE, DAC_ALIGN_12B_R);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, DDS_BUFFER_SIZE);



//  generate_sine_lut_fixed(1.0f);
  //HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)sine_lut_fixed, LUT_SIZE_FIXED, DAC_ALIGN_12B_R);


//  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
//  for (int i = 0; i<10; i++){
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);  // Or any LED pin you have
//      HAL_Delay(500);  // small delay to avoid busy-waiting
//  }

  // Check if USB connection is working message
  // Wait for host to open the COM port (USB configured)
  while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
      HAL_Delay(10);  // small delay to avoid busy-waiting
  }
  HAL_Delay(50);  // small delay to avoid busy-waiting
  // Now it's safe to transmit
  CDC_Transmit_FS((uint8_t *)connected_message, strlen(connected_message));


  //HAL_TIM_Base_Start_IT(&htim2);  // Enable TIM2 Update interrupt
  // Start the timer
  //HAL_TIM_Base_Start(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (usb_ready_to_transmit && g_controller.quad_stream_enabled)
	  {
		  calculate_LIA_quads(0, DDS_BUFFER_SIZE);
		  Send_XY_Over_USB_Raw(X, Y);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
//  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 240-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 240-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA1, SYSCFG_SWITCH_PA1_CLOSE);

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

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
