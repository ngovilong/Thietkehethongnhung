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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "i2c-lcd.h"

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

/* USER CODE BEGIN PV */

volatile float frequency = 0;
volatile uint32_t last_capture_tick = 0;
const uint32_t timeout_ms = 500;
volatile uint32_t pulse_count = 0;
double Temperature, VTmpSens, VrefeInt;
#define VREFRINT 1.20
#define ADCMAX 4095.0
#define AVG_SLOPE 0.0043
#define V25 1.43
extern UART_HandleTypeDef huart2;
#define CYCLE_MS 6000
float saved_frequency = 0;
float saved_temperature = 0;
uint16_t AdcRaw[2];
uint8_t AdcConvCmplt = 0;
uint32_t freq_tick_start = 0;
uint8_t freq_started = 0;
#define FRAME_MS 250
#define TOTAL_FRAMES 8
volatile uint32_t ic_val1 = 0, ic_val2 = 0;
volatile uint8_t is_first = 0;
volatile float measured_freq = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        if (!is_first) {
            ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            is_first = 1;
        } else {
            ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            uint32_t diff = (ic_val2 > ic_val1) ? (ic_val2 - ic_val1) : (0xFFFF - ic_val1 + ic_val2);
            measured_freq = 1e6 / diff; // 1 MHz timer clock
            is_first = 0;
        }
    }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        AdcConvCmplt = 1;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	pulse_count = 0;
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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AdcRaw, 2);
  uint32_t frame_start = HAL_GetTick();
  uint8_t frame_index = 0;
  lcd_init();
  lcd_clear();
  uint8_t uart_sent = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  char msg[64];
	  if (freq_started && HAL_GetTick() - freq_tick_start >= 1000) {
	      saved_frequency = pulse_count;
	      pulse_count = 0;
	      freq_started = 0;
	  }
	      // Frame 0, 4, 6, 2: T4
//	      if (frame_index == 0 || frame_index == 2 || frame_index == 4 || frame_index == 6) {
//	          lcd_clear();
//	          lcd_put_cur(0, 0);
//	          sprintf(msg, "F:%.0fHz", saved_frequency);
//	          lcd_send_string(msg);
//	          lcd_put_cur(1, 0);
//	          sprintf(msg, "T:%.1fC", saved_temperature);
//	          lcd_send_string(msg);
//	      }

	      // Frame 1, 5: T1 - đo tần số
	      if (frame_index == 1 || frame_index == 5) {
	    	  if (frame_index == 1 && !freq_started) {
	    	      freq_tick_start = HAL_GetTick();
	    	      freq_started = 1;
	    	  } else if (HAL_GetTick() - freq_tick_start >= 1000) {
	    	      saved_frequency = measured_freq;
	    	      freq_started = 0;
	    	  }

	      }

	      // Frame 2, 6: T2 - đo nhiệt độ
	      if (frame_index == 2 || frame_index == 6) {
	          if (AdcConvCmplt) {
	              VrefeInt = (VREFRINT * ADCMAX) / AdcRaw[0];
	              VTmpSens = (VrefeInt * AdcRaw[1]) / ADCMAX;
	              saved_temperature = (V25 - VTmpSens) / AVG_SLOPE + 25.0;
	              AdcConvCmplt = 0;
	          }
	      }

	      // Frame 7: T3 - gửi UART
	      if (frame_index == 7 && uart_sent == 0) {
	          sprintf(msg, "Tan so = %.0f Hz\r\nNhiet do = %.1f C\r\n", saved_frequency, saved_temperature);
	          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	          uart_sent = 1;
	      }


	      // Chờ hết frame
	      while (HAL_GetTick() - frame_start < FRAME_MS);
	      frame_start += FRAME_MS;
	      frame_index = (frame_index + 1) % TOTAL_FRAMES;
	      if (frame_index == 6) uart_sent = 0;


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
