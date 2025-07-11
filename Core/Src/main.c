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
uint32_t last_uart_tick = 0;
double Temperature, VTmpSens, VrefeInt;
#define VREFRINT 1.20
#define ADCMAX 4095.0
#define AVG_SLOPE 0.0043
#define V25 1.43
extern UART_HandleTypeDef huart2;
uint32_t last_debug_tick = 0;
uint32_t adc_duration = 0;
uint32_t uart_duration = 0;
uint32_t freq_duration = 0;
#define CYCLE_MS 6000
float saved_frequency = 0;
float saved_temperature = 0;
uint8_t lcd_sent = 0;
uint32_t last_freq_tick = 0;
uint8_t freq_measuring = 0;
uint32_t freq_count = 0;
uint16_t AdcRaw[2];
uint8_t AdcConvCmplt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        pulse_count++;
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
  HAL_TIM_Base_Start(&htim3);
  lcd_init();
  lcd_clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      uint32_t cycle_start = HAL_GetTick();
      pulse_count = 0;
      freq_measuring = 1;
      lcd_sent = 0;
      while (HAL_GetTick() - cycle_start < CYCLE_MS) {
          uint32_t now = HAL_GetTick();
          uint32_t elapsed = now - cycle_start;
          if (elapsed < 1000) {

          }
          else if (elapsed >= 1000 && freq_measuring) {
              saved_frequency = pulse_count;
              freq_measuring = 0;
          }
          else if (elapsed < 2000) {
              if (AdcConvCmplt) {
                  VrefeInt = (VREFRINT * ADCMAX) / (AdcRaw[0]);
                  VTmpSens = (VrefeInt * AdcRaw[1]) / (ADCMAX);
                  saved_temperature = (V25 - VTmpSens) / (AVG_SLOPE) + 25.0;
                  AdcConvCmplt = 0;
              }
          }
          else if (elapsed < 4000) {
              char msg[100];
              sprintf(msg, "Tan so = %.0f Hz\r\n\n", saved_frequency);
              HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
              sprintf(msg, "Nhiet do = %.1f C\r\n\n", saved_temperature);
              HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
              sprintf(msg, "Da gui len UART\r\n\n");
              HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
              while (HAL_GetTick() - cycle_start < 4000);
          }
          else if (elapsed < 4500 && !lcd_sent) {
              char line[17];
              char msg[100];
              lcd_clear();
              lcd_put_cur(0, 0);
              sprintf(line, "F:%.0fHz", saved_frequency);
              lcd_send_string(line);
              lcd_put_cur(1, 0);
              sprintf(line, "T:%.1fC", saved_temperature);
              lcd_send_string(line);
              lcd_sent = 1;
              sprintf(msg, "Da gui len lcd\r\n\n");
              HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
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
