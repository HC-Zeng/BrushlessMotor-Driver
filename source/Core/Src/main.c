/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static uint8_t phase;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static uint32_t GetTimeStampUS()
{
    // get ms
    uint32_t m = HAL_GetTick();
    // get tick reload value
    const uint32_t tms = SysTick->LOAD + 1;
    // get tick value
    __IO uint32_t u = tms - SysTick->VAL;
    // return value
    return(m*1000+(u*1000)/tms);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static void initStep()
{
    // A: --
    HAL_GPIO_WritePin(A1_GPIO_Port,A1_Pin,1);
    HAL_GPIO_WritePin(A2_GPIO_Port,A2_Pin,0);
    // B: --
    HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,1);
    HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,0);
    // C: --
    HAL_GPIO_WritePin(C1_GPIO_Port,C1_Pin,1);
    HAL_GPIO_WritePin(C2_GPIO_Port,C2_Pin,0);
}
static void stepAB()
{
    // A: 1
    HAL_GPIO_WritePin(A1_GPIO_Port,A1_Pin,0);
    HAL_GPIO_WritePin(A2_GPIO_Port,A2_Pin,0);

    // B: 0
    HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,1);
    HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,1);

    // C: --
    HAL_GPIO_WritePin(C1_GPIO_Port,C1_Pin,1);
    HAL_GPIO_WritePin(C2_GPIO_Port,C2_Pin,0);
}

static void stepAC()
{
    // A: 1
    HAL_GPIO_WritePin(A1_GPIO_Port,A1_Pin,0);
    HAL_GPIO_WritePin(A2_GPIO_Port,A2_Pin,0);

    // C: 0
    HAL_GPIO_WritePin(C1_GPIO_Port,C1_Pin,1);
    HAL_GPIO_WritePin(C2_GPIO_Port,C2_Pin,1);

    // B: --
    HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,1);
    HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,0);
}

static void stepBC()
{
    // B: 1
    HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,0);
    HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,0);

    // C: 0
    HAL_GPIO_WritePin(C1_GPIO_Port,C1_Pin,1);
    HAL_GPIO_WritePin(C2_GPIO_Port,C2_Pin,1);

    // A: --
    HAL_GPIO_WritePin(A1_GPIO_Port,A1_Pin,1);
    HAL_GPIO_WritePin(A2_GPIO_Port,A2_Pin,0);
}

static void stepBA()
{
    // B: 1
    HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,0);
    HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,0);

    // A: 0
    HAL_GPIO_WritePin(A1_GPIO_Port,A1_Pin,1);
    HAL_GPIO_WritePin(A2_GPIO_Port,A2_Pin,1);

    // C: --
    HAL_GPIO_WritePin(C1_GPIO_Port,C1_Pin,1);
    HAL_GPIO_WritePin(C2_GPIO_Port,C2_Pin,0);
}

static void stepCA()
{
    // C: 1
    HAL_GPIO_WritePin(C1_GPIO_Port,C1_Pin,0);
    HAL_GPIO_WritePin(C2_GPIO_Port,C2_Pin,0);

    // A: 0
    HAL_GPIO_WritePin(A1_GPIO_Port,A1_Pin,1);
    HAL_GPIO_WritePin(A2_GPIO_Port,A2_Pin,1);

    // B: --
    HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,1);
    HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,0);
}

static void stepCB()
{
    // C: 1
    HAL_GPIO_WritePin(C1_GPIO_Port,C1_Pin,0);
    HAL_GPIO_WritePin(C2_GPIO_Port,C2_Pin,0);

    // B: 0
    HAL_GPIO_WritePin(B1_GPIO_Port,B1_Pin,1);
    HAL_GPIO_WritePin(B2_GPIO_Port,B2_Pin,1);

    // A: --
    HAL_GPIO_WritePin(A1_GPIO_Port,A1_Pin,1);
    HAL_GPIO_WritePin(A2_GPIO_Port,A2_Pin,0);
}

void nextStep()
{
    switch (phase)
    {
        case 0:
            stepAB();
            break;
        case 1:
            stepAC();
            break;
        case 2:
            stepBC();
            break;
        case 3:
            stepBA();
            break;
        case 4:
            stepCA();
            break;
        case 5:
            stepCB();
            break;
        default:
            phase = 0;
    }
    phase ++;
    phase = phase % 6; // 0, 1, 2, 3, 4, 5
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */
  static uint32_t timeStamp;

  initStep();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t crr = GetTimeStampUS();
      if(crr - timeStamp >= 20000)
      {
          timeStamp = crr;
          nextStep();
      }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
