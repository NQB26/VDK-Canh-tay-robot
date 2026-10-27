/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Kenh nhan lenh UART: DMA vong (circular) + bo dem dong lenh */
typedef struct
{
  UART_HandleTypeDef *huart;
  uint8_t dma_buf[128];       /* bo dem DMA vong */
  uint16_t rd;                /* chi so doc cua CPU */
  char line[96];              /* dong lenh dang ghep */
  uint8_t line_len;
  uint8_t overflow;           /* dong qua dai -> bo qua */
  volatile uint8_t restart;   /* bao loi UART -> khoi dong lai DMA */
} UartRx_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SERVO         6
#define RX_DMA_SIZE       128
#define LINE_MAX_LEN      96

/* Do rong xung servo (us) ung voi 0 va 180 do.
 * Giu 1000..2000 giong project RobotArm (F103). Neu servo quay du 180 do
 * voi 500..2500 thi doi lai o day. */
#define SERVO_MIN_US      1000
#define SERVO_MAX_US      2000

#define SERVO_TICK_S      0.01f   /* TIM2 = 100 Hz (10 ms) */
#define SPEED_MIN_DPS     5.0f
#define SPEED_MAX_DPS     720.0f
#define SPEED_DEFAULT_DPS 90.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
static UartRx_t rx_ble;   /* USART1: module BLE (dien thoai) */
static UartRx_t rx_pc;    /* USART2: laptop / cau noi USB-UART */

static float servo_target[NUM_SERVO]  = {90, 90, 90, 90, 90, 90};
static float servo_current[NUM_SERVO] = {90, 90, 90, 90, 90, 90};

/* Gioi han goc an toan tung khop (do). Chinh theo co khi cua canh tay
 * de tranh dam vao chan de / tu va cham. */
static const float servo_min[NUM_SERVO] = {0, 0, 0, 0, 0, 0};
static const float servo_max[NUM_SERVO] = {180, 180, 180, 180, 180, 180};

static volatile float max_speed_dps = SPEED_DEFAULT_DPS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Servo_Write(uint8_t idx, float deg);
static void UartRx_Start(UartRx_t *ch);
static void UartRx_Poll(UartRx_t *ch);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static float Clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void Servo_Write(uint8_t idx, float deg)
{
  uint32_t pulse;

  deg = Clampf(deg, 0.0f, 180.0f);
  pulse = SERVO_MIN_US + (uint32_t)((deg / 180.0f) * (float)(SERVO_MAX_US - SERVO_MIN_US));

  switch (idx)
  {
    case 0: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse); break; /* SERVO_1 PA6 */
    case 1: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse); break; /* SERVO_2 PA7 */
    case 2: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse); break; /* SERVO_3 PB0 */
    case 3: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse); break; /* SERVO_4 PB1 */
    case 4: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse); break; /* SERVO_5 PB6 */
    case 5: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse); break; /* SERVO_6 PB7 */
    default: break;
  }
}

static void Servo_SetTarget(uint8_t idx, float deg)
{
  if (idx >= NUM_SERVO) return;
  servo_target[idx] = Clampf(deg, servo_min[idx], servo_max[idx]);
}

/* Doc 6 so thuc cach nhau boi dau ',' -> out[]. Tra ve 1 neu hop le. */
static uint8_t Parse6(const char *s, float out[NUM_SERVO])
{
  char *end;
  double v;
  uint8_t i;

  for (i = 0; i < NUM_SERVO; i++)
  {
    v = strtod(s, &end);
    if (end == s || v != v) return 0;   /* khong co so, hoac NaN */
    out[i] = (float)v;
    s = end;
    if (i < NUM_SERVO - 1)
    {
      if (*s != ',') return 0;
      s++;
    }
  }
  return 1;
}

/* Cac lenh (ket thuc bang '\n', giao thuc cu cua RobotArm van dung duoc):
 *   a1,a2,a3,a4,a5,a6   dat goc muc tieu 6 servo (do), vd "90,45.5,120,90,90,30"
 *   Sn:goc              dat 1 servo, n = 1..6,        vd "S3:135"
 *   Vtoc_do             toc do toi da (do/giay),      vd "V90"
 */
static void Cmd_Process(const char *line)
{
  float a[NUM_SERVO];
  uint8_t i;

  if (line[0] == 'S' || line[0] == 's')
  {
    char *end;
    long n = strtol(line + 1, &end, 10);
    if (end != line + 1 && *end == ':' && n >= 1 && n <= NUM_SERVO)
    {
      double v = strtod(end + 1, &end);
      if (v == v) Servo_SetTarget((uint8_t)(n - 1), (float)v);
    }
  }
  else if (line[0] == 'V' || line[0] == 'v')
  {
    char *end;
    double v = strtod(line + 1, &end);
    if (end != line + 1 && v == v)
    {
      max_speed_dps = Clampf((float)v, SPEED_MIN_DPS, SPEED_MAX_DPS);
    }
  }
  else if (Parse6(line, a))
  {
    for (i = 0; i < NUM_SERVO; i++) Servo_SetTarget(i, a[i]);
  }
}

static void UartRx_Start(UartRx_t *ch)
{
  ch->rd = 0;
  ch->line_len = 0;
  ch->overflow = 0;
  ch->restart = 0;
  HAL_UART_Receive_DMA(ch->huart, ch->dma_buf, RX_DMA_SIZE);
}

/* Goi lien tuc trong vong lap chinh: lay byte moi tu bo dem DMA vong,
 * ghep thanh dong, gap '\n' thi xu ly lenh. */
static void UartRx_Poll(UartRx_t *ch)
{
  uint16_t head;
  uint8_t c;

  if (ch->restart)
  {
    UartRx_Start(ch);
    return;
  }

  head = (uint16_t)(RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(ch->huart->hdmarx));
  while (ch->rd != head)
  {
    c = ch->dma_buf[ch->rd];
    ch->rd = (uint16_t)((ch->rd + 1) % RX_DMA_SIZE);

    if (c == '\n' || c == '\r')
    {
      if (ch->line_len > 0 && !ch->overflow)
      {
        ch->line[ch->line_len] = '\0';
        Cmd_Process(ch->line);
      }
      ch->line_len = 0;
      ch->overflow = 0;
    }
    else if (ch->line_len < LINE_MAX_LEN - 1)
    {
      ch->line[ch->line_len++] = (char)c;
    }
    else
    {
      ch->overflow = 1;
    }
  }
}

/* Loi UART (overrun/noise/framing): HAL dung DMA -> danh dau de khoi dong lai */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) rx_ble.restart = 1;
  else if (huart->Instance == USART2) rx_pc.restart = 1;
}

/* TIM2 100 Hz: dua goc hien tai tien dan toi goc muc tieu voi toc do toi da
 * max_speed_dps -> servo chuyen dong mem, khong giat khi keo thanh truot. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint8_t i;
  float step, diff;

  if (htim->Instance != TIM2) return;

  step = max_speed_dps * SERVO_TICK_S;
  for (i = 0; i < NUM_SERVO; i++)
  {
    diff = servo_target[i] - servo_current[i];
    if (diff > step)       servo_current[i] += step;
    else if (diff < -step) servo_current[i] -= step;
    else                   servo_current[i] = servo_target[i];
    Servo_Write(i, servo_current[i]);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  /* Dua servo ve vi tri ban dau (90 do) */
  {
    uint8_t i;
    for (i = 0; i < NUM_SERVO; i++) Servo_Write(i, servo_current[i]);
  }

  /* Nhan lenh tu BLE (USART1) va laptop (USART2) bang DMA vong */
  rx_ble.huart = &huart1;
  rx_pc.huart  = &huart2;
  UartRx_Start(&rx_ble);
  UartRx_Start(&rx_pc);

  /* TIM2 100 Hz: cap nhat chuyen dong mem */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    UartRx_Poll(&rx_ble);
    UartRx_Poll(&rx_pc);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 1500;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
#ifdef USE_FULL_ASSERT
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
