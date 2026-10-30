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
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SERVO_COUNT             6U
#define SERVO_NEUTRAL_DEG       90U
#define SERVO_MIN_PULSE_US      500U
#define SERVO_MAX_PULSE_US      2500U
#define SERVO_UPDATE_PERIOD_MS  20U
#define BLUETOOTH_BAUDRATE      9600 /* HC-06 UART baud rate confirmed by the user. */
#define BLUETOOTH_LINE_SIZE    40U
#define BLUETOOTH_IDLE_MS      100U

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

/* Each servo starts DISABLED. PWM is enabled only after its first command. */
static TIM_HandleTypeDef *const servo_timers[SERVO_COUNT] = {
  &htim3, &htim3, &htim3, &htim3, &htim4, &htim4
};
static const uint32_t servo_channels[SERVO_COUNT] = {
  TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
  TIM_CHANNEL_1, TIM_CHANNEL_2
};
/* Adjust individually to match the real mechanical limits before assembly. */
/* Conservative limits for FIRST TEST only; widen after mechanical calibration. */
static const uint8_t servo_min_deg[SERVO_COUNT] = {0, 0, 0, 0, 0, 45};
static const uint8_t servo_max_deg[SERVO_COUNT] = {180, 180, 180, 180, 180, 135};
static uint8_t servo_enabled[SERVO_COUNT] = {0};
static uint8_t servo_current_deg[SERVO_COUNT] = {90, 135, 60, 90, 60, 90};
static uint8_t servo_target_deg[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};

static char bluetooth_line[BLUETOOTH_LINE_SIZE];
static uint8_t bluetooth_line_length = 0;
static uint8_t bluetooth_line_overflow = 0;
static uint32_t bluetooth_last_byte_tick = 0;
static uint32_t servo_last_update = 0;

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

static void Bluetooth_Send(const char *message);
static void Bluetooth_ProcessRx(void);
static void Bluetooth_FinishLine(void);
static void Bluetooth_HandleCommand(char *line);
static void Servo_Update(void);
static void Servo_WriteAngle(uint8_t index, uint8_t angle);
static uint8_t Servo_Enable(uint8_t index);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Bluetooth_Send(const char *message)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)message, (uint16_t)strlen(message), 100);
}

static void Servo_WriteAngle(uint8_t index, uint8_t angle)
{
  /* TIM3/TIM4 tick at 1 MHz: one compare count = 1 microsecond. */
  uint32_t pulse = SERVO_MIN_PULSE_US +
    ((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) + 90U) / 180U;
  __HAL_TIM_SET_COMPARE(servo_timers[index], servo_channels[index], pulse);
}

static uint8_t Servo_Enable(uint8_t index)
{
  if (servo_enabled[index])
    return 1U;

  /* The servo's real position is unknown. Its first pulse is neutral (90 deg). */
  Servo_WriteAngle(index, servo_current_deg[index]);
  if (HAL_TIM_PWM_Start(servo_timers[index], servo_channels[index]) != HAL_OK)
    return 0U;

  servo_enabled[index] = 1U;
  return 1U;
}

static void Servo_Update(void)
{
  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - servo_last_update) < SERVO_UPDATE_PERIOD_MS)
    return;
  servo_last_update = now;

  for (uint8_t i = 0; i < SERVO_COUNT; i++)
  {
    if (!servo_enabled[i] || servo_current_deg[i] == servo_target_deg[i])
      continue;

    if (servo_current_deg[i] < servo_target_deg[i])
      servo_current_deg[i]++;
    else
      servo_current_deg[i]--;

    Servo_WriteAngle(i, servo_current_deg[i]);
  }
}

static char *SkipSpaces(char *s)
{
  while (*s == ' ' || *s == '\t')
    s++;
  return s;
}

static uint8_t IsEnd(const char *s)
{
  return *s == '\0';
}

static void Bluetooth_HandleCommand(char *line)
{
  char reply[80];
  char *p = SkipSpaces(line);

  /* Strip trailing spaces, so commands work with CR, LF or CRLF terminals. */
  size_t len = strlen(p);
  while (len > 0U && (p[len - 1U] == ' ' || p[len - 1U] == '\t'))
    p[--len] = '\0';
  if (len == 0U)
    return;

  if (strcmp(p, "HELP") == 0 || strcmp(p, "help") == 0)
  {
    Bluetooth_Send("J1 90  : set joint 1 to 90 deg\r\n");
    Bluetooth_Send("J1 +1  : increase joint 1 by 1 deg\r\n");
    Bluetooth_Send("J1 -1  : decrease joint 1 by 1 deg\r\n");
    Bluetooth_Send("STATUS | STOP | OFF 1 | OFF ALL | PING\r\n");
    return;
  }
  if (strcmp(p, "PING") == 0 || strcmp(p, "ping") == 0)
  {
    Bluetooth_Send("PONG\r\n");
    return;
  }
  if (strcmp(p, "STOP") == 0 || strcmp(p, "stop") == 0)
  {
    /* Hold the last commanded angle. This is NOT an emergency power cut. */
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
      servo_target_deg[i] = servo_current_deg[i];
    Bluetooth_Send("OK STOP (holding position)\r\n");
    return;
  }
  if (strcmp(p, "STATUS") == 0 || strcmp(p, "status") == 0)
  {
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
    {
      snprintf(reply, sizeof(reply), "J%u: %u -> %u, %s\r\n",
               (unsigned)(i + 1U), (unsigned)servo_current_deg[i],
               (unsigned)servo_target_deg[i], servo_enabled[i] ? "ON" : "OFF");
      Bluetooth_Send(reply);
    }
    return;
  }
  if (strncmp(p, "OFF ", 4) == 0 || strncmp(p, "off ", 4) == 0)
  {
    char *arg = SkipSpaces(p + 4);
    if (strcmp(arg, "ALL") == 0 || strcmp(arg, "all") == 0)
    {
      for (uint8_t i = 0; i < SERVO_COUNT; i++)
      {
        if (servo_enabled[i])
          HAL_TIM_PWM_Stop(servo_timers[i], servo_channels[i]);
        servo_enabled[i] = 0U;
        servo_current_deg[i] = SERVO_NEUTRAL_DEG;
        servo_target_deg[i] = SERVO_NEUTRAL_DEG;
      }
      Bluetooth_Send("OK OFF ALL (no holding torque)\r\n");
      return;
    }
    if (arg[0] >= '1' && arg[0] <= '6' && IsEnd(arg + 1))
    {
      uint8_t i = (uint8_t)(arg[0] - '1');
      if (servo_enabled[i])
        HAL_TIM_PWM_Stop(servo_timers[i], servo_channels[i]);
      servo_enabled[i] = 0U;
      servo_current_deg[i] = SERVO_NEUTRAL_DEG;
      servo_target_deg[i] = SERVO_NEUTRAL_DEG;
      snprintf(reply, sizeof(reply), "OK J%u OFF (no holding torque)\r\n",
               (unsigned)(i + 1U));
      Bluetooth_Send(reply);
      return;
    }
    Bluetooth_Send("ERR use OFF 1..6 or OFF ALL\r\n");
    return;
  }

  /* One joint per command. An explicit +/- means relative movement. */
  if ((p[0] == 'J' || p[0] == 'j') &&
      p[1] >= '1' && p[1] <= '6' &&
      (p[2] == ' ' || p[2] == '\t'))
  {
    uint8_t i = (uint8_t)(p[1] - '1');
    char *value_text = SkipSpaces(p + 3);
    int sign = 1;
    uint8_t relative = 0;
    if (*value_text == '+' || *value_text == '-')
    {
      relative = 1U;
      if (*value_text == '-')
        sign = -1;
      value_text++;
    }
    if (*value_text < '0' || *value_text > '9')
    {
      Bluetooth_Send("ERR missing angle\r\n");
      return;
    }
    int value = 0;
    while (*value_text >= '0' && *value_text <= '9')
    {
      value = value * 10 + (*value_text++ - '0');
      if (value > 180)
      {
        Bluetooth_Send("ERR angle magnitude must be 0..180\r\n");
        return;
      }
    }
    if (!IsEnd(SkipSpaces(value_text)))
    {
      Bluetooth_Send("ERR invalid angle text\r\n");
      return;
    }
    int target = relative ? (int)servo_target_deg[i] + sign * value : value;
    if (target < (int)servo_min_deg[i] || target > (int)servo_max_deg[i])
    {
      snprintf(reply, sizeof(reply), "ERR J%u allowed range %u..%u deg\r\n",
               (unsigned)(i + 1U), (unsigned)servo_min_deg[i],
               (unsigned)servo_max_deg[i]);
      Bluetooth_Send(reply);
      return;
    }

    if (!Servo_Enable(i))
    {
      Bluetooth_Send("ERR PWM start failed\r\n");
      return;
    }
    /* Only this joint may be in motion; the others hold their current angles. */
    for (uint8_t j = 0; j < SERVO_COUNT; j++)
      if (j != i)
        servo_target_deg[j] = servo_current_deg[j];
    servo_target_deg[i] = (uint8_t)target;

    snprintf(reply, sizeof(reply), "OK J%u target=%u deg\r\n",
             (unsigned)(i + 1U), (unsigned)target);
    Bluetooth_Send(reply);
    return;
  }
  Bluetooth_Send("ERR unknown command; send HELP\r\n");
}

static void Bluetooth_FinishLine(void)
{
  if (bluetooth_line_overflow)
    Bluetooth_Send("ERR command too long\r\n");
  else if (bluetooth_line_length != 0U)
  {
    bluetooth_line[bluetooth_line_length] = '\0';
    Bluetooth_HandleCommand(bluetooth_line);
  }
  bluetooth_line_length = 0U;
  bluetooth_line_overflow = 0U;
}

static void Bluetooth_ProcessRx(void)
{
  uint8_t ch;
  /* Same simple polling receive path as the successful BLE echo test. */
  if (HAL_UART_Receive(&huart1, &ch, 1U, 1U) == HAL_OK)
  {
    bluetooth_last_byte_tick = HAL_GetTick();
    if (ch == '\r' || ch == '\n')
    {
      Bluetooth_FinishLine();
    }
    else if (!bluetooth_line_overflow && ch >= 32U && ch <= 126U)
    {
      if (bluetooth_line_length < (BLUETOOTH_LINE_SIZE - 1U))
        bluetooth_line[bluetooth_line_length++] = (char)ch;
      else
        bluetooth_line_overflow = 1U;
    }
  }
  else if ((bluetooth_line_length != 0U || bluetooth_line_overflow) &&
           (uint32_t)(HAL_GetTick() - bluetooth_last_byte_tick) >= BLUETOOTH_IDLE_MS)
  {
    /* Phone apps may send an entire command without CR/LF. */
    Bluetooth_FinishLine();
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
  /* Do not actuate any joint automatically at boot. */
	
  servo_last_update = HAL_GetTick();
  huart1.Init.BaudRate = BLUETOOTH_BAUDRATE;
  if (HAL_UART_Init(&huart1) != HAL_OK)
    Error_Handler();
  Bluetooth_Send("RobotArm ready. Send PING, STATUS, or J1 90\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    Bluetooth_ProcessRx();
    Servo_Update();
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
