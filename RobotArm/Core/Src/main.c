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
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Max_Servo_Speed 20.0f
#define DEG_TO_RAD 0.01745329252f
#define RAD_TO_DEG 57.29577951f
#define CONTROL_DT 0.02f
#define LINK1 0.1f
#define LINK2 0.1f
#define LINK3 0.09f
#define LAMDA 0.02f
#define MAX_X_SPEED 0.015f
#define MAX_Y_SPEED 0.015f
#define BT_TIMEOUT_MS 200U
#define BT_PERIOD_MS 20U
#define BT_BUFFER_SIZE 64U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
typedef struct
{
    int8_t joy_x;
    int8_t joy_y;
    int8_t joy_turn;

    uint8_t enable;
    uint32_t last_rx_ms;
} BluetoothCommand;

static BluetoothCommand bluetoothCMD = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t bt_rx_count = 0;
volatile uint32_t bt_valid_count = 0;
volatile uint32_t bt_last_byte_ms = 0;
volatile uint32_t bt_last_valid_ms = 0;
volatile uint8_t bt_last_byte = 0;

uint8_t rx_buffer[128];
uint32_t rx_len;
uint8_t uart_rx_byte;
volatile uint8_t rx_ready = 0;
static uint8_t bt_uart_byte;
static uint8_t bt_rx_buffer[BT_BUFFER_SIZE];
static uint32_t bt_rx_len = 0;
static uint8_t bt_discard_line = 0;
static volatile uint8_t bt_rx_ready = 0;
char bt_last_line[BT_BUFFER_SIZE] = {0};
static uint8_t bluetooth_armed = 0;


float servo_angle[6] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f};
float servoX_angle[3] = {90.0f, 90.0f, 90.0f};
static const float direction[3] = {1.0f, 1.0f, 1.0f};
static const float angle_offset[3] = {0.0f, 0.0f, 0.0f};

static const float angle_min[3] = {45.0f, 45.0f, 45.0f};

static const float angle_max[3] = {135.0f, 135.0f, 135.0f};

static float clampf(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}


void UART_Start_Receive(void)
{
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}

void Bluetooth_Start_Receive(void)
{
    HAL_UART_Receive_IT(&huart3, &bt_uart_byte, 1);
}

void Servo_ParseCommand(char *data)
{
    float a[6];

    if (sscanf(data, "%f,%f,%f,%f,%f,%f", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]) == 6)
    {
        for (int i = 0; i < 6; i++)
        {
            servo_angle[i] = a[i];
        }
    }
}

void Caculate(float vx, float vy){
		float q[3];
		for(int i = 0; i < 3; i++){
				q[i] = angle_offset[i] + direction[i] * (servoX_angle[i] - 90.0f) * DEG_TO_RAD;
		}
		float t1 = q[0];
		float t2 = q[0] + q[1];
		float t3 = q[0] + q[1] + q[2];
		
		float sinx1 = sinf(t1);
		float sinx2 = sinf(t2);
		float sinx3 = sinf(t3);
		
		float cosy1 = cosf(t1);
		float cosy2 = cosf(t2);
		float cosy3 = cosf(t3);
		
		float JacobianX[3] = {
				-LINK1 * sinx1 - LINK2 * sinx2 - LINK3 * sinx3,
				-LINK2 * sinx2 - LINK3 * sinx3,
				-LINK3 * sinx3
		};
		float JacobianY[3] = {
				LINK1 * cosy1 + LINK2 * cosy2 + LINK3 * cosy3,
				LINK2 * cosy2 + LINK3 * cosy3,
				LINK3 * cosy3
		};
		
		float dq[3] = {0};
		
		float A = LAMDA * LAMDA;
		float B = 0.0f;
		float C = LAMDA * LAMDA;
		
		for(int i = 0; i < 3; i++){
				A += JacobianX[i] * JacobianX[i];
				B += JacobianX[i] * JacobianY[i];
				C += JacobianY[i] * JacobianY[i];
		}
		if(vx != 0.0f && vy == 0.0f){
				for(int i = 0; i < 3; i++){
						dq[i] = JacobianX[i] * vx/A;
				}
		}
		else if (vx == 0.0f && vy != 0.0f)
    {
        for (int i = 0; i < 3; i++)
            dq[i] = JacobianY[i] * vy / C;
    }
		else if (vx != 0.0f && vy != 0.0f){
				float det = A * C - B * B;
				if (det > 1e-8f)
        {
            float ux = (C*vx - B*vy) / det;
            float uy = (A*vy - B*vx) / det;

            for (int i = 0; i < 3; i++) dq[i] = JacobianX[i]*ux + JacobianY[i]*uy;
        }
		}
		for (int i = 0; i < 3; i++)
    {
        float speed_deg = dq[i] * RAD_TO_DEG * direction[i];

        speed_deg = clampf(speed_deg, -Max_Servo_Speed, Max_Servo_Speed);

        servoX_angle[i] += speed_deg * CONTROL_DT;

        servoX_angle[i] = clampf(servoX_angle[i], angle_min[i], angle_max[i]);
    }
}

void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, float angle){
    if (angle < 0) angle = 0;

    if (angle > 180) angle = 180;

    uint32_t pulse = 1000 + (uint32_t)((angle / 180.0f) * 1000.0f);

    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

void Bluetooth_Update20ms(void)
{
    uint32_t now = HAL_GetTick();

    // Mất lệnh quá 200ms thì giữ nguyên góc
    if ((uint32_t)(now - bluetoothCMD.last_rx_ms) > 200)
        return;

    // Người dùng không giữ nút cho phép
    if (!bluetoothCMD.enable)
        return;

    float x = bluetoothCMD.joy_x / 100.0f;
    float y = bluetoothCMD.joy_y / 100.0f;

    // Dead-zone
    if (fabsf(x) < 0.08f) x = 0.0f;
    if (fabsf(y) < 0.08f) y = 0.0f;

    // Chuyển joystick thành m/s
    float vx = x * MAX_X_SPEED;
    float vy = y * MAX_Y_SPEED;

    if (vx == 0.0f && vy == 0.0f)
        return;

    Caculate(vx, vy);

    Servo_SetAngle(&htim2, TIM_CHANNEL_3, servoX_angle[0]);
		Servo_SetAngle(&htim2, TIM_CHANNEL_4, servoX_angle[1]);
		Servo_SetAngle(&htim3, TIM_CHANNEL_2, servoX_angle[2]);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (!rx_ready)
        {
            if (uart_rx_byte == '\n')
            {
                rx_buffer[rx_len] = '\0';
                rx_ready = 1;
            }
            else if (rx_len < sizeof(rx_buffer) - 1)
            {
                rx_buffer[rx_len++] = uart_rx_byte;
            }
            else
            {
                rx_len = 0; 
            }
        }

        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
		else if (huart->Instance == USART3){
				bt_last_byte = bt_uart_byte;
        bt_rx_count++;
        bt_last_byte_ms = HAL_GetTick();

        if (bt_uart_byte == '\n')
        {
            if (!bt_rx_ready && !bt_discard_line)
            {
                bt_rx_buffer[bt_rx_len] = '\0';
                bt_rx_ready = 1;
            }
            bt_discard_line = 0;
        }
        else if (bt_uart_byte != '\r' && !bt_rx_ready && !bt_discard_line)
        {
            if (bt_rx_len < BT_BUFFER_SIZE - 1U)
                bt_rx_buffer[bt_rx_len++] = bt_uart_byte;
            else
            {
                bt_discard_line = 1;
                bt_rx_len = 0;
            }
        }

        HAL_UART_Receive_IT(&huart3, &bt_uart_byte, 1);
		}
}

void Bluetooth_ParseCommand(char *data){
		int x, y, turn, enable, used = 0;
    if (sscanf(data, "J,%d,%d,%d,%d%n", &x, &y, &turn, &enable, &used) != 4 ||
        used == 0 || data[used] != '\0' ||
        x < -100 || x > 100 || y < -100 || y > 100 ||
        turn < -100 || turn > 100 || (enable != 0 && enable != 1))
        return;

    // Moi lan vao manual phai tha joystick va nut enable truoc.
    if (enable == 0 && x >= -8 && x <= 8 &&
        y >= -8 && y <= 8 && turn >= -8 && turn <= 8)
        bluetooth_armed = 1;

    bluetoothCMD.joy_x = (int8_t)x;
    bluetoothCMD.joy_y = (int8_t)y;
    bluetoothCMD.joy_turn = (int8_t)turn;
    bluetoothCMD.enable = (uint8_t)(enable && bluetooth_armed);
    bluetoothCMD.last_rx_ms = HAL_GetTick();
    bt_valid_count++;
    bt_last_valid_ms = bluetoothCMD.last_rx_ms;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	UART_Start_Receive();
	Bluetooth_Start_Receive();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t last_bt_update_ms = HAL_GetTick();
  char ros2_line[sizeof(rx_buffer)];
  char bt_line[BT_BUFFER_SIZE];
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(rx_ready){
			rx_ready = 0;
			rx_len = 0;
			Servo_ParseCommand((char *)rx_buffer);

      Servo_SetAngle(&htim2, TIM_CHANNEL_2, servo_angle[0]);
      Servo_SetAngle(&htim2, TIM_CHANNEL_3, servo_angle[1]);
      Servo_SetAngle(&htim2, TIM_CHANNEL_4, servo_angle[2]);

      Servo_SetAngle(&htim3, TIM_CHANNEL_1, servo_angle[3]);
      Servo_SetAngle(&htim3, TIM_CHANNEL_2, servo_angle[4]);
			Servo_SetAngle(&htim3, TIM_CHANNEL_3, servo_angle[5]);

        // Cho ph�p nh?n g�i ti?p theo
      __disable_irq();
      __enable_irq();

		}
		if (bt_rx_ready)
		{
			__disable_irq();
			memcpy(bt_line, bt_rx_buffer, bt_rx_len + 1U);
			bt_rx_ready = 0;
			bt_rx_len = 0;
			__enable_irq();

			strcpy(bt_last_line, bt_line); // Debug: goi HC-06 gan nhat
			Bluetooth_ParseCommand(bt_line);
		}

		uint32_t now = HAL_GetTick();
		if ((uint32_t)(now - last_bt_update_ms) >= BT_PERIOD_MS)
		{
			last_bt_update_ms = now;
			Bluetooth_Update20ms();
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
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
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
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
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
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
