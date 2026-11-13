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
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SERVO_COUNT             6U
#define SERVO_NEUTRAL_DEG       90U
#define SERVO_MIN_PULSE_US      500U
#define SERVO_MAX_PULSE_US      2000U
#define SERVO_UPDATE_PERIOD_MS  20U
#define BLUETOOTH_BAUDRATE      9600 /* HC-06 UART baud rate confirmed by the user. */
#define BLUETOOTH_LINE_SIZE        80U
#define SERIAL_LINE_SIZE           128U
#define BT_TIMEOUT_MS              200U
#define BT_PERIOD_MS               20U
#define BT_DEADZONE                8

/* Retained from the joystick/Jacobian source. Length unit: metre. */
#define Max_Servo_Speed            20.0f
#define DEG_TO_RAD                 0.01745329252f
#define RAD_TO_DEG                 57.29577951f
#define CONTROL_DT                 0.02f
#define LINK1                      0.10f
#define LINK2                      0.10f
#define LINK3                      0.09f
#define LAMDA                      0.02f
#define MAX_X_SPEED                0.015f
#define MAX_Y_SPEED                0.015f
#define MAX_Z_TURN_SPEED_DPS       20.0f

/* Servo mapping: J1 is base yaw; J2-J4 form the 3-link planar XY chain. */
#define SERVO_BASE_YAW             0U
#define SERVO_XY_JOINT_1           1U
#define SERVO_XY_JOINT_2           2U
#define SERVO_XY_JOINT_3           3U

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
static uint8_t servo_target_deg[SERVO_COUNT] = {90, 135, 60, 90, 60, 90};

/* HC-06 joystick receiver on the original USART1 / 9600-baud connection. */
static char bluetooth_line[BLUETOOTH_LINE_SIZE];
static volatile uint8_t bluetooth_line_length = 0U;
static volatile uint8_t bluetooth_line_overflow = 0U;
static volatile uint8_t bluetooth_line_ready = 0U;
static uint8_t bluetooth_rx_byte = 0U;

/* USART2 / 115200 accepts six simultaneous joint targets as CSV. */
static char serial_line[SERIAL_LINE_SIZE];
static volatile uint16_t serial_line_length = 0U;
static volatile uint8_t serial_line_overflow = 0U;
static volatile uint8_t serial_line_ready = 0U;
static uint8_t serial_rx_byte = 0U;

/* Joystick packet from the second file: J,x,y,turn,enable; axes are -100..100. */
typedef struct
{
  int8_t joy_x;
  int8_t joy_y;
  int8_t joy_turn;
  uint8_t enable;
  uint32_t last_rx_ms;
} BluetoothCommand;

static BluetoothCommand bluetoothCMD = {0};
static uint8_t bluetooth_armed = 0U;
static uint8_t joystick_control_initialized = 0U;
static float servoX_angle[3] = {135.0f, 60.0f, 90.0f};
static float base_yaw_angle = 90.0f;
static const float direction[3] = {1.0f, 1.0f, 1.0f};
static const float angle_offset[3] = {0.0f, 0.0f, 0.0f};
static const float angle_min[3] = {45.0f, 45.0f, 45.0f};
static const float angle_max[3] = {135.0f, 135.0f, 135.0f};
static uint32_t last_bt_update_ms = 0U;
static uint32_t servo_last_update = 0U;

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
static void Serial_ProcessRx(void);
static void Servo_Update(void);
static void Servo_WriteAngle(uint8_t index, uint8_t angle);
static uint8_t Servo_Enable(uint8_t index);
static uint8_t Servo_SetAllTargets(const float angles[SERVO_COUNT]);
static void Bluetooth_ParseJoystickCommand(char *data);
static void Bluetooth_Update20ms(void);
static void Caculate(float vx, float vy);
static float clampf(float value, float min_value, float max_value);
static void StopJoystickAndHold(void);

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

  /* Start with the last software-known angle; the real pose is not sensed. */
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

static float clampf(float value, float min_value, float max_value)
{
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

/*
 * Damped-least-squares differential inverse kinematics from the joystick file.
 * The three links move in one XY plane. servoX_angle contains relative servo
 * angles, with 90 degrees treated as the zero-angle reference.
 */
static void Caculate(float vx, float vy)
{
  float q[3];
  float t1, t2, t3;
  float sinx1, sinx2, sinx3;
  float cosy1, cosy2, cosy3;
  float JacobianX[3];
  float JacobianY[3];
  float dq[3] = {0.0f, 0.0f, 0.0f};
  float A = LAMDA * LAMDA;
  float B = 0.0f;
  float C = LAMDA * LAMDA;

  for (uint8_t i = 0U; i < 3U; i++)
    q[i] = angle_offset[i] + direction[i] *
           (servoX_angle[i] - 90.0f) * DEG_TO_RAD;

  t1 = q[0];
  t2 = q[0] + q[1];
  t3 = q[0] + q[1] + q[2];

  sinx1 = sinf(t1);
  sinx2 = sinf(t2);
  sinx3 = sinf(t3);
  cosy1 = cosf(t1);
  cosy2 = cosf(t2);
  cosy3 = cosf(t3);

  JacobianX[0] = -LINK1 * sinx1 - LINK2 * sinx2 - LINK3 * sinx3;
  JacobianX[1] =                  - LINK2 * sinx2 - LINK3 * sinx3;
  JacobianX[2] =                                   - LINK3 * sinx3;

  JacobianY[0] = LINK1 * cosy1 + LINK2 * cosy2 + LINK3 * cosy3;
  JacobianY[1] =                 LINK2 * cosy2 + LINK3 * cosy3;
  JacobianY[2] =                                  LINK3 * cosy3;

  for (uint8_t i = 0U; i < 3U; i++)
  {
    A += JacobianX[i] * JacobianX[i];
    B += JacobianX[i] * JacobianY[i];
    C += JacobianY[i] * JacobianY[i];
  }

  if (fabsf(vx) > 1.0e-8f && fabsf(vy) <= 1.0e-8f)
  {
    for (uint8_t i = 0U; i < 3U; i++)
      dq[i] = JacobianX[i] * vx / A;
  }
  else if (fabsf(vx) <= 1.0e-8f && fabsf(vy) > 1.0e-8f)
  {
    for (uint8_t i = 0U; i < 3U; i++)
      dq[i] = JacobianY[i] * vy / C;
  }
  else if (fabsf(vx) > 1.0e-8f && fabsf(vy) > 1.0e-8f)
  {
    float det = A * C - B * B;
    if (det > 1.0e-8f)
    {
      float ux = (C * vx - B * vy) / det;
      float uy = (A * vy - B * vx) / det;
      for (uint8_t i = 0U; i < 3U; i++)
        dq[i] = JacobianX[i] * ux + JacobianY[i] * uy;
    }
  }

  for (uint8_t i = 0U; i < 3U; i++)
  {
    float speed_deg = dq[i] * RAD_TO_DEG * direction[i];
    speed_deg = clampf(speed_deg, -Max_Servo_Speed, Max_Servo_Speed);
    servoX_angle[i] += speed_deg * CONTROL_DT;
    servoX_angle[i] = clampf(servoX_angle[i], angle_min[i], angle_max[i]);
  }
}

static void StopJoystickAndHold(void)
{
  bluetoothCMD.joy_x = 0;
  bluetoothCMD.joy_y = 0;
  bluetoothCMD.joy_turn = 0;
  bluetoothCMD.enable = 0U;

  for (uint8_t i = 0U; i < SERVO_COUNT; i++)
    servo_target_deg[i] = servo_current_deg[i];

  joystick_control_initialized = 0U;
}

/* Parse the exact joystick packet used by the corrected second file. */
static void Bluetooth_ParseJoystickCommand(char *data)
{
  int x, y, turn, enable, used = 0;

  if (sscanf(data, "J,%d,%d,%d,%d%n", &x, &y, &turn, &enable, &used) != 4 ||
      used == 0 || data[used] != '\0' ||
      x < -100 || x > 100 || y < -100 || y > 100 ||
      turn < -100 || turn > 100 || (enable != 0 && enable != 1))
    return;

  /* Require a neutral/released packet before enabling manual joystick motion. */
  if (enable == 0 && x >= -BT_DEADZONE && x <= BT_DEADZONE &&
      y >= -BT_DEADZONE && y <= BT_DEADZONE &&
      turn >= -BT_DEADZONE && turn <= BT_DEADZONE)
    bluetooth_armed = 1U;

  bluetoothCMD.joy_x = (int8_t)x;
  bluetoothCMD.joy_y = (int8_t)y;
  bluetoothCMD.joy_turn = (int8_t)turn;
  bluetoothCMD.enable = (uint8_t)(enable && bluetooth_armed);
  bluetoothCMD.last_rx_ms = HAL_GetTick();

  /* Releasing the enable switch freezes the current commanded pose. */
  if (!bluetoothCMD.enable)
  {
    for (uint8_t i = 0U; i < SERVO_COUNT; i++)
      servo_target_deg[i] = servo_current_deg[i];
    joystick_control_initialized = 0U;
  }
}

/*
 * 20-ms joystick update:
 *   X/Y -> Jacobian motion of J2/J3/J4 in the arm's local plane.
 *   turn -> J1 base rotation about world Z (yaw), not linear Z translation.
 */
static void Bluetooth_Update20ms(void)
{
  uint32_t now = HAL_GetTick();
  float x, y, turn;
  float vx, vy;
  uint8_t need_xy;
  uint8_t need_yaw;

  if (!bluetoothCMD.enable)
    return;

  /* Lost joystick packets: freeze the current pose rather than extrapolating. */
  if ((uint32_t)(now - bluetoothCMD.last_rx_ms) > BT_TIMEOUT_MS)
  {
    StopJoystickAndHold();
    bluetooth_armed = 0U;
    Bluetooth_Send("ERR joystick timeout; holding current position\r\n");
    return;
  }

  x = (float)bluetoothCMD.joy_x / 100.0f;
  y = (float)bluetoothCMD.joy_y / 100.0f;
  turn = (float)bluetoothCMD.joy_turn / 100.0f;

  if (fabsf(x) < 0.08f) x = 0.0f;
  if (fabsf(y) < 0.08f) y = 0.0f;
  if (fabsf(turn) < 0.08f) turn = 0.0f;

  vx = x * MAX_X_SPEED;
  vy = y * MAX_Y_SPEED;
  need_xy = (uint8_t)((vx != 0.0f) || (vy != 0.0f));
  need_yaw = (uint8_t)(turn != 0.0f);

  if (!need_xy && !need_yaw)
    return;

  /* Resynchronise the model after a manual joint/CSV command. */
  if (!joystick_control_initialized)
  {
    /* The source Jacobian model is calibrated for 45..135 degrees on J2-J4. */
    if (servo_current_deg[SERVO_XY_JOINT_1] < 45U ||
        servo_current_deg[SERVO_XY_JOINT_1] > 135U ||
        servo_current_deg[SERVO_XY_JOINT_2] < 45U ||
        servo_current_deg[SERVO_XY_JOINT_2] > 135U ||
        servo_current_deg[SERVO_XY_JOINT_3] < 45U ||
        servo_current_deg[SERVO_XY_JOINT_3] > 135U)
    {
      bluetoothCMD.enable = 0U;
      bluetooth_armed = 0U;
      Bluetooth_Send("ERR set J2-J4 within 45..135 deg before joystick mode\r\n");
      return;
    }

    base_yaw_angle = (float)servo_current_deg[SERVO_BASE_YAW];
    servoX_angle[0] = (float)servo_current_deg[SERVO_XY_JOINT_1];
    servoX_angle[1] = (float)servo_current_deg[SERVO_XY_JOINT_2];
    servoX_angle[2] = (float)servo_current_deg[SERVO_XY_JOINT_3];
    joystick_control_initialized = 1U;
  }

  if (need_xy)
  {
    Caculate(vx, vy);
    if (!Servo_Enable(SERVO_XY_JOINT_1) ||
        !Servo_Enable(SERVO_XY_JOINT_2) ||
        !Servo_Enable(SERVO_XY_JOINT_3))
    {
      StopJoystickAndHold();
      Bluetooth_Send("ERR PWM start failed for XY joints\r\n");
      return;
    }

    servo_target_deg[SERVO_XY_JOINT_1] = (uint8_t)lroundf(servoX_angle[0]);
    servo_target_deg[SERVO_XY_JOINT_2] = (uint8_t)lroundf(servoX_angle[1]);
    servo_target_deg[SERVO_XY_JOINT_3] = (uint8_t)lroundf(servoX_angle[2]);
  }

  if (need_yaw)
  {
    float yaw_rate_dps = turn * MAX_Z_TURN_SPEED_DPS;
    base_yaw_angle += yaw_rate_dps * CONTROL_DT;
    base_yaw_angle = clampf(base_yaw_angle,
                             (float)servo_min_deg[SERVO_BASE_YAW],
                             (float)servo_max_deg[SERVO_BASE_YAW]);

    if (!Servo_Enable(SERVO_BASE_YAW))
    {
      StopJoystickAndHold();
      Bluetooth_Send("ERR PWM start failed for base yaw\r\n");
      return;
    }
    servo_target_deg[SERVO_BASE_YAW] = (uint8_t)lroundf(base_yaw_angle);
  }
}

/* Six-angle command parser from the second file, applied atomically. */
static uint8_t Servo_SetAllTargets(const float angles[SERVO_COUNT])
{
  uint8_t new_targets[SERVO_COUNT];

  for (uint8_t i = 0U; i < SERVO_COUNT; i++)
  {
    if (!isfinite(angles[i]) ||
        angles[i] < (float)servo_min_deg[i] ||
        angles[i] > (float)servo_max_deg[i])
      return 0U;
    new_targets[i] = (uint8_t)lroundf(angles[i]);
  }

  for (uint8_t i = 0U; i < SERVO_COUNT; i++)
  {
    if (!Servo_Enable(i))
      return 0U;
  }

  StopJoystickAndHold();
  for (uint8_t i = 0U; i < SERVO_COUNT; i++)
    servo_target_deg[i] = new_targets[i];

  bluetooth_armed = 0U;
  return 1U;
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
  char reply[96];
  char *p = SkipSpaces(line);
  size_t len = strlen(p);

  while (len > 0U && (p[len - 1U] == ' ' || p[len - 1U] == '\t'))
    p[--len] = '\0';
  if (len == 0U)
    return;

  /* Joystick stream: J,x,y,turn,enable. Do not ACK every high-rate packet. */
  if ((p[0] == 'J' || p[0] == 'j') && p[1] == ',')
  {
    Bluetooth_ParseJoystickCommand(p);
    return;
  }

  if (strcmp(p, "HELP") == 0 || strcmp(p, "help") == 0)
  {
    Bluetooth_Send("Joystick: J,x,y,turn,enable (-100..100)\r\n");
    Bluetooth_Send("Release first: J,0,0,0,0 then enable=1\r\n");
    Bluetooth_Send("XY=planar Jacobian; turn=base yaw about Z\r\n");
    Bluetooth_Send("J1 90 / J1 +10 / J1 -10 : single joint\r\n");
    Bluetooth_Send("STATUS | STOP | OFF 1 | OFF ALL | PING\r\n");
    Bluetooth_Send("Six angles CSV is accepted on USART2 at 115200\r\n");
    return;
  }

  if (strcmp(p, "PING") == 0 || strcmp(p, "ping") == 0)
  {
    Bluetooth_Send("PONG\r\n");
    return;
  }

  if (strcmp(p, "STOP") == 0 || strcmp(p, "stop") == 0)
  {
    StopJoystickAndHold();
    bluetooth_armed = 0U;
    Bluetooth_Send("OK STOP (holding current position)\r\n");
    return;
  }

  if (strcmp(p, "STATUS") == 0 || strcmp(p, "status") == 0)
  {
    for (uint8_t i = 0U; i < SERVO_COUNT; i++)
    {
      snprintf(reply, sizeof(reply), "J%u: %u -> %u, %s\r\n",
               (unsigned)(i + 1U), (unsigned)servo_current_deg[i],
               (unsigned)servo_target_deg[i], servo_enabled[i] ? "ON" : "OFF");
      Bluetooth_Send(reply);
    }
    return;
  }

  if (strncmp(p, "OFF ", 4U) == 0 || strncmp(p, "off ", 4U) == 0)
  {
    char *arg = SkipSpaces(p + 4);
    StopJoystickAndHold();
    bluetooth_armed = 0U;

    if (strcmp(arg, "ALL") == 0 || strcmp(arg, "all") == 0)
    {
      for (uint8_t i = 0U; i < SERVO_COUNT; i++)
      {
        if (servo_enabled[i])
          HAL_TIM_PWM_Stop(servo_timers[i], servo_channels[i]);
        servo_enabled[i] = 0U;
        servo_current_deg[i] = SERVO_NEUTRAL_DEG;
        servo_target_deg[i] = SERVO_NEUTRAL_DEG;
      }
      base_yaw_angle = 90.0f;
      servoX_angle[0] = 90.0f;
      servoX_angle[1] = 90.0f;
      servoX_angle[2] = 90.0f;
      Bluetooth_Send("OK OFF ALL (no holding torque)\r\n");
      return;
    }

    if (arg[0] >= '1' && arg[0] <= '6' && arg[1] == '\0')
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

  /* Optional six-angle CSV command over Bluetooth as well as USART2. */
  {
    float angles[SERVO_COUNT];
    int used = 0;
    if (sscanf(p, "%f,%f,%f,%f,%f,%f %n",
               &angles[0], &angles[1], &angles[2],
               &angles[3], &angles[4], &angles[5], &used) == 6 &&
        used > 0 && p[used] == '\0')
    {
      if (Servo_SetAllTargets(angles))
        Bluetooth_Send("OK six joint targets accepted\r\n");
      else
        Bluetooth_Send("ERR six-angle packet outside servo limits\r\n");
      return;
    }
  }

  /* Single-joint command retained from the first file. */
  if ((p[0] == 'J' || p[0] == 'j') &&
      p[1] >= '1' && p[1] <= '6' &&
      (p[2] == ' ' || p[2] == '\t'))
  {
    uint8_t i = (uint8_t)(p[1] - '1');
    char *value_text = SkipSpaces(p + 3);
    int sign = 1;
    uint8_t relative = 0U;

    if (*value_text == '+' || *value_text == '-')
    {
      relative = 1U;
      if (*value_text == '-') sign = -1;
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

    StopJoystickAndHold();
    bluetooth_armed = 0U;
    if (!Servo_Enable(i))
    {
      Bluetooth_Send("ERR PWM start failed\r\n");
      return;
    }
    for (uint8_t j = 0U; j < SERVO_COUNT; j++)
      if (j != i) servo_target_deg[j] = servo_current_deg[j];
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
  /*
   * UART reception itself is interrupt-driven.
   * The main loop parses a complete line here, outside the ISR.
   */
  if (bluetooth_line_ready)
  {
    Bluetooth_FinishLine();

    /*
     * Re-arm the line buffer only after the command has been processed.
     * The callback ignores incoming bytes while bluetooth_line_ready == 1.
     */
    bluetooth_line_length = 0U;
    bluetooth_line_overflow = 0U;
    bluetooth_line_ready = 0U;
  }
}

/* Parse one six-angle line received through USART2 (115200 baud). */
static void Serial_ProcessRx(void)
{
  char local_line[SERIAL_LINE_SIZE];
  float angles[SERVO_COUNT];
  int used = 0;
  uint8_t ready;
  uint8_t overflow;

  __disable_irq();
  ready = serial_line_ready;
  overflow = serial_line_overflow;
  if (ready)
  {
    memcpy(local_line, serial_line, (size_t)serial_line_length + 1U);
    serial_line_ready = 0U;
    serial_line_overflow = 0U;
    serial_line_length = 0U;
  }
  __enable_irq();

  if (!ready)
    return;
  if (overflow)
  {
    const char msg[] = "ERR USART2 CSV line too long\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), 100U);
    return;
  }

  if (sscanf(local_line, "%f,%f,%f,%f,%f,%f %n",
             &angles[0], &angles[1], &angles[2],
             &angles[3], &angles[4], &angles[5], &used) == 6 &&
      used > 0 && local_line[used] == '\0' && Servo_SetAllTargets(angles))
  {
    const char msg[] = "OK six targets accepted\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), 100U);
  }
  else
  {
    const char msg[] = "ERR CSV format/range; expected six valid angles\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), 100U);
  }
}

/*
 * USART1 RX interrupt callback: collect one command line from HC-06.
 * Commands are terminated by CR or LF. Parsing and servo control are
 * deliberately kept in the main loop, not inside this interrupt.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (!bluetooth_line_ready)
    {
      uint8_t ch = bluetooth_rx_byte;
      if (ch == '\r' || ch == '\n')
      {
        if (bluetooth_line_length > 0U || bluetooth_line_overflow)
        {
          bluetooth_line[bluetooth_line_length] = '\0';
          bluetooth_line_ready = 1U;
        }
      }
      else if (!bluetooth_line_overflow && ch >= 32U && ch <= 126U)
      {
        if (bluetooth_line_length < (BLUETOOTH_LINE_SIZE - 1U))
          bluetooth_line[bluetooth_line_length++] = (char)ch;
        else
          bluetooth_line_overflow = 1U;
      }
    }
    HAL_UART_Receive_IT(&huart1, &bluetooth_rx_byte, 1U);
  }
  else if (huart->Instance == USART2)
  {
    if (!serial_line_ready)
    {
      uint8_t ch = serial_rx_byte;
      if (ch == '\r' || ch == '\n')
      {
        if (serial_line_length > 0U || serial_line_overflow)
        {
          serial_line[serial_line_length] = '\0';
          serial_line_ready = 1U;
        }
      }
      else if (!serial_line_overflow && ch >= 32U && ch <= 126U)
      {
        if (serial_line_length < (SERIAL_LINE_SIZE - 1U))
          serial_line[serial_line_length++] = (char)ch;
        else
          serial_line_overflow = 1U;
      }
    }
    HAL_UART_Receive_IT(&huart2, &serial_rx_byte, 1U);
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
  /* Do not start PWM at boot; enable each output when its first command arrives. */
  servo_last_update = HAL_GetTick();
  last_bt_update_ms = servo_last_update;

  if (HAL_UART_Receive_IT(&huart1, &bluetooth_rx_byte, 1U) != HAL_OK)
    Error_Handler();
  if (HAL_UART_Receive_IT(&huart2, &serial_rx_byte, 1U) != HAL_OK)
    Error_Handler();

  Bluetooth_Send("RobotArm ready. HC-06 joystick J,x,y,turn,enable; HELP for details\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    Bluetooth_ProcessRx();
    Serial_ProcessRx();

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_bt_update_ms) >= BT_PERIOD_MS)
    {
      last_bt_update_ms = now;
      Bluetooth_Update20ms();
    }

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
  huart1.Init.BaudRate = BLUETOOTH_BAUDRATE;
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
