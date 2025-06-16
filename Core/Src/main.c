/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Self-balancing robot – STM32F103 + MPU6050 (+ auto-calib 5 s)
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "Kalman.h"
#include "utils.h"
#include "stdlib.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- PID params --- */
#define KP      38.0f
#define KI       0.0f
#define KD      5.0f
#define I_LIM    2.5f

/* --- GPIO map --- */
#define DIR_L_F   GPIO_PIN_0
#define DIR_L_R   GPIO_PIN_1
#define DIR_R_F   GPIO_PIN_10
#define DIR_R_R   GPIO_PIN_11
#define DIR_PORT  GPIOB

/* --- PWM / giới hạn --- */
#define MAX_PWM      2700
#define DEAD_PWM     150
#define FALL_ANGLE     45.0f

#define TURN_DURATION_US 150000
#define TURN_PWM_OFFSET    80
#define PERIOD_M 3600

/* --- Auto-calib --- */
#define AUTOCALIB_MS     5000

#define LED_CALIB_PIN   GPIO_PIN_12
#define LED_CALIB_PORT  GPIOB

#define LOOP_MIN_DT_S   0.005f
#define LOOP_MAX_DT_S   0.004f

#define ANGLE_STOP_TH   0.5f
#define GYRO_STOP_TH    2.0f

#define LEFT_MOTOR_COMP  1.05f
#define RIGHT_MOTOR_COMP 1.00f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
Kalman_t kal;
volatile uint8_t  bt_rx;
volatile char     bt_cmd = 'S';
volatile uint8_t  newCmd = 0;

float  integ = 0, prevErr = 0, prevDeriv = 0;
float setPointOriginal= 0.0f * DEG2RAD;
float  setPoint         = 0.0f;

volatile uint32_t turn_active_until = 0;
volatile char     current_turn_direction = 'S';
volatile char     prev_bt_cmd = 'S';

float    angle_offset_deg = 0.0f;
uint8_t  autoCalib_done  = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void readMPU(float *pitchDeg, float *gyroRate, float dt);
void stopMotor(void);
void setIndividualMotors(int16_t left_pwm, int16_t right_pwm);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        bt_cmd = bt_rx;
        newCmd = 1;
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&bt_rx, 1);
    }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  Kalman_Init(&kal);
  MPU6050_Init();

  uint32_t  tPrev = micros();
  uint32_t  tick_start = HAL_GetTick();
  float     pitchDeg_raw = 0, gyro = 0;
  float offset_sum = 0.0f;
  uint32_t offset_cnt = 0;
  static uint32_t counter = 0;
  static uint32_t last_debug = 0;

  while (1)
  {
    uint32_t now  = micros();
    float    dt   = (now - tPrev) * 1e-6f;

    if (dt < LOOP_MIN_DT_S) continue;
    if (dt > LOOP_MAX_DT_S) dt = LOOP_MAX_DT_S;
    tPrev = now;
    counter++;

    if (HAL_GetTick() - last_debug >= 1000) {
      printf("Loop/s: %lu\r\n", counter);
      counter = 0;
      last_debug = HAL_GetTick();
    }

    readMPU(&pitchDeg_raw, &gyro, dt);

    if (!autoCalib_done && HAL_GetTick() - tick_start < AUTOCALIB_MS) {
      HAL_GPIO_WritePin(LED_CALIB_PORT, LED_CALIB_PIN, GPIO_PIN_SET);
      stopMotor();
      offset_sum += pitchDeg_raw;
      offset_cnt++;
      continue;
    }
    else if (!autoCalib_done) {
      angle_offset_deg = offset_sum / (float)offset_cnt;
      autoCalib_done   = 1;
      integ = 0.0f;
      setPoint = setPointOriginal ;
      HAL_GPIO_WritePin(LED_CALIB_PORT, LED_CALIB_PIN, GPIO_PIN_RESET);
    }

    float pitchDeg = pitchDeg_raw - angle_offset_deg;

    if (fabsf(pitchDeg) > FALL_ANGLE) {
      stopMotor();
      integ = prevErr = prevDeriv = 0;
      current_turn_direction = 'S';
      setPoint = setPointOriginal;
      prev_bt_cmd = 'S';
      continue;
    }

    float theta = pitchDeg * DEG2RAD;
    float omega = -gyro * DEG2RAD;
    float err   = setPoint - theta;

    float up = KP * err;
    float ud = KD * omega;
    integ += err * dt;
    if (integ >  I_LIM) integ =  I_LIM;
    if (integ < -I_LIM) integ = -I_LIM;
    float ui = KI * integ;

    float out = up + ud + ui;

    int16_t pwm_raw = (int16_t)(out * 1000.0f);
    int16_t sign    = (pwm_raw >= 0) ?  1 : -1;
    int16_t mag     = abs(pwm_raw);

    if (mag < DEAD_PWM) {
      if (fabsf(pitchDeg) < ANGLE_STOP_TH && fabsf(gyro) < GYRO_STOP_TH) {
        mag = 0;
      } else {
        mag = DEAD_PWM;
      }
    }

    int16_t pwm = sign * mag;
    if (pwm >  MAX_PWM) pwm =  MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;

    int16_t left_pwm  = pwm;
    int16_t right_pwm = pwm;

    if (current_turn_direction != 'S' && now < turn_active_until) {
      if (current_turn_direction == 'L') {
        left_pwm  = pwm - TURN_PWM_OFFSET;
        right_pwm = pwm + TURN_PWM_OFFSET;
      } else {
        left_pwm  = pwm + TURN_PWM_OFFSET;
        right_pwm = pwm - TURN_PWM_OFFSET;
      }
    } else {
      current_turn_direction = 'S';
      prev_bt_cmd = 'S';
    }

    left_pwm  = (int16_t)(left_pwm * LEFT_MOTOR_COMP);
    right_pwm = (int16_t)(right_pwm * RIGHT_MOTOR_COMP);

    left_pwm  = fminf(fmaxf(left_pwm,  -MAX_PWM), MAX_PWM);
    right_pwm = fminf(fmaxf(right_pwm, -MAX_PWM), MAX_PWM);

    setIndividualMotors(left_pwm, right_pwm);

    //printf("Pitch: %.2f | Gyro: %.2f | Err: %.2f | PWM: %d | L: %d | R: %d\r\n",
      //     pitchDeg, gyro, err, pwm, left_pwm, right_pwm);


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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3600;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11
                           PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ---------------- MPU6050 ---------------- */
void MPU6050_Init(void)
{
    uint8_t d = 0;
    // Reset device - recommended by datasheet when changing power settings
    // HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, 1, &(uint8_t){0x80}, 1, HAL_MAX_DELAY); // Reset MPU6050
    // HAL_Delay(100);

    HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, 1, &d, 1, HAL_MAX_DELAY);
    HAL_Delay(100);

    // Configure MPU6050 registers explicitly as in Arduino example comments
    // SMPLRT_DIV (0x19): Set sample rate to 1000Hz (8kHz / (7+1))
    HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x19, 1, &(uint8_t){0x07}, 1, HAL_MAX_DELAY);
    // CONFIG (0x1A): Disable FSYNC, DLPF_CFG = 0 (256Hz Gyro, 260Hz Accel bandwidth)
    HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x1A, 1, &(uint8_t){0x00}, 1, HAL_MAX_DELAY);
    // GYRO_CONFIG (0x1B): FS_SEL = 0 (+/- 250 deg/s)
    HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x1B, 1, &(uint8_t){0x00}, 1, HAL_MAX_DELAY);
    // ACCEL_CONFIG (0x1C): AFS_SEL = 0 (+/- 2g)
    HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x1C, 1, &(uint8_t){0x00}, 1, HAL_MAX_DELAY);

    /* lấy 200 mẫu avg để init Kalman.angle */
    int32_t sum = 0;
    for (int i = 0; i < 200; i++) {
        uint8_t buf[6];
        if (HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0x3B, 1, buf, 6, HAL_MAX_DELAY) != HAL_OK)
            continue;
        int16_t ax = (buf[0]<<8)|buf[1];
        int16_t az = (buf[4]<<8)|buf[5];
        float pitchAcc = atan2f(-ax, az) * 57.2958f;
        sum += pitchAcc;
        HAL_Delay(2);
    }
    kal.angle = sum / 200.0f;

    /* gyro X offset */
    int32_t gsum = 0;
    for (int i = 0; i < 1000; i++) {
        uint8_t buf[14];
        if (HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0x3B, 1, buf, 14, HAL_MAX_DELAY) != HAL_OK)
            continue;
        int16_t gx = (buf[8]<<8)|buf[9];
        gsum += gx;
        HAL_Delay(1);
    }
    kal.bias = (float)gsum / 1000.0f / 131.0f;   /* bias lưu luôn vào Kalman */
}

void readMPU(float *pitchDeg, float *gyroRate, float dt)
{
    uint8_t b[14];
    if (HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0x3B, 1, b, 14, HAL_MAX_DELAY) != HAL_OK)
        return;
    int16_t ax = (b[0]<<8)|b[1];
    int16_t az = (b[4]<<8)|b[5];
    int16_t gx = (b[8]<<8)|b[9];

    *gyroRate = (gx / 131.0f);  // bias is compensated inside Kalman_GetAngle
    float pitchAcc = atan2f(-ax, az) * 57.2958f; // atan(-ax/az)
    *pitchDeg = Kalman_GetAngle(&kal, pitchAcc, *gyroRate, dt);
}

/* --------------- Motor helpers --------------- */
void stopMotor(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    HAL_GPIO_WritePin(DIR_PORT, DIR_L_F|DIR_L_R|DIR_R_F|DIR_R_R, GPIO_PIN_RESET);
}

void setIndividualMotors(int16_t left_pwm, int16_t right_pwm)
{
    /* Left direction */
    int16_t lsign = (left_pwm >= 0) ? 1 : -1;
    if (lsign < 0) left_pwm = -left_pwm;

    HAL_GPIO_WritePin(DIR_PORT, DIR_L_F, (lsign > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_PORT, DIR_L_R, (lsign > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    if (left_pwm == 0) HAL_GPIO_WritePin(DIR_PORT, DIR_L_F|DIR_L_R, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_pwm);

    int16_t rsign = (right_pwm >= 0) ? 1 : -1;
    if (rsign < 0) right_pwm = -right_pwm;

    HAL_GPIO_WritePin(DIR_PORT, DIR_R_F, (rsign > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIR_PORT, DIR_R_R, (rsign > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (right_pwm == 0) HAL_GPIO_WritePin(DIR_PORT, DIR_R_F|DIR_R_R, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right_pwm);
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
