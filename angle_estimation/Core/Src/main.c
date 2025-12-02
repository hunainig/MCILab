/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>   // ✅ abs(), labs()

#define RAD_TO_DEG 57.2957795f

// ================= CONTROL LOOP =================
#define CTRL_HZ     100.0f
#define DT_SEC      (1.0f/CTRL_HZ)     // 0.01s at 100Hz

// Complementary filter
#define ALPHA       0.98f              // 0.95–0.99 typical

// ================= AXIS / SIGN CONFIG =================
#define USE_PITCH_AXIS_Y   0   // 1 for AY-based pitch, 0 for AX-based pitch

// Flip angle direction if needed:
#define ANGLE_SIGN         (1.0f)

// Motor sign: flip if u>0 makes it fall more
#define MOTOR_SIGN         (-1.0f)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ---- Motor pins ----
// DIR1 = PA9, DIR2 = PA10
#define DIR1_PORT GPIOA
#define DIR1_PIN  GPIO_PIN_9
#define DIR2_PORT GPIOA
#define DIR2_PIN  GPIO_PIN_10

// PWM uses TIM3 CH1
#define PWM_TIMER   htim3
#define PWM_CH      TIM_CHANNEL_1
#define PWM_MAX     999   // TIM3 period = 999

// ---- Motor B pins ----
// DIR3 = PC8, DIR4 = PC9
#define DIR3_PORT GPIOC
#define DIR3_PIN  GPIO_PIN_8
#define DIR4_PORT GPIOC
#define DIR4_PIN  GPIO_PIN_9

// PWM B uses TIM3 CH2
#define PWM_CH_B  TIM_CHANNEL_2

static inline void Motors_SetDirection(int dir);
static inline void Motors_SetPWM(uint16_t pwm);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* -------- LSM303AGR (Accel over I2C) -------- */
#define LSM_A_ADDR_W      0x32
#define LSM_A_ADDR_R      0x33
#define LSM_A_CTRL1       0x20
#define LSM_A_CTRL4       0x23
#define LSM_A_OUT_X_L     0x28
#define LSM_A_AUTO_INC    0x80
#define LSM_A_G_PER_LSB   0.0039f   // g/LSB

/* -------- L3GD20 style Gyro (SPI) -------- */
#define GYRO_REG_WHOAMI    0x0F
#define GYRO_REG_CTRL1     0x20
#define GYRO_REG_CTRL4     0x23
#define GYRO_REG_OUT_X_L   0x28
#define GYRO_SPI_READ      0x80
#define GYRO_SPI_AUTO_INC  0x40

#define GYRO_CS_LOW()   HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_SET)

#define GYRO_DPS_PER_LSB  0.00875f  // ±250 dps scale

typedef struct {
    // raw values
    int16_t arx, ary, arz;
    int16_t grx, gry, grz;

    // converted values
    float ax, ay, az;   // g
    float gx, gy, gz;   // dps

    // calibrated
    float ax_off, ay_off, az_off;
    float gx_off, gy_off, gz_off;
} imu_t;

volatile uint8_t doCtrl = 0;   // set in TIM2 ISR @100Hz

static imu_t imu;

// ===== PID state =====
static float prev_angle = 0.0f; // previous angle for derivative calculation

// ===== Gains (tune) =====
static float Kp = 18.0f; // proportional gain
static float Ki = 0.6f; // integral gain
static float Kd = 1.2f;  // derivative gain

// scale to PWM
#define U_SCALE  30.0f 

static float angle_deg = 0.0f; 
static float accel_angle = 0.0f; // from accelerometer
static float gyro_rate = 0.0f;  // angular rate from gyro

static float pid_integral = 0.0f; // integral term
static float last_error = 0.0f; // for integral windup check

// Upright setpoint (deg) — tune mechanical offset
static float setpoint_deg = 0.0f; // target angle
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef LSM_Accel_Init(void);
HAL_StatusTypeDef LSM_Accel_Read(imu_t *m);

uint8_t gyro_read_u8(uint8_t reg);
void    gyro_write_u8(uint8_t reg, uint8_t val);
void    gyro_init_basic(void);
HAL_StatusTypeDef Gyro_Read(imu_t *m);

void Offset_Calibrate(imu_t *m);

int _write(int file, char *data, int len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ---------- LSM303AGR accel ----------
HAL_StatusTypeDef LSM_Accel_Init(void)
{
    uint8_t v;

    v = 0x67;  // 200Hz, XYZ ON
    if (HAL_I2C_Mem_Write(&hi2c1, LSM_A_ADDR_W, LSM_A_CTRL1,
                         I2C_MEMADD_SIZE_8BIT, &v, 1, 100) != HAL_OK)
        return HAL_ERROR;

    v = 0x00;  // ±2g
    if (HAL_I2C_Mem_Write(&hi2c1, LSM_A_ADDR_W, LSM_A_CTRL4,
                         I2C_MEMADD_SIZE_8BIT, &v, 1, 100) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(10);
    return HAL_OK;
}

HAL_StatusTypeDef LSM_Accel_Read(imu_t *m)
{
    uint8_t buf[6];

    if (HAL_I2C_Mem_Read(&hi2c1, LSM_A_ADDR_R, (LSM_A_OUT_X_L | LSM_A_AUTO_INC),
                         I2C_MEMADD_SIZE_8BIT, buf, 6, 100) != HAL_OK)
        return HAL_ERROR;

    m->arx = (int16_t)((buf[1] << 8) | buf[0]);
    m->ary = (int16_t)((buf[3] << 8) | buf[2]);
    m->arz = (int16_t)((buf[5] << 8) | buf[4]);

    // 10-bit left-justified -> >>6
    int16_t rx = m->arx >> 6;
    int16_t ry = m->ary >> 6;
    int16_t rz = m->arz >> 6;

    m->ax = rx * LSM_A_G_PER_LSB - m->ax_off;
    m->ay = ry * LSM_A_G_PER_LSB - m->ay_off;
    m->az = rz * LSM_A_G_PER_LSB - m->az_off;

    return HAL_OK;
}

// ---------- Gyro helpers ----------
uint8_t gyro_read_u8(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | GYRO_SPI_READ), 0x00 };
    uint8_t rx[2] = {0};

    GYRO_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    GYRO_CS_HIGH();

    return rx[1];
}

void gyro_write_u8(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };

    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    GYRO_CS_HIGH();
}

void gyro_init_basic(void)
{
    gyro_write_u8(GYRO_REG_CTRL1, 0x0F);  // power on + XYZ enable
    gyro_write_u8(GYRO_REG_CTRL4, 0x00);  // ±245 dps
    HAL_Delay(50);
}

HAL_StatusTypeDef Gyro_Read(imu_t *m)
{
    uint8_t reg = GYRO_REG_OUT_X_L | GYRO_SPI_READ | GYRO_SPI_AUTO_INC;
    uint8_t buf[6];

    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
    HAL_SPI_Receive(&hspi1, buf, 6, 100);
    GYRO_CS_HIGH();

    m->grx = (int16_t)(buf[1]<<8 | buf[0]);
    m->gry = (int16_t)(buf[3]<<8 | buf[2]);
    m->grz = (int16_t)(buf[5]<<8 | buf[4]);

    m->gx = m->grx * GYRO_DPS_PER_LSB - m->gx_off;
    m->gy = m->gry * GYRO_DPS_PER_LSB - m->gy_off;
    m->gz = m->grz * GYRO_DPS_PER_LSB - m->gz_off;

    return HAL_OK;
}

// ---------- Offset calibration ----------
void Offset_Calibrate(imu_t *m)
{
    const int N = 200;   // ✅ more samples = more stable offsets
    float sax=0, say=0, saz=0;
    float sgx=0, sgy=0, sgz=0;

    m->ax_off = m->ay_off = m->az_off = 0;
    m->gx_off = m->gy_off = m->gz_off = 0;

    for (int i=0; i<N; i++)
    {
        LSM_Accel_Read(m);
        Gyro_Read(m);

        sax += m->ax; say += m->ay; saz += m->az;
        sgx += m->gx; sgy += m->gy; sgz += m->gz;

        HAL_Delay(5);
    }

    m->ax_off = sax / N;
    m->ay_off = say / N;
    m->az_off = (saz / N) - 1.0f;  // remove gravity

    m->gx_off = sgx / N;
    m->gy_off = sgy / N;
    m->gz_off = sgz / N;
}

// ---------- printf retarget ----------
int _write(int file, char *data, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
}

// ---------- Motor helpers ----------
static inline void Motors_SetDirection(int dir)
{
    if (dir >= 0) {
        HAL_GPIO_WritePin(DIR1_PORT, DIR1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR2_PORT, DIR2_PIN, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(DIR3_PORT, DIR3_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR4_PORT, DIR4_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(DIR1_PORT, DIR1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR2_PORT, DIR2_PIN, GPIO_PIN_SET);

        HAL_GPIO_WritePin(DIR3_PORT, DIR3_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR4_PORT, DIR4_PIN, GPIO_PIN_SET);
    }
}

static inline void Motors_SetPWM(uint16_t pwm)
{
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    __HAL_TIM_SET_COMPARE(&htim3, PWM_CH, pwm);
    __HAL_TIM_SET_COMPARE(&htim3, PWM_CH_B, pwm);
}

static void Calibrate_Setpoint(void)
{
    printf("Hold robot upright & still for setpoint...\r\n");

    float sp_sum = 0.0f;
    const int SP_N = 100; // 1 sec at 100Hz

    for (int i = 0; i < SP_N; i++)
    {
        LSM_Accel_Read(&imu);
        Gyro_Read(&imu);

    #if USE_PITCH_AXIS_Y
        float acc = atan2f(imu.ay, imu.az) * RAD_TO_DEG;
        float gyr = imu.gy;
    #else
        float acc = atan2f(imu.ax, imu.az) * RAD_TO_DEG;
        float gyr = imu.gx;
    #endif
        acc *= ANGLE_SIGN;
        gyr *= ANGLE_SIGN;

        if (i == 0) angle_deg = acc;

        angle_deg = ALPHA * (angle_deg + gyr * DT_SEC)
                  + (1.0f - ALPHA) * acc;

        sp_sum += angle_deg;
        HAL_Delay(10);
    }

    setpoint_deg = sp_sum / SP_N;
    printf("Setpoint = %.2f deg\r\n", setpoint_deg);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void)
{
   HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);   // ✅ 100 Hz control ticker

  // init accel
  if (HAL_I2C_IsDeviceReady(&hi2c1, LSM_A_ADDR_W, 3, 50) == HAL_OK) {
      if (LSM_Accel_Init() == HAL_OK)
          printf("LSM accel init OK\r\n");
      else
          printf("LSM accel init FAIL\r\n");
  } else {
      printf("LSM accel not found on I2C\r\n");
  }

  // init gyro
  gyro_init_basic();
  uint8_t who = gyro_read_u8(GYRO_REG_WHOAMI);
  printf("GYRO WHOAMI=0x%02X\r\n", who);

  // offset calibration (keep board still!)
  printf("Calibrating offsets...\r\n");
  Offset_Calibrate(&imu);
  printf("Done.\r\n");

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  
  Calibrate_Setpoint();
  // seed previous angle
  prev_angle = 0.0f;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    char buf[80];
        int n = snprintf(buf, sizeof(buf),
                         "%.3f,%.3f,%.3f\r\n",
                         angle_deg,
                         imu.gy,
                         angle_deg);

        HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);   // terminal
  }
  /* USER CODE END WHILE */
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
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
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 5999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 79;
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

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;

    // ---------------- 1) READ IMU ----------------
    if (LSM_Accel_Read(&imu) != HAL_OK) return;
    if (Gyro_Read(&imu) != HAL_OK) return;

    // accel angle (same axis choice as your code)
#if USE_PITCH_AXIS_Ywhile
    accel_angle = atan2f(imu.ay, imu.az) * RAD_TO_DEG;
    gyro_rate   = imu.gy;   // dps
#else
    accel_angle = atan2f(imu.ax, imu.az) * RAD_TO_DEG;
    gyro_rate   = imu.gx;
#endif
    accel_angle *= ANGLE_SIGN;
    gyro_rate   *= ANGLE_SIGN;

    // init filter once
    // (helps reduce initial spike)
    static uint8_t first_init = 1;
    if (first_init) {
        angle_deg = accel_angle;
        first_init = 0;
    }

    // ---------------- 2) COMPLEMENTARY FILTER ----------------
    // fuse accel & gyro to get angle
    angle_deg = ALPHA * (angle_deg + gyro_rate * DT_SEC)
              + (1.0f - ALPHA) * accel_angle;

    // ---------------- 3) ERROR ----------------
    // computes how far the angle is from setpoint
    float error = setpoint_deg - angle_deg;

    // deadband near upright
    // helps prevent oscillation at small angles
    const float DEADBAND = 0.15f;
    if (fabsf(error) < DEADBAND) {
      error = 0.0f;
      pid_integral *= 0.995f;
    }

    // clamp error so it doesn't explode
    const float MAX_ERR = 12.0f;
    if (error >  MAX_ERR) error =  MAX_ERR;
    if (error < -MAX_ERR) error = -MAX_ERR;

    // ---------------- 4) PID ----------------
    // integral term with windup guard
    pid_integral += error * DT_SEC;
    if (pid_integral >  20.0f) pid_integral =  20.0f;
    if (pid_integral < -20.0f) pid_integral = -20.0f;

    // derivative term helps damping
    float derivative = (error - last_error) / DT_SEC;
    last_error = error;

    // control signal
    float u = (Kp*error + Ki*pid_integral + Kd*derivative) * U_SCALE;

    // motor sign
    u *= MOTOR_SIGN;
    // ---------------- 5) DRIVE MOTORS ----------------
    // clamp to PWM limits (0-999)
    if (u >  PWM_MAX) u =  PWM_MAX;
    if (u < -PWM_MAX) u = -PWM_MAX;

    // absolute value for PWM
    float u_abs = fabsf(u);

    // stronger kick helps a lot
    const float PWM_MIN = 260.0f;
    if (u_abs > 0 && u_abs < PWM_MIN) u_abs = PWM_MIN;

    // set motor direction & PWM
    Motors_SetDirection(u >= 0 ? 1 : -1);
    Motors_SetPWM((uint16_t)u_abs);

    // 5) Bluetooth stream
      }
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */