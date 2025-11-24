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
#include <string.h>
#include <stdio.h>
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t do10Hz = 0;        // set in ISR, consumed in while(1)

/* -------- LSM303AGR (Accel over I2C) -------- */
#define LSM_A_ADDR_W      0x32   // 8-bit write addr
#define LSM_A_ADDR_R      0x33   // 8-bit read addr
#define LSM_A_CTRL1       0x20
#define LSM_A_CTRL4       0x23
#define LSM_A_OUT_X_L     0x28
#define LSM_A_AUTO_INC    0x80
#define LSM_A_G_PER_LSB   0.0039f   // 3.9 mg/LSB -> g

/* -------- L3GD20 / I3G4250D style Gyro over SPI -------- */
#define GYRO_REG_WHOAMI    0x0F
#define GYRO_REG_CTRL1     0x20
#define GYRO_REG_CTRL4     0x23
#define GYRO_REG_OUT_X_L   0x28
#define GYRO_SPI_READ      0x80
#define GYRO_SPI_AUTO_INC  0x40

#define GYRO_CS_LOW()   HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_SET)

/* Sensitivity for ±250 dps full-scale (typical L3GD20) */
#define GYRO_DPS_PER_LSB  0.00875f  // 8.75 mdps/LSB

typedef struct {
    // raw
    int16_t arx, ary, arz;
    int16_t grx, gry, grz;

    // scaled (physical)
    float ax, ay, az;   // g
    float gx, gy, gz;   // dps

    // offsets
    float ax_off, ay_off, az_off;
    float gx_off, gy_off, gz_off;
} imu_t;

static imu_t imu;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* accel */
HAL_StatusTypeDef LSM_Accel_Init(void);
HAL_StatusTypeDef LSM_Accel_Read(imu_t *m);

/* gyro */
uint8_t gyro_read_u8(uint8_t reg);
void    gyro_write_u8(uint8_t reg, uint8_t val);
void    gyro_init_basic(void);
HAL_StatusTypeDef Gyro_Read(imu_t *m);

/* calibration */
void Offset_Calibrate(imu_t *m);

/* retarget printf -> UART */
int _write(int file, char *data, int len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---------- LSM303AGR accel ---------- */
HAL_StatusTypeDef LSM_Accel_Init(void)
{
    uint8_t v;

    // CTRL1_A = 0x63 per lab (ODR 100Hz, all axes ON, normal mode)
    v = 0x63;
    if (HAL_I2C_Mem_Write(&hi2c1, LSM_A_ADDR_W, LSM_A_CTRL1,
                          I2C_MEMADD_SIZE_8BIT, &v, 1, 100) != HAL_OK)
        return HAL_ERROR;

    // CTRL4_A = 0x00 per lab (±2g normal mode)
    v = 0x00;
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

    // left-justified 10-bit -> right shift by 6
    int16_t rx = m->arx >> 6;
    int16_t ry = m->ary >> 6;
    int16_t rz = m->arz >> 6;

    m->ax = rx * LSM_A_G_PER_LSB - m->ax_off;
    m->ay = ry * LSM_A_G_PER_LSB - m->ay_off;
    m->az = rz * LSM_A_G_PER_LSB - m->az_off;

    return HAL_OK;
}

/* ---------- Gyro SPI helpers ---------- */
uint8_t gyro_read_u8(uint8_t reg)
{
    uint8_t header = (reg & 0x3F) | GYRO_SPI_READ;
    uint8_t v = 0;

    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &header, 1, 100);
    HAL_SPI_Receive(&hspi1, &v, 1, 100);
    GYRO_CS_HIGH();

    return v;
}

void gyro_write_u8(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x3F), val };
    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, 100);
    GYRO_CS_HIGH();
}

void gyro_init_basic(void)
{
    // CTRL1: power on, enable XYZ, ODR ~95/100Hz (0x0F + ODR bits)
    gyro_write_u8(GYRO_REG_CTRL1, 0x0F);

    // CTRL4: ±250 dps full-scale
    gyro_write_u8(GYRO_REG_CTRL4, 0x00);

    HAL_Delay(10);
}

HAL_StatusTypeDef Gyro_Read(imu_t *m)
{
    uint8_t header = (GYRO_REG_OUT_X_L & 0x3F) | GYRO_SPI_READ | GYRO_SPI_AUTO_INC;
    uint8_t buf[6];

    GYRO_CS_LOW();
    if (HAL_SPI_Transmit(&hspi1, &header, 1, 100) != HAL_OK) {
        GYRO_CS_HIGH();
        return HAL_ERROR;
    }
    if (HAL_SPI_Receive(&hspi1, buf, 6, 100) != HAL_OK) {
        GYRO_CS_HIGH();
        return HAL_ERROR;
    }
    GYRO_CS_HIGH();

    m->grx = (int16_t)((buf[1] << 8) | buf[0]);
    m->gry = (int16_t)((buf[3] << 8) | buf[2]);
    m->grz = (int16_t)((buf[5] << 8) | buf[4]);

    m->gx = m->grx * GYRO_DPS_PER_LSB - m->gx_off;
    m->gy = m->gry * GYRO_DPS_PER_LSB - m->gy_off;
    m->gz = m->grz * GYRO_DPS_PER_LSB - m->gz_off;

    return HAL_OK;
}

/* ---------- Offset calibration ---------- */
void Offset_Calibrate(imu_t *m)
{
    const int N = 20;
    float sax=0, say=0, saz=0;
    float sgx=0, sgy=0, sgz=0;

    // ensure offsets start at 0 during calibration
    m->ax_off = m->ay_off = m->az_off = 0;
    m->gx_off = m->gy_off = m->gz_off = 0;

    for (int i=0; i<N; i++)
    {
        LSM_Accel_Read(m);
        Gyro_Read(m);

        sax += m->ax; say += m->ay; saz += m->az;
        sgx += m->gx; sgy += m->gy; sgz += m->gz;

        HAL_Delay(10);
    }

    m->ax_off = sax / N;
    m->ay_off = say / N;
    // subtract gravity so Z becomes ~0 when flat
    m->az_off = (saz / N) - 1.0f;

    m->gx_off = sgx / N;
    m->gy_off = sgy / N;
    m->gz_off = sgz / N;
}

/* ---------- printf retarget ---------- */
int _write(int file, char *data, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);   // start TIM2 in interrupt mode (100 Hz)

  // init accel
  if (HAL_I2C_IsDeviceReady(&hi2c1, LSM_A_ADDR_W, 3, 50) == HAL_OK) {
      if (LSM_Accel_Init() == HAL_OK)
          printf("LSM accel init OK\r\n");
      else
          printf("LSM accel init FAIL\r\n");
  } else {
      printf("LSM accel not found on I2C\r\n");
  }

  // init gyro (SPI)
  gyro_init_basic();
  printf("Gyro WHOAMI=0x%02X\r\n", gyro_read_u8(GYRO_REG_WHOAMI));

  // offset calibration (keep board still)
  printf("Calibrating offsets...\r\n");
  Offset_Calibrate(&imu);
  printf("Done.\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (do10Hz)
    {
        do10Hz = 0;

        if (LSM_Accel_Read(&imu) == HAL_OK && Gyro_Read(&imu) == HAL_OK)
        {
            // OPTION A: integer CSV (works without float printf)
            int ax_mg = (int)(imu.ax *1000.0f);
            int ay_mg = (int)(imu.ay *1000.0f);
            int az_mg = (int)(imu.az *1000.0f);

            int gx_mdps = (int)(imu.gx);
            int gy_mdps = (int)(imu.gy);
            int gz_mdps = (int)(imu.gz);

            char buf[128];
            int n = snprintf(buf, sizeof(buf),
                "%d,%d,%d,%d,%d,%d\r\n",
                ax_mg, ay_mg, az_mg,
                gx_mdps, gy_mdps, gz_mdps);

            HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
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

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
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
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE END TIM2_Init 1 */
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
GPIO_InitStruct.Pin = GPIO_PIN_0;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        // 100 Hz tick work (already required)
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);

        // Divide to 10 Hz using a small counter
        static uint8_t subcnt = 0;   // lives only in ISR
        if (++subcnt >= 10)          // 100 Hz / 10 = 10 Hz
        {
            subcnt = 0;
            do10Hz = 1;              // signal main loop
        }
    }
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
