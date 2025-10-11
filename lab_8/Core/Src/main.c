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
#include <string.h>  // for strlen()

// --- Gyro registers ---
// --- Axis output registers (little-endian: L then H) ---
#define I3G4250D_OUT_X_L   0x28
#define I3G4250D_OUT_X_H   0x29
#define I3G4250D_OUT_Y_L   0x2A
#define I3G4250D_OUT_Y_H   0x2B
#define I3G4250D_OUT_Z_L   0x2C
#define I3G4250D_OUT_Z_H   0x2D

// Sensitivity for ±245 dps (default FS): 8.75 mdps/LSB = 0.00875 dps/LSB
#define I3G4250D_SENS_245DPS  0.00875f

#define I3G4250D_WHO_AM_I      0x0F
#define I3G4250D_CTRL_REG1     0x20
#define I3G4250D_OUT_TEMP      0x26

// CTRL_REG1 value: PD=1, Xen=Yen=Zen=1; DR/BW left at default
#define I3G4250D_CTRL_REG1_VAL 0x0F

// --- CS pin (as you already use) ---
#define GYRO_CS_PORT GPIOE
#define GYRO_CS_PIN  CS_I2C_SPI_Pin

// pacing period (ms) for temperature reads
#define TEMP_PERIOD_MS 150


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

UART_HandleTypeDef huart1;

// SPI interrupt state
static volatile uint8_t spi_busy = 0;
static uint8_t spi_cmd = 0;
static uint8_t temp_raw = 0;
static uint32_t next_due = 0;


/* USER CODE BEGIN PV */
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline void GYRO_CS_L(void){ HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET); }
static inline void GYRO_CS_H(void){ HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET); }

static void uart_print(const char* s) {
  HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), 100);
}
static void Gyro_Init(void)
{
  uint8_t tx[2];
  tx[0] = (I3G4250D_CTRL_REG1 & 0x3F);     // write: bit7=0, bits[5:0]=addr
  tx[1] = I3G4250D_CTRL_REG1_VAL;

  GYRO_CS_L();
  HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY); // blocking ok for init
  GYRO_CS_H();

  HAL_Delay(5); // small settle time
}
static void Start_Temp_Read_IT(void)
{
  if (spi_busy) return;
  spi_busy = 1;

  // build read command: bit7=1 (read) | addr
  spi_cmd = 0x80 | (I3G4250D_OUT_TEMP & 0x3F);

  GYRO_CS_L();
  // 1) send command byte via interrupt
  if (HAL_SPI_Transmit_IT(&hspi1, &spi_cmd, 1) != HAL_OK) {
    GYRO_CS_H(); spi_busy = 0;
  }
}


static uint8_t Gyro_ReadReg(uint8_t reg)
{
  uint8_t cmd = 0x80 | (reg & 0x3F); // bit7=1 read, no auto-inc
  uint8_t val = 0;

  GYRO_CS_L();
  if (HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY) != HAL_OK) { GYRO_CS_H(); Error_Handler(); }
  if (HAL_SPI_Receive (&hspi1, &val, 1, HAL_MAX_DELAY) != HAL_OK) { GYRO_CS_H(); Error_Handler(); }
  GYRO_CS_H();

  return val;

  
}

// Read N bytes starting at 'start_addr' using auto-increment
static void Gyro_ReadMulti(uint8_t start_addr, uint8_t *buf, uint16_t len)
{
  uint8_t cmd = 0x80 | 0x40 | (start_addr & 0x3F); // READ + AUTO-INCR + addr
  GYRO_CS_L();
  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive (&hspi1, buf, len, HAL_MAX_DELAY);
  GYRO_CS_H();
}

// Do one sample: temp + XYZ (raw→dps) and print: "<temp>, <x>, <y>, <z>\n"
static void SampleAndPrint_IntCSV(void)
{
  // --- temperature (1 byte, signed) ---
  uint8_t t_u8 = 0;
  uint8_t t_cmd = 0x80 | (I3G4250D_OUT_TEMP & 0x3F);
  GYRO_CS_L();
  HAL_SPI_Transmit(&hspi1, &t_cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive (&hspi1, &t_u8, 1, HAL_MAX_DELAY);
  GYRO_CS_H();
  int8_t temp = (int8_t)t_u8;   // signed integer (OK for the script)

  // --- axes: read 6 bytes starting at OUT_X_L (auto-increment read) ---
  uint8_t raw[6];
  Gyro_ReadMulti(I3G4250D_OUT_X_L, raw, 6);

  int16_t x_raw = (int16_t)((((int16_t)raw[1]) << 8) | raw[0]);
  int16_t y_raw = (int16_t)((((int16_t)raw[3]) << 8) | raw[2]);
  int16_t z_raw = (int16_t)((((int16_t)raw[5]) << 8) | raw[4]);

  // ---- convert to mdps (integers) : mdps = round(raw * 8.75) ----
  // Use integer math with rounding: (val*875 + 50)/100  (mdps)
  int32_t x_mdps = ((int32_t)x_raw * 875 + (x_raw >= 0 ? 50 : -50)) / 100;
  int32_t y_mdps = ((int32_t)y_raw * 875 + (y_raw >= 0 ? 50 : -50)) / 100;
  int32_t z_mdps = ((int32_t)z_raw * 875 + (z_raw >= 0 ? 50 : -50)) / 100;

  // ---- CSV with NO spaces, integers only, newline at end ----
  char line[64];
  //           temp,   x_mdps, y_mdps, z_mdps
  // Example:   25,1234,-87,314\n
  int n = snprintf(line, sizeof(line), "%d,%ld,%ld,%ld\n",
                   (int)temp, (long)x_mdps, (long)y_mdps, (long)z_mdps);
  if (n > 0) uart_print(line);
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
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
HAL_Delay(10); // power-up
HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);

// optional: verify device (you already have this)
{
  uint8_t cmd = 0x80 | (I3G4250D_WHO_AM_I & 0x3F);
  uint8_t who = 0;
  GYRO_CS_L();
  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive (&hspi1, &who, 1, HAL_MAX_DELAY);
  GYRO_CS_H();
  // do not print here (Python expects strict CSV)
}


Gyro_Init();              // power on, enable axes
next_due = HAL_GetTick(); // start immediately
/* USER CODE END 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE BEGIN WHILE */
/* USER CODE BEGIN WHILE */
while (1)
{
  // --- read OUT_TEMP (blocking, 1 byte) ---
  uint8_t t_u8 = 0;
  uint8_t t_cmd = 0x80 | (I3G4250D_OUT_TEMP & 0x3F); // READ + addr
  GYRO_CS_L();
  HAL_SPI_Transmit(&hspi1, &t_cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive (&hspi1, &t_u8, 1, HAL_MAX_DELAY);
  GYRO_CS_H();

  // print ONE signed integer + newline
  int8_t t = (int8_t)t_u8;
  char line[16];
  int n = snprintf(line, sizeof(line), "%d\n", (int)t);
  if (n > 0) uart_print(line);

  HAL_Delay(150);  // 100–200 ms window
}

/* USER CODE END WHILE */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

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
  hi2c1.Init.Timing = 0x2000090E;
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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1)
  {
    if (HAL_SPI_Receive_IT(&hspi1, &temp_raw, 1) != HAL_OK) {
      GYRO_CS_H(); spi_busy = 0;
    }
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1)
  {
    GYRO_CS_H();
    spi_busy = 0;
    next_due = HAL_GetTick() + TEMP_PERIOD_MS;
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
