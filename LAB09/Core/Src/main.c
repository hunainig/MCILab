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
#include "stdio.h"
#include "string.h"
#include "usb_device.h"


/* Private variables ---------------------------------------------------------*/
// I2C_HandleTypeDef hi2c1;
// SPI_HandleTypeDef hspi1;
// UART_HandleTypeDef huart1;
// PCD_HandleTypeDef hpcd_USB_FS;
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



/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* User variables */
// const uint8_t LSM303AGR_7BIT_ADDR = 0x33;    // 7-bit address
// const uint8_t LSM303AGR_WHO_AM_I = 0x0F;     // WHO_AM_I register
// uint8_t dev_addr;                            // 8-bit HAL device address (7-bit << 1)
// uint8_t whoami = 0;
// #define LSM_A_ADDR_8 (0x19u << 1)
// #define LSM_A_WHOAMI 0x0F
// char msg[80];
/* USER CODE END PFP */
/* ---- integer-only printing of g-values ---- */
#define GYRO_CS_GPIO_Port   GPIOE
#define GYRO_CS_Pin         CS_I2C_SPI_Pin
#define GYRO_CS_LOW()       HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()      HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET)

//GYROSCOPE
/* ===== L3GD20 (on-board gyro) via SPI1 ===== */
#define GYRO_REG_WHOAMI   0x0F   // expect 0xD4 or 0xD7
#define GYRO_REG_CTRL1    0x20   // power + ODR
#define GYRO_REG_CTRL4    0x23   // full-scale
#define GYRO_REG_OUT_X_L  0x28   // X/Y/Z output regs start
/* SPI read/multi bits for L3GD20: bit7=1 (READ), bit6=1 (auto-inc) */
#define GYRO_SPI_READ     0x80
#define GYRO_SPI_AUTO_INC 0x40

#define GYRO_SENS_250DPS  0.00875f  // dps/LSB @ ±250 dps

static void uart_print(const char *s) {
  HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}
static inline int16_t s16(uint8_t lo, uint8_t hi) {
  return (int16_t)((hi << 8) | lo);
}
static void gyro_write_u8(uint8_t reg, uint8_t val) {
  uint8_t tx[2] = { reg & 0x3F, val }; // write: bit7=0, bit6=0
  GYRO_CS_LOW();
  HAL_SPI_Transmit(&hspi1, tx, 2, 100);
  GYRO_CS_HIGH();
}

static void gyro_read_n(uint8_t reg, uint8_t *buf, uint16_t n) {
  uint8_t header = (reg & 0x3F) | GYRO_SPI_READ | (n>1 ? GYRO_SPI_AUTO_INC : 0);
  GYRO_CS_LOW();
  HAL_SPI_Transmit(&hspi1, &header, 1, 100);
  HAL_SPI_Receive(&hspi1, buf, n, 100);
  GYRO_CS_HIGH();
}

static uint8_t gyro_read_u8(uint8_t reg) {
  uint8_t v=0; gyro_read_n(reg, &v, 1); return v;
}

/* Bring gyro up: normal mode, 100 Hz, all axes on; ±250 dps */
static void gyro_init(void) {
  /* CTRL1: ODR=95/100Hz & enable X/Y/Z + power on -> 0x0F is common start */
  gyro_write_u8(GYRO_REG_CTRL1, 0x0F);
  /* CTRL4: scale = ±250 dps (FS bits = 00), continuous update -> 0x00 */
  gyro_write_u8(GYRO_REG_CTRL4, 0x00);
  HAL_Delay(10);
}

// static inline int16_t s16(uint8_t lo, uint8_t hi) { return (int16_t)((hi<<8)|lo); }

/* Read X/Y/Z raw and convert to dps (we’ll print as fixed-point) */
typedef struct { float gx, gy, gz; } gyro_t;

static void gyro_read(gyro_t *g) {
  uint8_t b[6];
  gyro_read_n(GYRO_REG_OUT_X_L, b, 6);  // auto-increment reads X/Y/Z L/H
  int16_t x = s16(b[0], b[1]);
  int16_t y = s16(b[2], b[3]);
  int16_t z = s16(b[4], b[5]);
  g->gx = x * GYRO_SENS_250DPS;
  g->gy = y * GYRO_SENS_250DPS;
  g->gz = z * GYRO_SENS_250DPS;
}


static void dps_to_text(char *dst, size_t n, float dps){
  int32_t mdps = (int32_t)((dps>=0)?(dps*1000.0f+0.5f):(dps*1000.0f-0.5f));
  int32_t s = (mdps<0); if(s) mdps=-mdps;
  int32_t i = mdps/1000, f = mdps%1000;
  if(s) snprintf(dst,n,"-%ld.%03ld",(long)i,(long)f);
  else  snprintf(dst,n, "%ld.%03ld",(long)i,(long)f);
}
static void print_gyro_csv(float gx,float gy,float gz){
  char ax[16], ay[16], az[16], line[64];
  dps_to_text(ax,sizeof ax,gx);
  dps_to_text(ay,sizeof ay,gy);
  dps_to_text(az,sizeof az,gz);
  snprintf(line,sizeof line,"%s, %s, %s\r\n",ax,ay,az);
  uart_print(line);
}

//ACCELEROMETER
/* convert one g-value (float) to text like "-0.012" WITHOUT float printf */
static void g_to_text(char *dst, size_t n, float g) {
  /* scale to milli-g and round */
  int32_t mg = (int32_t)( (g >= 0.0f) ? (g*1000.0f + 0.5f) : (g*1000.0f - 0.5f) );
  int32_t s = (mg < 0);
  if (s) mg = -mg;
  int32_t i = mg / 1000;      /* integer part in g */
  int32_t f = mg % 1000;      /* fractional part in mg */
  if (s)
    snprintf(dst, n, "-%ld.%03ld", (long)i, (long)f);
  else
    snprintf(dst, n,  "%ld.%03ld", (long)i, (long)f);
}

/* print three g-values as CSV: ax, ay, az using only integer printf */
static void print_xyz_csv(float ax, float ay, float az) {
  char a[16], b[16], c[16], line[64];
  g_to_text(a, sizeof(a), ax);
  g_to_text(b, sizeof(b), ay);
  g_to_text(c, sizeof(c), az);
  snprintf(line, sizeof(line), "%s, %s, %s\r\n", a, b, c);
  uart_print(line);
}
//task2
/* ===== LSM303AGR accelerometer (ACC) ===== */
#define LSM_A_ADDR_8        (0x19u << 1)   // 0x32
#define LSM_A_CTRL1         0x20           // ODR & axes enable
#define LSM_A_CTRL4         0x23           // full-scale & mode
#define LSM_A_OUT_X_L       0x28           // first data reg (auto-inc)
#define LSM_A_AUTO_INC      0x80           // OR this for multi-byte read

/* Lab’s Normal mode scale: 3.9 mg/LSB = 0.0039 g/LSB (±2g) */
#define LSM_A_G_PER_LSB     0.0039f

static HAL_StatusTypeDef i2c_write8(uint16_t dev8, uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(&hi2c1, dev8, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}
static HAL_StatusTypeDef i2c_readn(uint16_t dev8, uint8_t reg, uint8_t *buf, uint16_t n) {
  return HAL_I2C_Mem_Read(&hi2c1, dev8, reg, I2C_MEMADD_SIZE_8BIT, buf, n, 100);
}


typedef struct {
  float ax, ay, az;      // in g
  float offx, offy, offz;
} lsm_acc_t;

/* 100 Hz ODR, all axes on; ±2g normal mode */
static void LSM_Accel_Init(void) {
  /* CTRL1_A:
     ODR=100 Hz (0b0101 << 4 = 0x50), LPen=0 (normal), Zen/Yen/Xen=1 → 0x07
     => 0x57 */
  i2c_write8(LSM_A_ADDR_8, LSM_A_CTRL1, 0x57);

  /* CTRL4_A:
     BDU=0, BLE=0, FS=±2g(00), HR=0 (normal mode) → 0x00 */
  i2c_write8(LSM_A_ADDR_8, LSM_A_CTRL4, 0x00);

  HAL_Delay(10);
}

/* Read raw XYZ (6 bytes), convert to g (Normal mode: 10-bit/12-bit effective; lab scale 3.9 mg/LSB) */
static HAL_StatusTypeDef LSM_Accel_Read(lsm_acc_t *m) {
  uint8_t b[6];
  HAL_StatusTypeDef st = i2c_readn(LSM_A_ADDR_8, (LSM_A_OUT_X_L | LSM_A_AUTO_INC), b, 6);
  if (st != HAL_OK) return st;

  int16_t rx = s16(b[0], b[1]);
  int16_t ry = s16(b[2], b[3]);
  int16_t rz = s16(b[4], b[5]);

  /* Convert to g using 0.0039 g/LSB, then subtract offsets */
  m->ax = rx * LSM_A_G_PER_LSB - m->offx;
  m->ay = ry * LSM_A_G_PER_LSB - m->offy;
  m->az = rz * LSM_A_G_PER_LSB - m->offz;
  return HAL_OK;
}

/* Simple offset calibration: average N samples while device is still/flat */
static void LSM_Accel_Calibrate(lsm_acc_t *m, int N) {
  float sx=0, sy=0, sz=0;
  lsm_acc_t t = {0};
  for (int i=0; i<N; i++) {
    if (LSM_Accel_Read(&t) == HAL_OK) {
      sx += t.ax; sy += t.ay; sz += t.az;
    }
    HAL_Delay(10);
  }
  /* If lying flat Z ~ +1 g. We want offsets that zero readings except gravity on Z.
     For Task 2 we’ll just remove the raw bias; keep gravity as-is (no 1g correction).
     If you want Z ~ 0 when flat, use m->offz = (sz/N) - 1.0f; */
  m->offx = sx / N;
  m->offy = sy / N;
  m->offz = sz / N;   // or (sz/N) - 1.0f to make flat Z≈0
}
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len); // retarget printf to UART
/* USER CODE END 0 */
int _write(int file, char *ptr, int len)
{
  /* file param is ignored; send over huart1 */
  if (HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY) == HAL_OK)
    return len;
  return -1;
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // dev_addr = (uint8_t)(LSM303AGR_7BIT_ADDR << 1); // HAL wants 8-bit address
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // dev_addr = (uint8_t)(LSM303AGR_7BIT_ADDR << 1); // HAL wants 8-bit address
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
HAL_Delay(100);

/* --- SPI CS idle high before any SPI use --- */
GYRO_CS_HIGH();

/* --- Accelerometer bring-up --- */
if (HAL_I2C_IsDeviceReady(&hi2c1, LSM_A_ADDR_8, 3, 50) == HAL_OK) {
  printf("Accel present\r\n");
} else {
  printf("Accel NOT present (I2C err=0x%lX)\r\n", hi2c1.ErrorCode);
}
LSM_Accel_Init();
lsm_acc_t a = {0};
/* quick bias estimate (optional) */
LSM_Accel_Calibrate(&a, 20);

/* --- Gyro bring-up --- */
uint8_t gid = gyro_read_u8(GYRO_REG_WHOAMI);   // expect 0xD4 or 0xD7
printf("Gyro WHO_AM_I=0x%02X\r\n", gid);
gyro_init();
/* Stream CSV every ~100 ms: ax, ay, az in g */
gyro_t g;
while (1) {

    /* USER CODE BEGIN 3 */
  gyro_read(&g);

  /* read accel and use Y */
  LSM_Accel_Read(&a);   // 'a' already holds offsets from calibration

  /* print: gx (dps), ay (g) using integer-only formatting */
  char gy_txt[16], ay_txt[16], line[64];
dps_to_text(gy_txt, sizeof gy_txt, g.gy);
g_to_text(ay_txt, sizeof ay_txt, a.ay);
snprintf(line, sizeof line, "GyroY=%s dps, AccY=%s g\r\n", gy_txt, ay_txt);

  uart_print(line);

  HAL_Delay(100);
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */


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

//TASK 3 -----------------------------------------------------------------------
