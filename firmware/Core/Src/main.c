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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float last_error;
    float out_min, out_max;
} PID_TypeDef;

typedef struct __attribute__((packed)) {
    uint16_t header;      // 2 bytes: 0xABCD
    uint32_t timestamp;   // INSTANTE DE LA INT DE LA IMU

    int32_t  enc_l;       // 4 bytes
    int32_t  enc_r;       // 4 bytes

    int16_t  accel[3];    // 6 bytes:
    int16_t  gyro[3];     // 6 bytes
    int16_t  magnet[3];   // 6 bytes

    uint16_t batt_v;      // 2 bytes
    uint16_t batt_curr;   // 2 bytes

    uint8_t  flags;       // 1 byte: BIT0 = MAGNET UPDATED, BIT1 = BATT UPDATED, BIT3 = TRIGGER FOTO
    uint8_t  checksum;    // 1 byte: XOR DE TODOS LOS BYTES DEL PAQUETE
} Telemetry_t;

typedef struct __attribute__((packed)) {
    uint16_t header;
    float    target_vel_l;
    float    target_vel_r;
    uint8_t  checksum;
} Command_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ICM20948_ADDR (0x69 << 1)
#define MAG_ADDR      (0x0C << 1)


#define AIN1_PIN GPIO_PIN_13
#define AIN1_PORT GPIOB
#define AIN2_PIN GPIO_PIN_12
#define AIN2_PORT GPIOB

#define BIN1_PIN GPIO_PIN_14
#define BIN1_PORT GPIOB
#define BIN2_PIN GPIO_PIN_15
#define BIN2_PORT GPIOB

#define CMD_WATCHDOG_TIMEOUT 500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
// PID
PID_TypeDef pid_l = {40.0f, 2.0f, 0.0f, 0.0f, 0.0f, -8399.0f, 8399.0f};
PID_TypeDef pid_r = {40.0f, 2.0f, 0.0f, 0.0f, 0.0f, -8399.0f, 8399.0f};

// TELEMETRIA
Telemetry_t tx_buffer;
volatile uint8_t uart_ready = 1;

// IMU
volatile int16_t imu_accel_x, imu_accel_y, imu_accel_z;
volatile int16_t imu_gyro_x,  imu_gyro_y,  imu_gyro_z;
volatile int16_t mag_x, mag_y, mag_z;
volatile uint32_t last_imu_ts    = 0;
volatile uint32_t last_mag_bat_ts = 0;
volatile uint8_t  imu_data_ready_flag = 0;

// ENCODERS
volatile int32_t snapshot_enc_l = 0;
volatile int32_t snapshot_enc_r = 0;
int32_t prev_enc_l = 0;
int32_t prev_enc_r = 0;

// ADC
uint16_t adc_buffer[2];

// UART RX
Command_t  rx_cmd;
volatile float setpoint_l = 0.0f, setpoint_r = 0.0f;
uint8_t raw_rx_buffer[sizeof(Command_t)];

// TIMING
uint32_t last_led_tick  = 0;
uint32_t last_slow_loop = 0;

//WATCHDOG CONSIGNA VELOCIDAD
volatile uint32_t last_cmd_time = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void build_telemetry_packet(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//PID
float PID_Compute(PID_TypeDef *pid, float setpoint, float measured) {
    float error = setpoint - measured;
    float P = pid->Kp * error;
    pid->integral += error;

    // Anti-windup simple
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    else if (pid->integral < pid->out_min) pid->integral = pid->out_min;

    float I = pid->Ki * pid->integral;
    float D = pid->Kd * (error - pid->last_error);
    pid->last_error = error;

    float output = P + I + D;
    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;
    return output;
}
//CHECKSUM
uint8_t calculate_checksum(uint8_t* data, uint16_t len) {
    uint8_t checksum = 0;
    for(uint16_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}
//DRIVER IMU ICM-20948
void ICM20948_Init(void) {
    uint8_t data;
    // 1. RESET INTEGRAL
    data = 0x80; // Reset bit
    HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x06, 1, &data, 1, 10); // PWR_MGMT_1
    HAL_Delay(50);

    // SELECCIÓN DE RELOJ AUTOMÁTICO
    data = 0x01; // Auto clock select
    HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x06, 1, &data, 1, 10); // PWR_MGMT_1
    HAL_Delay(10);

    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x7F, 1, &data, 1, 10); // REG_BANK_SEL

    // CONFIGURAR PIN DE INTERRUPCIÓN Y BYPASS
    data = 0x12;
    HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x0F, 1, &data, 1, 10); // INT_PIN_CFG

    // ACTIVAR INTERRUPCIÓN
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x11, 1, &data, 1, 10); // INT_ENABLE_1

    // CAMBIAR A BANCO 2 PARA CONFIGURAR IMU
    data = 0x20;
    HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x7F, 1, &data, 1, 10); // REG_BANK_SEL

    // CONFIGURACIÓN GYRO
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x01, 1, &data, 1, 10); // GYRO_CONFIG_1

    // ODR: 1125 / (1 + 10) = 102.2 HZ
    data = 0x0A;
    HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x00, 1, &data, 1, 10); // GYRO_SMPLRT_DIV

    // CONFIGURACIÓN ACCEL
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x14, 1, &data, 1, 10); // ACCEL_CONFIG

    // ODR: 1125 / (1 + 10) = 102.2 HZ
    data = 0x00; HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x10, 1, &data, 1, 10); // MSB
    data = 0x0A; HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x11, 1, &data, 1, 10); // LSB

    // VOLVER AL BANCO 0
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, 0x7F, 1, &data, 1, 10);

    // CONFIGURAR MAGNETOMETRO
    data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, MAG_ADDR, 0x31, 1, &data, 1, 10);
}
void IMU_Read_Fast(void) {
    uint8_t raw_data[12];
    // LECTURA ACCEL Y GYRO
    if(HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDR, 0x2D, 1, raw_data, 12, 5) == HAL_OK) {
        imu_accel_x = (int16_t)(raw_data[0] << 8 | raw_data[1]);
        imu_accel_y = (int16_t)(raw_data[2] << 8 | raw_data[3]);
        imu_accel_z = (int16_t)(raw_data[4] << 8 | raw_data[5]);
        imu_gyro_x  = (int16_t)(raw_data[6] << 8 | raw_data[7]);
        imu_gyro_y  = (int16_t)(raw_data[8] << 8 | raw_data[9]);
        imu_gyro_z  = (int16_t)(raw_data[10] << 8 | raw_data[11]);
    }
}
void IMU_Read_Mag(void) {
    uint8_t mag_data[6];
    // LECTURA MAGNET
    if(HAL_I2C_Mem_Read(&hi2c1, MAG_ADDR, 0x11, 1, mag_data, 6, 2) == HAL_OK) {
        mag_x = (int16_t)(mag_data[1] << 8 | mag_data[0]);
        mag_y = (int16_t)(mag_data[3] << 8 | mag_data[2]);
        mag_z = (int16_t)(mag_data[5] << 8 | mag_data[4]);

        uint8_t dummy;
        HAL_I2C_Mem_Read(&hi2c1, MAG_ADDR, 0x18, 1, &dummy, 1, 1);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // INICIAR CONTADOR ENCODERS
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  // INICIAR TIMER3 PARA TIMESTAMPS PRECISOS (1MHZ)
  HAL_TIM_Base_Start(&htim3);

  // INICIAR TIMER4 PARA PID (1KHZ)
  HAL_TIM_Base_Start_IT(&htim4);

  // INICIAR TIMER1 PARA PWM (DC INICIAL AL 0%)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  // INICIAR ADC POR DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 2);

  // INICIAR RECEPCION POR UART2
  HAL_UART_Receive_DMA(&huart2, raw_rx_buffer, sizeof(Command_t));

  // INICIALIZACION CONFIGURACION IMU
  ICM20948_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t current_time = HAL_GetTick();

	  // LED BLINKING PLACA
	  if (current_time - last_led_tick >= 500) {
		  last_led_tick = current_time;
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  }

	  // WATCHDOG VELOCIDAD
		if (current_time - last_cmd_time > CMD_WATCHDOG_TIMEOUT) {
			setpoint_l = 0.0f;
			setpoint_r = 0.0f;
		}

	  // LECTURA MAGNETOMETRO Y BATERIA
	  if (current_time - last_slow_loop >= 20) {
		  last_slow_loop = current_time;
		  IMU_Read_Mag();

		  tx_buffer.batt_v = adc_buffer[0];
		  tx_buffer.batt_curr = adc_buffer[1];

		  // FLAG MAGNET Y BATERIA ACTULIZADOS
		  tx_buffer.flags |= 0x03;
	  }

	  // GESTIÓN DE TELEMETRIA (CUANDO INT DE LA IMU, EL CB PONE FLAG A 1)
	  if (imu_data_ready_flag == 1) {
		  imu_data_ready_flag = 0;

		  // LECTURA ACCEL Y GYRO
		  IMU_Read_Fast();


		  // SI UART READY SE ENVIA
		  if(uart_ready) {
			  uart_ready = 0;
			  build_telemetry_packet();
			  HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&tx_buffer, sizeof(Telemetry_t));
			  tx_buffer.flags = 0; // LIMPIAR FLAGS DESPUES DE ENVIO
		  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart2.Init.BaudRate = 460800;
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
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// CALLBACK RECEPCIÓN UART (DMA)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART2) {
        // GUARDA RECEPCION EN BUFFER
        Command_t *cmd_ptr = (Command_t*)raw_rx_buffer;

        // COMPROBAR HEADER
        if(cmd_ptr->header == 0xBBBB) {

            // CALCULO CHECKSUM
        	uint8_t calculated_sum = calculate_checksum(raw_rx_buffer, sizeof(Command_t) - 1);

			// COMPRUEBA CHECKSUM
			if(calculated_sum == cmd_ptr->checksum) {
				last_cmd_time = HAL_GetTick(); // RESET WATCHDOG
				// SI CHECKSUM CORRECTO SE GUARDA CONSIGNA RECIBIDA (EN TICKS/S)
				setpoint_l = cmd_ptr->target_vel_l / 100.0f; // CONVERSION TICKS/S A TICKS POR CADA 10MS (YA QUE PID SE EJECUTA A 100HZ)
				setpoint_r = cmd_ptr->target_vel_r / 100.0f;
            }
        }
    }
}
// CALLBACK INTERRUPCION DE LA IMU (EXTI0)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == GPIO_PIN_0) {
        // CAPTURA DE IMU Y ENCODERS
        last_imu_ts = __HAL_TIM_GET_COUNTER(&htim3);
        snapshot_enc_l = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        snapshot_enc_r = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

        // FLAG: NUEVOS DATOS A ENVIAR
        imu_data_ready_flag = 1;
    }
}
// CALLBACK UART HA TERMINADO DE ENVIAR, VEULVE A ESTAR LISTA
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART2) {
        uart_ready = 1;
    }
}

// FUNCIÓN AUXILIAR: CONSTRUCCIÓN DEL PAQUETE
void build_telemetry_packet(void) {
    tx_buffer.header    = 0xABCD;
    tx_buffer.timestamp = last_imu_ts;
    tx_buffer.enc_l     = snapshot_enc_l;
    tx_buffer.enc_r     = snapshot_enc_r;


    tx_buffer.accel[0]  = imu_accel_x;
    tx_buffer.accel[1]  = imu_accel_y;
    tx_buffer.accel[2]  = imu_accel_z;
    tx_buffer.gyro[0]   = imu_gyro_x;
    tx_buffer.gyro[1]   = imu_gyro_y;
    tx_buffer.gyro[2]   = imu_gyro_z;


    tx_buffer.magnet[0] = mag_x;
    tx_buffer.magnet[1] = mag_y;
    tx_buffer.magnet[2] = mag_z;

    // CALCULO CHECKSUM DEL PCKG MENOS DEL ULTIMO BYTE
    tx_buffer.checksum = calculate_checksum((uint8_t*)&tx_buffer, sizeof(Telemetry_t) - 1);
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM4)
  {
      // LEER ENCODERS
      int32_t current_enc_l = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
      int32_t current_enc_r = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

      // CALCULAR VELOCIDAD ACTUAL (DELTA)
      float delta_l = (float)((int16_t)(current_enc_l - prev_enc_l));
      float delta_r = (float)((int16_t)(current_enc_r - prev_enc_r));

      prev_enc_l = current_enc_l;
      prev_enc_r = current_enc_r;

      // CALCULAR SALIDA PID
      float out_l = PID_Compute(&pid_l, setpoint_l, delta_l);
      float out_r = PID_Compute(&pid_r, setpoint_r, delta_r);

      // MOTOR IZQUIERDO
      if (out_l >= 0.0f) {
          HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
          HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
      } else {
          HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
          out_l = -out_l; // PWM SIEMPRE POSITIVO
      }

      // MOTOR DERECHO
      if (out_r >= 0.0f) {
          HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
          HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
      } else {
          HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
          out_r = -out_r;
      }

      // LIMITAR Y APLICAR PWM
      if (out_l > 8399.0f) out_l = 8399.0f;
      if (out_r > 8399.0f) out_r = 8399.0f;

      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)out_l);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)out_r);
  }



  /* USER CODE END Callback 1 */
}

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
