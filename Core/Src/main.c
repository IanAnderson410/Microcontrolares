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
#include "usb_device.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//Librerías del MPU
#include "math.h"
#include "stdio.h"
//Librerías de us gral
#include "usbd_cdc_if.h"
/* USER CODE END Includes */


/* USER CODE BEGIN PTD */
// --- 1. Estructura Protocolo UNER (33 Bytes) ---
typedef struct __attribute__((packed)) {
    float pitch;            // 4 bytes
    float roll;             // 4 bytes
    float yaw;              // 4 bytes
    float velocidad;        // 4 bytes
    uint16_t sensoresIR[8]; // 16 bytes
    uint8_t modo;           // 1 byte
} PayloadUNER;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0 // Dirección I2C (AD0 = GND)
//#define RAD_TO_DEG 57.295779513082320876798154814105	//Usado en el MPU
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
PayloadUNER txPayload; // Instancia para transmisión

// --- 2. Variables de Estado (Cálculos MPU) ---

//MPU Y PAYLOAD
// Ángulos calculados
float Current_Pitch = 0.0f;
float Current_Roll  = 0.0f;
float Current_Yaw   = 0.0f;

// Variables auxiliares
float dt = 0.01f; // ASUMO 100Hz (Ajusta esto según tu timer de lectura MPU)
const float RAD_TO_DEG = 57.295779f;

// Variables externas (Placeholder para lo que aun no conectamos)
float Robot_Velocidad = 0.0f;
uint16_t Lecturas_IR[8] = {8,7,6,5,4,3,2,1};
uint8_t Robot_Modo = 0; // 0=IDLE

//MPU Y PAYLOAD

uint8_t rx_buffer_uart[256];  // Buffer para recibir del ESP8266
uint8_t rx_buffer_usb[256];   // Buffer auxiliar para datos USB
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE]; // Buffer interno del driver USB
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

// Datos Crudos y Buffer DMA
uint8_t mpu_buffer[14]; // El DMA dejará los datos aquí
int16_t Accel_Y_RAW, Accel_Z_RAW, Accel_X_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
// Variables de Control (Físicas)
volatile float Current_Angle_Y = 0.0;
volatile float Gyro_Rate_X = 0.0;
float dt1 = 0.005; // 5ms asumiendo 200Hz

// Semáforos para el Main
volatile uint8_t nueva_lectura_lista = 0;

// USB Debug
char msg_buffer[50];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        CDC_Transmit_FS(rx_buffer_uart, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer_uart, 256);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // Si hubo ruido o error de trama (común al arrancar el ESP)
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer_uart, 256);
    }
}

void MPU6050_Init_IT(I2C_HandleTypeDef *hi2c) {
    uint8_t Data;
    uint8_t temp; // Variable basura para limpiar

    // 1. Despertar
    Data = 0x00; HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x6B, 1, &Data, 1, 100);

    // 2. Sample Rate 100Hz
    Data = 0x09; HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x19, 1, &Data, 1, 100);

    // 3. Filtro DLPF
    Data = 0x03; HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1A, 1, &Data, 1, 100);

    // 4. Configurar INT Pin (Latch y Clear on Read)
    Data = 0x30; HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x37, 1, &Data, 1, 100);

    // 5. Habilitar Interrupción
    Data = 0x01; HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x38, 1, &Data, 1, 100);

    // 6. Escalas
    Data = 0x18; HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1B, 1, &Data, 1, 100);
    Data = 0x10; HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1C, 1, &Data, 1, 100);

    // --- CORRECCIÓN CRÍTICA: "Limpiar la tubería" ---
    // Leemos el registro INT_STATUS (0x3A) una vez para bajar el pin INT a 0V
    // Si no hacemos esto, el pin nace en 3.3V y el STM32 nunca ve el primer flanco.
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3A, 1, &temp, 1, 100);
}
// 1. El MPU avisa que tiene datos (EXTI Callback)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Asegurate que sea el PIN donde conectaste el INT del MPU
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    if(GPIO_Pin == GPIO_PIN_5) {
        // Disparamos la lectura por DMA. NO bloquea el CPU.
    //	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    	HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, 0x3B, 1, mpu_buffer, 14);
    }
}
// 2. El DMA terminó de traer los datos (I2C Callback)
/* USER CODE BEGIN 4 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if(hi2c->Instance == I2C1) {

        // --- A. Lectura RAW (16 bits con signo) ---
        // Índices del Buffer:
        // [0-1] Ax, [2-3] Ay, [4-5] Az
        // [6-7] Temp
        // [8-9] Gx, [10-11] Gy, [12-13] Gz

        int16_t Accel_X_RAW = (int16_t)(mpu_buffer[0] << 8 | mpu_buffer[1]);
        int16_t Accel_Y_RAW = (int16_t)(mpu_buffer[2] << 8 | mpu_buffer[3]); // NUEVO
        int16_t Accel_Z_RAW = (int16_t)(mpu_buffer[4] << 8 | mpu_buffer[5]);

        int16_t Gyro_X_RAW  = (int16_t)(mpu_buffer[8]  << 8 | mpu_buffer[9]);  // NUEVO (Roll Rate)
        int16_t Gyro_Y_RAW  = (int16_t)(mpu_buffer[10] << 8 | mpu_buffer[11]); // (Pitch Rate)
        int16_t Gyro_Z_RAW  = (int16_t)(mpu_buffer[12] << 8 | mpu_buffer[13]); // (Yaw Rate)
        // --- B. Conversión a Unidades Físicas ---
        // Asumiendo escalas default: Accel +/- 2g (4096 LSB/g)?? -> OJO: Default es 16384.
        // Si usas 4096 es porque configuraste +/- 8g. Mantenemos TU escala.
        float Ax = Accel_X_RAW / 4096.0f;
        float Ay = Accel_Y_RAW / 4096.0f;
        float Az = Accel_Z_RAW / 4096.0f;
        // Giroscopio (LSB/(°/s)) -> 65.5 para +/- 500dps, 16.4 para +/- 2000dps.
        // Mantenemos TU escala (16.4).
        float Gyro_Rate_X = Gyro_X_RAW / 16.4f;
        float Gyro_Rate_Y = Gyro_Y_RAW / 16.4f;
        float Gyro_Rate_Z = Gyro_Z_RAW / 16.4f;
        // --- C. Fusión de Sensores (Cálculo de Ángulos) ---
        // 1. PITCH (Inclinación Adelante/Atrás - Para Equilibrio)
        // atan2(Ax, Az) nos da el ángulo del vector gravedad proyectado en XZ
        float Accel_Angle_Pitch = atan2(Ax, Az) * RAD_TO_DEG;
        // Filtro Complementario: 98% Giroscopio + 2% Acelerómetro
        Current_Pitch = 0.98f * (Current_Pitch + Gyro_Rate_Y * dt) + 0.02f * Accel_Angle_Pitch;
        // 2. ROLL (Ladeo Izquierda/Derecha)
        // atan2(Ay, Az) nos da el ladeo lateral
        float Accel_Angle_Roll = atan2(Ay, Az) * RAD_TO_DEG;
        Current_Roll = 0.98f * (Current_Roll + Gyro_Rate_X * dt) + 0.02f * Accel_Angle_Roll;
        // 3. YAW (Brújula / Dirección)
        // Solo integración del giroscopio Z (Tiene drift con el tiempo, normal en IMUs baratas)
        Current_Yaw += Gyro_Rate_Z * dt;
        // Mantener Yaw entre 0 y 360 grados (Estética para la brújula en Qt)
        if(Current_Yaw >= 360.0f) Current_Yaw -= 360.0f;
        if(Current_Yaw < 0.0f)    Current_Yaw += 360.0f;

        nueva_lectura_lista=1;
    }
}
void ProtocoloUNER(void)
{
    if(huart1.gState != HAL_UART_STATE_READY) return;
    txPayload.pitch = Current_Pitch;
    txPayload.roll  = Current_Roll;
    txPayload.yaw   = Current_Yaw;
    txPayload.velocidad = Robot_Velocidad;
    txPayload.modo      = Robot_Modo;
    memcpy(txPayload.sensoresIR, Lecturas_IR, sizeof(Lecturas_IR));
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&txPayload, sizeof(PayloadUNER));
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
  static uint32_t last_print = 0;
    	  static uint32_t lastTime0 = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init_IT(&hi2c1); // Inicializamos con interrupciones
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer_uart, 256);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (nueva_lectura_lista && (HAL_GetTick() - last_print > 100)) {
	  	          nueva_lectura_lista = 0; // Bajamos la bandera
	  	          last_print = HAL_GetTick();
	  	          // Imprimir flotante (asegurate de tener habilitado float en printf en settings)
	  	          //int angulo_entero = (int)Current_Angle_Y;
	  	          //int angulo_decimal = abs((int)((Current_Angle_Y - angulo_entero) * 100));
	  	          //   int len = sprintf(msg_buffer, "Angulo: %d.%02d\r\n", angulo_entero, angulo_decimal);
	  	          int yaw_entero = (int)Current_Yaw;
	  			  int yaw_decimal = abs((int)((Current_Yaw - yaw_entero) * 100));
	  			  // Mostramos "Brújula"
	  			  //int len = sprintf(msg_buffer, "Brujula: %d.%02d\r\n", yaw_entero, yaw_decimal);
	  			  //  CDC_Transmit_FS((uint8_t*)msg_buffer, len);

	  			  ProtocoloUNER();
	  	      	  }


	  	      if (HAL_GetTick() - lastTime0 > 500) {
	  	          lastTime0 = HAL_GetTick();
	  	          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
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
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
