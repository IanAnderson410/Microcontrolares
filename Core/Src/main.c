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
  *
  *   Este firmware implementa un sistema de control de lazo cerrado para un robot
  *   balancín mediante el uso de un microcontrolador STM32. La lógica principal
  *   reside en una interrupción periódica de 10ms donde se procesan los datos
  *   del sensor MPU6050, obtenidos mediante DMA, fusionándolos a través de un
  *   filtro complementario que combina la estabilidad del acelerómetro con la
  *   velocidad del giroscopio previamente calibrado para eliminar el bias. Con
  *   el ángulo calculado, un algoritmo PID genera una señal de corrección
  *   que se traduce en señales PWM para los motores, incluyendo una compensación
  *   de zona muerta (deadband) para vencer la fricción estática inicial y una
  *   gestión de dirección por hardware específica en los pines PA9 y PA10 para
  *   el motor izquierdo y PB3 con PA15 para el derecho, contando además con un
  *   sistema de seguridad que desactiva los actuadores ante inclinaciones críticas
  *   superiores a 45 grados.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//Al usar __attribute__((packed)), garantizamos que no haya "padding" (huecos de memoria)
typedef struct __attribute__((packed)) {
    int16_t acc_x, acc_y, acc_z; // 6 bytes - Datos crudos para debug en Qt
    int16_t gyro_pitch, gyro_yaw; // 4 bytes - Pitch (Y) y Yaw (Z)
    float pitch_angle;            // 4 bytes - El theta filtrado
    float pos_x;                  // 4 bytes - Trayectoria X
    float pos_y;                  // 4 bytes - Trayectoria Y
    float velocidad;              // 4 bytes - Velocidad lineal
    uint8_t modo;                 // 1 byte  - IDLE, FOLLOW_LINE, etc.
    uint16_t IR[8];
    uint8_t infoAdicional;
} PayloadData_t;

typedef union {
    PayloadData_t data;
    uint8_t buffer[sizeof(PayloadData_t)]; // 27 bytes totales
} PayloadUNER_t;


typedef struct {
    uint8_t header[4];      // "UNER"
    uint8_t length;         // CMD + N_Payload + Checksum
    uint8_t token;          // ':'
    uint8_t cmd;            // ID del comando
    uint8_t payload[64];    // Buffer genérico (sobra espacio)
    // El checksum no lo ponemos en el struct fijo porque su posición varía
} UnerPacket_t;

enum {
    // Sistema y Heartbeat
    CMD_ALIVE       = 0x01, // Verificar conexión
    CMD_SET_HB      = 0x02, // Configurar intervalo de Heartbeat

    // Control de Estado
    CMD_CALIBRATE   = 0x05, // Calibración de MPU6050
    CMD_START       = 0x06, // Activar motores / Inicio de balanceo
    CMD_STOP        = 0x07, // Parada de emergencia / Motores a 0

    // Control Remoto (Manual)
    CMD_MOVE_RC     = 0x10, // Movimiento manual (adelante, atrás, giro)

    // Parámetros PID (Crucial para el balanceo)
    CMD_PID_KP      = 0x20, // Ajustar constante Proporcional
    CMD_PID_KI      = 0x21, // Ajustar constante Integral
    CMD_PID_KD      = 0x22, // Ajustar constante Derivativa
    CMD_PID_SETPOINT = 0x23, // Ajustar ángulo de equilibrio (offset del centro de masa)

    // Telemetría (STM32 -> Qt)
    CMD_TELEMETRY   = 0xA0, // Envío de ángulos, velocidad y sensores IR
    CMD_LOG_MSG     = 0xA1  // Envío de mensajes de texto para debug
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR (0x68 << 1) // Dirección I2C desplazada

#define UNER_HEADER_STR "UNER"
#define UNER_TOKEN      ':'    // 0x3A según tu PDF o preferencia


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// ================= [ Flags ] ================= //
volatile 	uint8_t flagDisplay=0;
			uint8_t flagSendUNER = 0;
			uint8_t dma_ready = 0;
			uint8_t calibration_ready = 0; // Bandera para no activar el PID antes de tiempo
// ================= [ Counters ] ================= //
			uint16_t contador = 0;
volatile 	uint32_t counter=0;		/*!< Utilizado en la interrupción del Timer 4. Es volatile por que se usan en interrupciones*/
			uint8_t  counter1=0;	/*!< Utilizado para refrezcar la pantalla OLED*/
//RECEPCION DE DATOS
char rx_buffer[20];
uint8_t rx_index = 0;
uint8_t rx_data;
// Nuevas variables para compensar la diferencia entre motores
int16_t deadband_L = 500;		/*!< Zona Muerta del PWM para el motor 1*/
int16_t deadband_R = 500; 		/*!< Zona Muerta del PWM para el motor 2*/
// =================[ Variables de Control PID ] ================= //
float Kp = 80.0;				/*!< Término Proporcional*/
float Ki = 0.5;					/*!< Término Integrativo: */
float Kd = 2.5;					/*!< Término Derivativo: */
float integral = 0, last_error = 0;
float setpoint = 0.0; 						// El ángulo donde el robot se queda parado (0 grados)
// =================[ Variables del Filtro del MPU6050 ] =================//
float angle_y = 0;
float alpha = 0.95; // Factor del filtro complementario
uint32_t last_time = 0;
uint8_t mpu_data[14]; // Los 14 bytes que trae el DMA
// =================[ Variables de Calibración ] =================//
float gyro_bias = 0;
volatile float giro=0, giro_z=0, salida=0;
volatile uint16_t accelx=0, accely=0, accelz=0;
uint32_t lastTime0 = 0;
PayloadUNER_t telemetria;
// recibidos desde el qt
uint8_t rx_buffer_uart[256];
uint16_t delayHB= 60; //ENTRE 1 Y 200
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void DataToQt();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Robot_Drive(int16_t speed_L, int16_t speed_R);
void MPU6050_Calibrate(void);
void MPU6050_Init(I2C_HandleTypeDef *hi2c);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DataToQt(){

		flagSendUNER=0;
		telemetria.data.acc_x = accelx;
		telemetria.data.acc_y = accely;
		telemetria.data.acc_z = accelz;
		telemetria.data.gyro_pitch = (int16_t)giro;
		telemetria.data.gyro_yaw = (int16_t)giro_z;
		telemetria.data.pitch_angle = angle_y;
		telemetria.data.pos_x = 3;
		telemetria.data.pos_y = 2;
		telemetria.data.velocidad = 1;
		telemetria.data.modo = 0;
		telemetria.data.IR[0] = 100;
		telemetria.data.IR[1] = 200;
		telemetria.data.IR[2] = 300;
		telemetria.data.IR[3] = 400;
		telemetria.data.IR[4] = 500;
		telemetria.data.IR[5] = 600;
		telemetria.data.IR[6] = 700;
		telemetria.data.IR[7] = 800;
		telemetria.data.infoAdicional=0;
		uint8_t frame[52]; 							// 4(UNER) + 1(LEN) + 1(TOKEN) + 1(CMD) + 27(PAYLOAD) + 1(CHK)
		memcpy(&frame[0], "UNER", 4);    			// Header
		frame[4] = 46;                  			// Length (CMD + Payload + CHK)
		frame[5] = 0xFD;                			// TOKEN (ejemplo de constante de fin cabecera)
		frame[6] = 0x01;                			// CMD: 0x01 = Telemetría
		memcpy(&frame[7], telemetria.buffer, 44); 	// Payload
		// 3. Cálculo del Checksum XOR
		uint8_t checksum = 0;
		for (int i = 0; i < 51; i++) { // calculamos el checksu,
			checksum ^= frame[i];
		}
		frame[51] = checksum;           // agregamos Checksum
		HAL_UART_Transmit_DMA(&huart1, frame, 52);// enviamos los datos al ESP01 via UART con DMA


}
void Robot_Drive(int16_t speed_L, int16_t speed_R) {
    // Motor 1 (PA8, PA9, PB4)
	// 1. Aplicar Deadband (Zona muerta)
	    if (speed_L > 0) speed_L += deadband_L;
	    else if (speed_L < 0) speed_L -= deadband_L;

	    if (speed_R > 0) speed_R += deadband_R;
	    else if (speed_R < 0) speed_R -= deadband_R;

	    // 2. Saturación final al ARR (3599)
	    if (speed_L > 3599) speed_L = 3599;
	    if (speed_L < -3599) speed_L = -3599;
	    if (speed_R > 3599) speed_R = 3599;
	    if (speed_R < -3599) speed_R = -3599;

    if (speed_L >= 0) {

       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)speed_L);
    } else {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)(-speed_L));
    }

    // Motor 2 (PB3, PA15, PB5)
    if (speed_R >= 0) {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)speed_R);
    } else {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)(-speed_R));
    }
}

void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check, data;

    // 1. Verificamos si el sensor responde (Who Am I)
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x75, 1, &check, 1, 100);

    if (check == 0x68) { // El valor por defecto del registro WHO_AM_I es 0x68
        // 2. Power Management: Salir de Sleep Mode
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x6B, 1, &data, 1, 100);
        // 3. Configurar Acelerómetro (+/- 2g)
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1C, 1, &data, 1, 100);
        // 4. Configurar Giroscopio (+/- 500 dps)
        data = 0x08;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1B, 1, &data, 1, 100);
        data = 0x03; // Filtro de ~42Hz. Limpia muchísima basura del sensor.
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1A, 1, &data, 1, 100);
    }
}
void MPU6050_Calibrate(void) {
    if(calibration_ready==0){
		int32_t sum_gy = 0;
		int32_t sum_ay = 0; // Para saber el ángulo inicial de inclinación
		int num_samples = 1000; // Un poquito más de muestras para mayor precisión
		uint8_t buffer[6];

		for (int i = 0; i < num_samples; i++) {
			// Leemos Accel_Y (para el ángulo inicial) y Gyro_Y
			// Accel_Y: 0x3D, 0x3E | Gyro_Y: 0x45, 0x46
			// Para optimizar, podrías leer todos los ejes de una, pero vamos a lo que necesitás:
			HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x45, 1, buffer, 2, 100);
			int16_t gy_raw = (int16_t)(buffer[0] << 8 | buffer[1]);
			sum_gy += gy_raw;

			HAL_Delay(2);
		}

		// Bias en unidades RAW para restarlo directo antes de convertir a grados
		// Es más eficiente procesar en RAW y convertir al final
		gyro_bias = (float)sum_gy / (float)num_samples;

		calibration_ready = 1;
		}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// utilizamos esta interrupción del timer para llamar la lectura del I2C vía DMA
	// en la dirección del MPU y cargamos esos datos en mpu_data.
	// En esta interrupción tambien actualizamos la bandera para el display y hacemos
	// el HeartBit con el if del counter
    if (htim->Instance == TIM4 && calibration_ready) {
    	if (hi2c1.State == HAL_I2C_STATE_READY) {
			HAL_I2C_Mem_Read_DMA(&hi2c1, (0x68 << 1), 0x3B, 1, mpu_data, 14);
		}
        counter++;
        if(counter > delayHB){ // ~200ms
            counter = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Heartbeat LED [cite: 46]
        }
    }
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	// en esta interrupción provocada por la llegada de los datos de I2C sacamos los
	// datos en crudo de las aceleraciones y giros para procesarlos y calcular el PID
	// Al procesar el PID inmediatamente después de que el DMA termina de recibir los
//	   datos (HAL_I2C_MemRxCpltCallback), garantizás que el cálculo se hace con los datos
//	   más frescos posibles.
    if (hi2c->Instance == I2C1) {
        // RECIÉN ACÁ los datos en mpu_data son válidos y nuevos
    	int16_t ax = (int16_t)(mpu_data[0] << 8 | mpu_data[1]);
		int16_t ay = (int16_t)(mpu_data[2] << 8 | mpu_data[3]); // Nuevo: Accel Y
		int16_t az = (int16_t)(mpu_data[4] << 8 | mpu_data[5]);

		int16_t gx = (int16_t)(mpu_data[8] << 8 | mpu_data[9]);   // Nuevo: Gyro X (Roll)
		int16_t gy = (int16_t)(mpu_data[10] << 8 | mpu_data[11]); // Gyro Y (Pitch)
		int16_t gz = (int16_t)(mpu_data[12] << 8 | mpu_data[13]); // Nuevo: Gyro Z (Yaw)
        accelx = ax;
        accely = ay;
        accelz = az;
        giro 	= (float)gy / 65.5f;
        giro_z 	= (float)gz / 65.5f;
        angle_y = (float)gx / 65.5f;

        float gyro_rate = ((float)gy / 65.5f) - gyro_bias;
//        giro = (float)gy / 131.0f; // Ejemplo de escala para 250dps
//        float gyro_rate = ((float)gy / 131.0f) - gyro_bias;
	   float accel_angle = atan2f((float)ax, (float)az) * 57.2957f;
	   //angle_y = alpha * (angle_y + gyro_rate * 0.01f) + (1.0f - alpha) * accel_angle;
	   angle_y = alpha * (angle_y + gyro_rate * 0.005f) + (1.0f - alpha) * accel_angle;

	   float error = angle_y - setpoint;
	   float P = Kp * error;
	   //integral += error * 0.01f;
	   integral += error * 0.005f;
	   if(integral > 1000) integral = 1000;
	   else if(integral < -1000) integral = -1000;
	  // float D = Kd * (error - last_error) / 0.01f;
	   float D = Kd * (error - last_error) / 0.005f;
	   last_error = error;
	   float output = P + (Ki * integral) + D;
	   if (angle_y > 45.0f || angle_y < -45.0f) {
		   Robot_Drive(0, 0);
		   integral = 0;
	   } else {
		   Robot_Drive((int16_t)output, (int16_t)output);
		   salida=output;
	   }
    }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1){
    	for (int i = 0; i < (Size - 6); i++){
			if (rx_buffer_uart[i]   == 'U' && rx_buffer_uart[i+1] == 'N' && rx_buffer_uart[i+2] == 'E' && rx_buffer_uart[i+3] == 'R') {
				uint8_t len   = rx_buffer_uart[i+4];
				uint8_t token = rx_buffer_uart[i+5];
				uint8_t cmd   = rx_buffer_uart[i+6];
				if (token != ':') continue;
				uint8_t pos_checksum = i + 5 + len;
				if (pos_checksum >= Size) break; // Evitar desbordamiento si el paquete llegó cortado
				uint8_t checksum_recibido = rx_buffer_uart[pos_checksum];
				uint8_t checksum_calc = 0;
				for(int k = i; k < pos_checksum; k++){
					checksum_calc ^= rx_buffer_uart[k];
				}
				if (checksum_calc == checksum_recibido) {
					uint8_t *payload_ptr = &rx_buffer_uart[i+7];
					switch(cmd) {
						case CMD_SET_HB:
							 delayHB = payload_ptr[0];
							break;
						case CMD_CALIBRATE:
							calibration_ready=0;
							 break;
					}
					memset(rx_buffer_uart, 0, Size);
					break; // Salimos del for
				}
			}
		}
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
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    uint8_t mpu_wake = 0;
    HAL_I2C_Mem_Write(&hi2c1, (0x68 << 1), 0x6B, 1, &mpu_wake, 1, 100);
    if (SSD1306_Init() != 1) { // OJO: Verificá si tu librería devuelve 1 o 0 en éxito
        Error_Handler();
    }
    SSD1306_Fill(SSD1306_COLOR_BLACK);
    SSD1306_UpdateScreen();
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_Delay(2000);
    MPU6050_Calibrate();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer_uart, 256);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (HAL_GetTick() - lastTime0 > 50) {
		// Este if solo puede utilizarse para actualizar datos para mostrar por pantalla y
		// no para calcular nada por que no es confiable
	   lastTime0 = HAL_GetTick();
	   DataToQt(); //llamada cada 50ms
	   counter1++;
	   if(counter1 > 4)
		   flagDisplay=1;
	   	   MPU6050_Calibrate();	// solo se llamará si la bandera dentro de la funcion esta activa
	   }
	if(flagDisplay){
		flagDisplay=0;
		char msg[20];
//		SSD1306_Fill(SSD1306_COLOR_BLACK); // Borra lo anterior [cite: 28]}
//		SSD1306_GotoXY(2, 15); // [cite: 36]
//		sprintf(msg, "OUT:%.2f", salida);
//		SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE); // [cite: 40]
//		SSD1306_UpdateScreen(); // Fundamental para que se vea el cambio [cite: 26]
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  htim2.Init.Period = 3599;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3599;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|MOTB_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10|MOTB_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 MOTB_IN1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|MOTB_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 MOTB_IN2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|MOTB_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
