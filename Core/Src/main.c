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
  *
  *	  ============= [ Cómo usar sensores ] =============
  *	  IR:
  *	  entonces en resumen, llamo a la funcion Iniciar_Calibracion_Linea. luego tengo que
  *	  hacer pasar todos los sensores por la pista y la linea negra un par de veces y
  *	  finalmente llamar a la funcion Finalizar_Calibracion_Linea. Por otra parte,
  *	  Leer_Linea_Digital se llama constantemente y carga los estados de los sensores .
  *   ============= [ Actualizaciones ] =============
  *   15-2:
  *   Se implementaron los comandos para la comunicación inalambrica entre el robot
  *   y la interfaz Qt. Los mismos se componen de 2 partes: CMD name y Param. Algunos
  *   comandos no tienen parámetros
  *
  *   16-2:
  *   Se implementará la opción de utilizar diferentes filtros para los valores del
  *   MPU6050.
  *
  *   ============= [ BITÁCORA DE PROBLEMAS CON EL  PID del balanceo ] =============
  *		1. giro rate y accel giro deben tener el mismo signo en la siguiente fórmula: angle_y = alpha * (angle_y + gyro_rate * DT_PID) + (DT_PID * accel_angle);
  *		COnfirmo que tienen el mismo signo al inclinar el robot
  *		2. Montaje y Ejes: Las aceleraciones y giros estan todas en el sentido correcto para calcular la respuesta confirmado
  *		3. La inicialización del MPU no genera problemas. El retardo generado por el filtro pasabajos es menor al tiempo entre muestra y la escala de giroscopio y acelerómetro esta bien
  *		4. el problema no esta en la función Robot_Drive
  *		.
  *
  *
  *  El sistema de navegación para el modo follow line utiliza una barra de 4 sensores TCRT5000 digitalizados mediante umbrales individuales calibrados en el arranque (punto medio entre blanco y negro), asignando pesos simétricos de -3, -1, 1 y 3 a cada sensor para calcular un error de posición mediante el promedio ponderado de los sensores que detectan la línea; este error alimenta un PID de Yaw independiente donde el término proporcional ($P$) busca centrar la línea y el término derivativo ($D$) utiliza la velocidad angular real del Giroscopio en Z como amortiguador para suavizar los giros, resultando en un valor de steering que se suma al output de equilibrio en el motor izquierdo y se resta en el derecho, permitiendo que el robot avance con un setpoint de inclinación constante (1° o 2°) mientras corrige su trayectoria diferencialmente sin necesidad de encoders.
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
#include "math.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//Al usar __attribute__((packed)), garantizamos que no haya "padding" (huecos de memoria)
typedef struct __attribute__((packed)) {
    int16_t     acc_x, acc_y, acc_z;    // 6 bytes - Datos crudos
    int16_t     gyro_pitch, gyro_yaw;
    float       pitch_filtrado;         // 4 bytes - Pitch filtrado
    float       yaw_filtrado;           // 4 bytes - Yaw filtrado
    float       pos_x;                  // 4 bytes - Trayectoria X
    float       pos_y;                  // 4 bytes - Trayectoria Y
    float       velocidad;              // 4 bytes - Velocidad lineal
    uint8_t     modo;                   // 1 byte  - IDLE, FOLLOW_LINE, RC
    uint16_t    IR[8];                  // 16 bytes- Sensores IR
    uint8_t     infoAdicional;          // 1 byte  - Info Adicional
} PayloadData_t;

typedef union {
    PayloadData_t 	data;
    uint8_t 		buffer[sizeof(PayloadData_t)]; // 27 bytes totales
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
	MODO_IDDLE 			= 	0,
	MODO_RC				=	1,
	MODO_FOLLOWLINE 	= 	2
};
enum {
    // Sistema y Heartbeat
	CMD_ALIVE       			= 0, 		/*!< Busca confirmar conexión inalambrica						*/
    CMD_ACK 	      			= 1, 		/*!< Confirma conexión inalambrica								*/
    CMD_SET_HB      			= 2, 		/*!< Configurar intervalo de Heartbeat							*/
	CMD_CHANGE_MODE				= 3,		/*!< Cambiar entre los modos  IDLE, FOLLOW_LINE, RC				*/
    CMD_CALIBRATE   			= 5, 		/*!< Calibración de MPU6050										*/
    CMD_START       			= 6, 		/*!< Activar motores / Inicio de balanceo						*/
    CMD_STOP        			= 7, 		/*!< Parada de emergencia / Motores a 0							*/
	CMD_TCP_CONNECTED			= 8,		/*!< Comando utilizado en la conexión del TCP y el robot		*/
	CMD_CHANGE_OLED_SCREEN  	= 9,		/*!< Cambia o apaga la pantalla OLED							*/
	//RADIO CONTROL
    CMD_RC    					= 10, 		/*!< Movimiento manual (adelante, atrás, giros)					*/
	//MOTORES GENERAL
	CMD_CHANGE_DEADLINE_LEFT 	= 12,		/*!< Ajustar  Deadband del motor izquierdo						*/
	CMD_CHANGE_DEADLINE_RIGHT 	= 13,		/*!< Ajustar  Deadband del motor derecho						*/
	CMD_CHANGE_SETPOINT 		= 14, 		/*!< Ajustar  Setpoint de los motores*/
	CMD_ONOFFMOTORS 			= 15, 		/*!< Prender y apagar motores*/
	CMD_DATA 					= 16,		/*!< El robot manda datos de la unidad Sensitiva*/
	//PID SISTEMA BALANCEO
	CMD_PID_PITCH_KP      		= 20, 		/*!< Ajustar Término Proporcional del PID basado en grado de libertad Pitch*/
	CMD_PID_PITCH_KI      		= 21, 		/*!< Ajustar Término Integral del PID basado en grado de libertad Pitch*/
	CMD_PID_PITCH_KD      		= 22, 		/*!< Ajustar Término Derivativa del PID basado en grado de libertad Pitch*/
	CMD_PID_ALPHA 				= 23, 		/*!< Ajustar  Alpha del del filtro complementario utilizado en el PID basado en grado de libertad Pitch*/
	//PID SISTEMA DE GIRO
	CMD_PID_YAW_KP      		= 24, 		/*!< Ajustar Término Proporcional del PID basado en grado de libertad Yaw*/
	CMD_PID_YAW_KI      		= 25, 		/*!< Ajustar Término Integral del PID basado en grado de libertad Yaw*/
	CMD_PID_YAW_KD      		= 26, 		/*!< Ajustar Término Derivativa del PID basado en grado de libertad Yaw*/
	//SENSORES
	CMD_IR_INICIAR_CALIBRACION	= 27, 		/*!< Ajustar Término Proporcional del PID basado en grado de libertad Yaw*/
	CMD_IR_DETENER_CALIBRACION  = 28, 		/*!< Ajustar Término Integral del PID basado en grado de libertad Yaw*/
	//NETWORK
	CMD_NETWORK_CHANGE_SSID		= 30,
	CMD_NETWORK_CHANGE_PASSWORD	= 31
	   // CMD_TELEMETRY   			= 0xA0, 	/*!< Envío de ángulos, velocidad y sensores IR	*/
	   // CMD_LOG_MSG     			= 0xA1,  	/*!< Envío de mensajes de texto para debug		*/
};
typedef enum {
    FILTRO_COMPLEMENTARIO = 0,	/*!< Predeterminado */
    FILTRO_KALMAN = 1,			/*!< Presenta matemática más compleja que el filtro complementario */
    FILTRO_SOLO_ACCEL = 2 		/*!< Desactivar filtro */
} FiltroTipo_t;

typedef struct {
    uint16_t duration;  // Cuánto tiempo suena (ms)
    uint16_t interval;  // Cuánto tiempo de silencio entre beeps (ms)
    uint8_t repeat;    // Cuántos beeps faltan por sonar
    uint32_t last_tick; // Auxiliar para el tiempo
    uint8_t state;      // 0 = Silencio, 1 = Sonando
} Buzzer_Seq_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ================= [ PID ] ================= //
//#define 	ALPHA_PID 			0.98f    // Suaviza las vibraciones del acelerómetro
#define 	DT_PID 				0.01f
// ================= [ Comunicación ] ================= //
#define 	RX_BUFFER_SIZE 		64          // Suficiente para la IP y futuros comandos UNER
#define 	UNER_HEADER_STR 	"UNER"
#define 	UNER_TOKEN      	':'    // 0x3A según tu PDF o preferencia
// ================= [ Periféricos ] ================= //
#define 	MPU6050_ADDR 	(0x68 << 1) // Dirección I2C desplazada
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// ================= [ Typedef ] ================= //
			PayloadUNER_t 	telemetria;
			Buzzer_Seq_t 	hBuzzer = {0}; // Inicializamos en cero
			FiltroTipo_t 	currentlySelectedFilter = 	FILTRO_COMPLEMENTARIO; //FILTRO_COMPLEMENTARIO;
// ================= [ Variables generales ] ================= //
			uint16_t 		delayHB	= 50; //ENTRE 1 Y 200
			uint32_t 		lastTime0 = 0; // tiempo en el while 1
// ================= [ Flags ] ================= //
volatile 	uint8_t 		flagPID 				= 	0;
volatile 	uint8_t 		flagDisplay				=	0;
volatile	uint8_t			flagSendUNER 			= 	0;
volatile	uint8_t			flagDataToQt 			= 	0;
volatile	uint8_t			flagWIFI				=   0;
volatile	uint8_t 		flagOLED 				= 	0;
volatile	uint8_t			flagMotorsAreOn 		=	0;
volatile	uint8_t 		flagCalibrationIsReady 	= 	0; // Bandera para no activar el PID antes de tiempo
volatile	uint8_t			flag_RC_active			=	0;
// ================= [ Counters ] ================= //
			uint16_t 		contador = 0;
volatile 	uint32_t 		counterHB=0;				/*!< Utilizado en la interrupción del Timer 4 para manejar el HeartBit*/
volatile 	uint32_t 		counterDataToQt=0;				/*!< Utilizado en la interrupción del Timer 4 para manejar los datos mandados a Qt*/
			uint8_t  		counter1=0;				/*!< Utilizado para refrezcar la pantalla OLED*/
//RECEPCION DE DATOS
			char 			rx_buffer[20];
			uint8_t 		rx_index = 0;
			uint8_t 		rx_data;
// Nuevas variables para compensar la diferencia entre motores
			int16_t 		deadband_L = 55;			/*!< Zona Muerta del PWM para el motor 1*/
			int16_t 		deadband_R = 1; 			/*!< Zona Muerta del PWM para el motor 2*/
// =================[ Variables de Control PID ] ================= //
			float 			Kp = 155.0f;				/*!< Término Proporcional: [30] Si hay inclinación aplica una fuerza proporcional. Si se usara solo P, el robot oscilaría de un lado a otro sin quedarse quieto.*/
			float 			Ki = 1700.0f;					/*!< Término Integrativo: Elimina el error de estado estacionario*/
			float 			Kd = 4.0f;					/*!< Término Derivativo: [1.5] mide la velocidad a la que está cambiando el error. Actúa como un amortiguador*/
			float 			setpoint = 6.0f; // 4.0f;		/*!< Set Point, el punto en el qeu el robot queda a vertical*/
			float 			integral = 0;
			float 			last_error = 0;
			float           ALPHA_PID = 0.98f;
// =================[ Variables del Filtro del MPU6050 ] =================//
			float 			angle_y 	= 0;
// =================[ Variables de Calibración ] =================//
			float 			accel_bias_x;
			float 			accel_bias_y ;
			float 			accel_bias_z ;
			float 			gyro_bias_x ;
			float 			gyro_bias_y ;
			float 			gyro_bias_z ;
// =================[ Buffers de Sensores ] =================//
			uint8_t			mpu_data[14]; // Los 14 bytes que trae el DMA
			uint16_t 		adc_buffer[8]; // El buffer que llena el DMA
// =================[ Modo Radio Control ] =================//
volatile 	float 			RC_setpoint = 0;
volatile 	float 			RC_slow_setpoint = 0;
volatile 	int16_t   		RC_steering = 0;
float 		paso = 0.1f; // Velocidad de inclinación
// =================[ Protocolo UNER ] =================//
volatile 	uint16_t 		accelx=0;	/*!< Utilizado para refrezcar la pantalla OLED*/
volatile 	uint16_t 		accely=0;
volatile 	uint16_t		accelz=0;
volatile 	float 			giro=0;
volatile 	float			giro_z=0;
volatile 	float			accelGiro=0;
volatile 	float			salida=0;
// =================[ I2C Scheduler ] =================//
volatile 	uint8_t 		oled_update_requested = 0;
volatile 	uint8_t 		oled_current_page = 0;
volatile 	uint8_t 		oled_is_busy = 0; // Para saber si el display está ocupado

// =================[ Digitalizador del IR ] =================//
uint16_t 	adc_raw[4];
uint8_t 	estado_sensores[4];
uint16_t 	sensor_min[4];
uint16_t 	sensor_max[4];
uint16_t 	sensor_threshold[4];
uint8_t 	flag_calibrando_linea = 0; // Para saber en qué estado estamos
// recibidos desde el qt
uint8_t rx_buffer_uart[256];
char msg[20];
uint8_t BS=0;
// Banderas y contadores
volatile int16_t axRaw, ayRaw, azRaw, gyPitchRaw, gzYawRaw;
// --- Variables para los Filtros IIR ---
float ax_lpf = 0.0f;
float az_lpf = 0.0f;
float gy_hpf = 0.0f;
float gy_prev_raw = 0.0f;
// --- Constantes de Sintonía ---
volatile uint32_t tiempo_anterior_pid = 0;
volatile uint32_t delta_t_medido = 0;
volatile int16_t axRaw, ayRaw, azRaw, gyPitchRaw, gzYawRaw;

volatile uint32_t timeout_rc=0;



char ip_address[16];               // Buffer para guardar el string "192.168.XXX."
volatile uint8_t ip_received_flag = 0; // Bandera para avisar al loop principal
uint8_t esperando_digitos_ip = 0; // Bandera para nuestra mini máquina de estados

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

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
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/**
 * @brief buzzerSecuence:  				Máquina de estados para señales auditivas (Buzzer) no-bloqueante.
 * @param seq:			   				Estructura con tiempos de duración, intervalo y repeticiones.
 */
void buzzerSecuence(Buzzer_Seq_t *seq);
/**
 * @brief BS_tcpConnectSecuence:		Secuencia para la conexión del servidor
 */
void BS_tcpConnectSecuence();
/**
 * @brief BS_Error:						Secuencia para indicar un error general
 */
void BS_Error();
/**
 * @brief BS_ACK_NOT_FOUND: 			Secuencia para indicar un error al no recibir el ACK
 */
void BS_ACK_NOT_FOUND();
void screenScheduler(void);
void Iniciar_Calibracion_Linea(void);
void Iniciar_Calibracion_Linea(void);
void Procesar_Calibracion_Linea(void);
void Leer_Linea_Digital(void);
/**
 * @brief PID_PITCH:					Calcula la salida del controlador PID para el equilibrio.
 * @details 							Crea una respuesta en forma de impulso con los motores la cual es proporcional
 * 										al error (pitch) del robot.	Implementa un filtro complementario para fusionar
 * 										acelerómetro y giroscopio.
 * @note 								Frecuencia de ejecución dependiente de la llegada de datos del MPU (100Hz nominal).
 */
void PID_PITCH(void);

void PIDYAW();
/**
 * @brief sendCMD:						Envía un comando bajo el Protocolo UNER vía UART DMA.
 * @param cmd: 							Código del comando (CMD)
 * @param param: 						Parámetro de 16 bits (enviado en Little Endian ).
 */

void sendCMD(uint8_t cmd, uint16_t param);
/**
 * @brief DataToQt:						Empaqueta y envía la telemetría completa hacia la interfaz Qt.
 * @details 							Envía aceleración (X, Y, Z), giro, posición y estado de los 8 sensores IR
 * 										Utiliza HAL_UART_Transmit_DMA para no bloquear el bucle de control
 */
void DataToQt();
/**
 * @brief Robot_Drive:					Controla el puente H TB6612FNG para el movimiento de las ruedas.
 * @param speed_L: 						Velocidad motor izquierdo (-3599 a 3599).
 * @param speed_R: 						Velocidad motor derecho (-3599 a 3599).
 * @note 								Aplica deadband para vencer la inercia mecánica de los motores.
 */
void Robot_Drive(int16_t speed_L, int16_t speed_R);
/**
 * @brief MPU6050_Init:					Inicializa el MPU6050 con configuración específica para equilibrio.
 * @details								Configura Full Scale: Accel +/- 2g, Gyro +/- 500 dps y DLPF a 42Hz.
 */
void MPU6050_Init(I2C_HandleTypeDef *hi2c);
/**
 * @brief MPU6050_Calibrate:			Función calibrante del MPU6050
 * @details 							Calibra el MPU6050 cargado datos a las variables terminadas en byas para restarla
 * 										a las mediciones realizadas con el sensor
 */
void MPU6050_Calibrate(void);
/**
 * @brief HAL_TIM_PeriodElapsedCallback:Interrupción provocada por un timer
 * @param htim: 						manejador al timer utilizado
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/**
 * @brief HAL_I2C_MemRxCpltCallback:	Callback
 * @details
 * @param hi2c			manejador del I2C
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
/**
 * @brief HAL_I2C_MasterTxCpltCallback:
 * @details
 * @param
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
/**
 * @brief HAL_UARTEx_RxEventCallback	Callback de recepción UART por evento IDLE o Buffer lleno.
 * @details 							Gestiona la recepción de la IP del ESP-01 y el parseo de comandos UNER.
 * 										Implementa la verificación de Checksum mediante operación XOR[cite: 81].
 * @param Size Cantidad de bytes recibidos.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
/**
 * @brief HAL_UART_ErrorCallback
 * @details
 * @param huart:
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Función pura para el filtro
void buzzerSecuence(Buzzer_Seq_t *seq) {
    if (seq->repeat == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // Asegurar apagado
        return;
    }
    uint32_t current_tick = HAL_GetTick();
    if (seq->state == 0 && (current_tick - seq->last_tick >= seq->interval)) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        seq->state = 1;
        seq->last_tick = current_tick;
    }
    else if (seq->state == 1 && (current_tick - seq->last_tick >= seq->duration)) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        seq->state = 0;
        seq->last_tick = current_tick;
        seq->repeat--; // Descontamos un beep
    }
}
void BS_tcpConnectSecuence() {
    hBuzzer.duration = 100;  // Beep corto de 100ms
    hBuzzer.interval = 50;   // Silencio de 50ms
    hBuzzer.repeat = 2;      // bip bip
    hBuzzer.state = 0;
    hBuzzer.last_tick = HAL_GetTick();
}
void BS_Error()
	{
    hBuzzer.duration = 500;  // Beep largo de 800ms
    hBuzzer.interval = 100;
    hBuzzer.repeat = 1;      //bip
    hBuzzer.state = 0;
    hBuzzer.last_tick = HAL_GetTick();
	}
void BS_ACK_NOT_FOUND(){
	hBuzzer.duration = 200;  // Beep largo de 800ms
	hBuzzer.interval = 50;
	hBuzzer.repeat = 3;
	hBuzzer.state = 0;
	hBuzzer.last_tick = HAL_GetTick();
}
void BS_NEWPARAM_OK() {
    hBuzzer.duration = 80;  // Beep corto de 100ms
    hBuzzer.interval = 50;   // Silencio de 50ms
    hBuzzer.repeat = 1;      // bip bip
    hBuzzer.state = 0;
    hBuzzer.last_tick = HAL_GetTick();
}
void BS_NEWPARAM_ISNOTOK() {
    hBuzzer.duration = 800;  // Beep corto de 100ms
    hBuzzer.interval = 1;   // Silencio de 50ms
    hBuzzer.repeat = 1;      // bip bip
    hBuzzer.state = 0;
    hBuzzer.last_tick = HAL_GetTick();
}
void screenScheduler(void){
	if (!oled_is_busy) {
		SSD1306_Fill(SSD1306_COLOR_BLACK);
			switch(flagOLED){
			case 0:
				sprintf(msg, "IP: %s",ip_address);
				SSD1306_GotoXY(1, 0);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

				sprintf(msg, "Dig  | Min | Max");
				SSD1306_GotoXY(2, 10);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);


				sprintf(msg, "0:%d", estado_sensores[0]);
				SSD1306_GotoXY(1, 20);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

				sprintf(msg, "| %d", sensor_min[0]);
				SSD1306_GotoXY(1, 30);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

				sprintf(msg, "| %d |", sensor_max[0]);
				SSD1306_GotoXY(1, 40);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);





//				sprintf(msg, "1:  %d", estado_sensores[1]);
//				SSD1306_GotoXY(1, 30);
//				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
//
//				sprintf(msg, "2:  %d", estado_sensores[2]);
//				SSD1306_GotoXY(1, 40);
//				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
//
//				sprintf(msg, "3:  %d", estado_sensores[3]);
//				SSD1306_GotoXY(1, 50);
//				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

//
//				sprintf(msg, "|%d", sensor_min[0]);
//				SSD1306_GotoXY(1, 20);
//				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
//				sprintf(msg, "|%d", estado_sensores[1]);
//				SSD1306_GotoXY(1, 30);
//				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
//
//				sprintf(msg, "|| S th  %d", estado_sensores[2]);
//				SSD1306_GotoXY(1, 40);
//				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);




				oled_update_requested = 1;
				break;
			case 1:

				sprintf(msg, "AdicionalInfoScreen");
				SSD1306_GotoXY(0, 0);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
				sprintf(msg, "Kp:%.2f",Kp);
				SSD1306_GotoXY(1, 20);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
				sprintf(msg, "Kd:%.2f",Kd);
				SSD1306_GotoXY(1, 30);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
				sprintf(msg, "Ki:%.2f",Ki);
				SSD1306_GotoXY(1, 40);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);


//					sprintf(msg, "SP:%.1f", setpoint);
//					SSD1306_GotoXY(1, 30);
//					SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
//					sprintf(msg, "AP:%.3f", ALPHA_PID);
//					SSD1306_GotoXY(1, 40);
//					SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
//					sprintf(msg, "dm: %ld", delta_t_medido);
//					SSD1306_GotoXY(60, 0);
//					SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
//					sprintf(msg, "AG:%.2f", accelGiro);
//					SSD1306_GotoXY(60, 10);
//					SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
//					sprintf(msg, "AX: %.0d", axRaw);
//					SSD1306_GotoXY(60, 30);
//					SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
//					sprintf(msg, "AY:%.0d", ayRaw);
//					SSD1306_GotoXY(60, 40);
//					SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
//					sprintf(msg, "AZ:%.0d", azRaw);
//					SSD1306_GotoXY(60, 50);
//					SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
				SSD1306_GotoXY(1, 20);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
				sprintf(msg, "DBL: %d",deadband_L);
				SSD1306_GotoXY(1, 30);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
				sprintf(msg, "DBR: %d",deadband_R);
				if (flagWIFI) {
					// Dibujamos el icono de WiFi (podes usar lineas o circulos)
					SSD1306_Clear();
					SSD1306_DrawLine(110, 8, 114, 4, SSD1306_COLOR_WHITE); // Onda 1
					SSD1306_DrawLine(114, 4, 118, 8, SSD1306_COLOR_WHITE);
					SSD1306_DrawLine(112, 10, 116, 10, SSD1306_COLOR_WHITE); // Punto base
				} else {
					// Icono tachado o texto simple
					SSD1306_Clear();
					SSD1306_GotoXY(105, 2);
					SSD1306_Puts("X", &Font_7x10, SSD1306_COLOR_WHITE);
					SSD1306_DrawLine(105, 2, 120, 12, SSD1306_COLOR_WHITE); // Tachado
				}
				oled_update_requested = 1;// NO llamamos a UpdateScreen(). Simplemente levantamos la bandera para el Scheduler.
				break;
			case 2:
				sprintf(msg, "IR and MPU Screen ");
				SSD1306_GotoXY(2, 0);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
				for (int i = 0; i < 8; i++) {
				        // 1. Calculamos el ancho de la barra (Supongamos 50 píxeles de ancho máximo)
				        // Mapeo: (Valor ADC * Ancho Máximo) / 4095
				        uint8_t barWidth = (uint8_t)((adc_buffer[i] * 50) / 4095);
				        uint8_t yPos = 10 + (i * 7);
				        if(i==0 || i == 7){
				        	char label[4];
							sprintf(label, "IR%d", i);
							SSD1306_GotoXY(2, yPos);
							SSD1306_Puts(label, &Font_7x10, SSD1306_COLOR_WHITE);
				        }
				        SSD1306_DrawRectangle(25, yPos, 50, 5, SSD1306_COLOR_WHITE);
				        SSD1306_DrawFilledRectangle(25, yPos, barWidth, 5, SSD1306_COLOR_WHITE);
				    }
				    oled_update_requested = 1; // Le avisamos al scheduler que mande los datos al OLED
				break;
			}
	    }
}
void Iniciar_Calibracion_Linea(void) {
    for(int i = 0; i < 4; i++) {
        sensor_min[i] = 4095; // Valor máximo del ADC
        sensor_max[i] = 0;    // Valor mínimo del ADC
    }
    flag_calibrando_linea = 1;
}
void Procesar_Calibracion_Linea(void) {
    if (flag_calibrando_linea) {
        for(int i = 0; i < 4; i++) {
            // Buscamos si hay un nuevo récord de valor bajo (Blanco)
            if(adc_raw[i] < sensor_min[i]) {
                sensor_min[i] = adc_raw[i];
            }
            // Buscamos si hay un nuevo récord de valor alto (Negro)
            if(adc_raw[i] > sensor_max[i]) {
                sensor_max[i] = adc_raw[i];
            }
        }
    }
}
void Leer_Linea_Digital(void) {
    for(int i = 0; i < 4; i++) {
        // Si el valor analógico superó la mitad, está viendo la línea
        if(adc_raw[i] > sensor_threshold[i]) {
            estado_sensores[i] = 1; // NEGRO
        } else {
            estado_sensores[i] = 0; // BLANCO
        }
    }
}
void Finalizar_Calibracion_Linea(void) {
    flag_calibrando_linea = 0;
    for(int i = 0; i < 4; i++) {
        // El punto medio perfecto entre lo más blanco y lo más negro que vio
        sensor_threshold[i] = (sensor_max[i] + sensor_min[i]) / 2;
    }
}
void PID_PITCH(void){
		float gyro_rate = -(((float)gyPitchRaw / 131.0f)); // 65.5f));
		float accel_angle = (atan2f((float)axRaw , (float)azRaw ) * 57.2957f) ;
		giro 	= gyro_rate;
		accelGiro = accel_angle;
		angle_y = ALPHA_PID * (angle_y + gyro_rate * DT_PID) + (1.0f - ALPHA_PID) * accel_angle;
			telemetria.data.pitch_filtrado 	= 	angle_y;
			accelx 	= axRaw;
			accely 	= ayRaw;
			accelz 	= azRaw;
	  // if (RC_slow_setpoint < RC_setpoint) RC_slow_setpoint += paso;
	  // if (RC_slow_setpoint > RC_setpoint) RC_slow_setpoint -= paso;
//	   float error = angle_y - (setpoint + RC_slow_setpoint);
	   float error = angle_y - (setpoint + RC_setpoint);//	   float error = angle_y - setpoint;
	   integral += error * DT_PID;
	   if(integral > 2000) integral = 2000;
	   else if(integral < -2000) integral = -2000;
	   float P =  Kp * error;
	   float I =  Ki * integral;
	   float D = Kd * (error - last_error) / DT_PID; //Kd * gyro_filtrado_ema; //Kd * (error - last_error) / DT_PID;//float D =  Kd * gyro_rate; //
	   float output = P + I + D ; // Funcion de transferencia
	   last_error = error;
	   if(flagMotorsAreOn){
		   int16_t outputLeft = (int16_t)output + RC_steering;
		   int16_t outputRigth = (int16_t)output - RC_steering;
		   Robot_Drive(outputLeft, outputRigth);
		//   Robot_Drive((int16_t)output, (int16_t)output);
	   }
	   if(flagMotorsAreOn==0||angle_y > 50||angle_y < -50)	   Robot_Drive(0, 0);
}
void PIDYAW(){

}
void sendCMD(uint8_t cmd, uint16_t param) {
    uint8_t frame[10];
    memcpy(&frame[0], "UNER", 4);     // [0-3] Header
    frame[4] = 4;                     //
   // frame[5] = 0xFD;                  // [5] Token
    frame[5] = ':';                  // [5] Token
    frame[6] = cmd;                   // [6] CMD
    frame[7] = (uint8_t)(param & 0xFF);        // Byte Menos Significativo (LSB)
    frame[8] = (uint8_t)((param >> 8) & 0xFF); // Byte Más Significativo (MSB)
    uint8_t checksum = 0;
    for(int i=0; i<9; i++) {
        checksum ^= frame[i];
    }
    frame[9] = checksum;              // [9] Checksum al final
    HAL_UART_Transmit_DMA(&huart1, frame, 10);
}
void DataToQt(){
    if (huart1.gState != HAL_UART_STATE_READY) return; // El UART está ocupado
    flagSendUNER = 0;
    telemetria.data.acc_x       = (int16_t)accelx;
    telemetria.data.acc_y       = (int16_t)accely;
    telemetria.data.acc_z       = (int16_t)accelz;
    telemetria.data.gyro_yaw    = (int16_t)giro_z;
    telemetria.data.yaw_filtrado= 1;
    telemetria.data.pos_x       = 3.0f;
    telemetria.data.pos_y       = 2.0f;
    telemetria.data.velocidad   = 1.0f;
    for(uint8_t i=0; i<8; i++)       telemetria.data.IR[i] = adc_buffer[i];
    telemetria.data.modo = MODO_FOLLOWLINE;
    telemetria.data.infoAdicional = 1;
    static uint8_t frame[56];
    memcpy(&frame[0], "UNER", 4);
    frame[4] = 50;                  // Length = CMD(1) + Payload(48) + CHK(1) = 50
    frame[5] = ':';                 // TOKEN
    frame[6] = CMD_DATA;                  // CMD_DATA
    memcpy(&frame[7], telemetria.buffer, 48); // Copiamos los 48 bytes del payload
    uint8_t checksum = 0;
    for (int i = 0; i < 55; i++) {  // XOR de los primeros 55 bytes (índices 0 al 54)
        checksum ^= frame[i];
    }
    frame[55] = checksum;           // Guardamos el checksum en el último byte (índice 55)
    HAL_UART_Transmit_DMA(&huart1, frame, 56); // Transmitimos los 56 bytes
}
void Robot_Drive(int16_t speed_L, int16_t speed_R) {
	    if (speed_L > 0) speed_L += deadband_L;			// Aplicar Deadband (Zona muerta)
	    else if (speed_L < 0) speed_L -= deadband_L;
	    if (speed_R > 0) speed_R += deadband_R;
	    else if (speed_R < 0) speed_R -= deadband_R;
	    if (speed_L > 3599) speed_L = 3599; 			// Establecemos límites
	    if (speed_L < -3599) speed_L = -3599;
	    if (speed_R > 3599) speed_R = 3599;
	    if (speed_R < -3599) speed_R = -3599;
	if (speed_L == 0) { // Motor 1 (PA8, PA9, PB4)
		HAL_GPIO_WritePin(GPIOA, MOT1_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, MOT1_IN2_Pin, GPIO_PIN_RESET);
		// __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)speed_L);
	   } else {
		if (speed_L >= 0) { // Motor 1 (PA8, PA9, PB4)
			HAL_GPIO_WritePin(GPIOA, MOT1_IN1_Pin,  GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, MOT1_IN2_Pin,  GPIO_PIN_RESET  );
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)speed_L);
			} else {
				HAL_GPIO_WritePin(GPIOA, MOT1_IN1_Pin, GPIO_PIN_RESET );
				HAL_GPIO_WritePin(GPIOA, MOT1_IN2_Pin,  GPIO_PIN_SET);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)(-speed_L));
			}
	   	   }
	if (speed_R == 0) {// Motor 2 (PB3, PA15, PB5)
	    	HAL_GPIO_WritePin(GPIOB, MOT2_IN1_Pin, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, MOT2_IN2_Pin, GPIO_PIN_RESET);
	      //  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)speed_R);
	    } else {
		if (speed_R >= 0) {// Motor 2 (PB3, PA15, PB5)
			HAL_GPIO_WritePin(GPIOB, MOT2_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, MOT2_IN2_Pin, GPIO_PIN_SET  );
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)speed_R);
		} else {
			HAL_GPIO_WritePin(GPIOB, MOT2_IN1_Pin, GPIO_PIN_SET  );
			HAL_GPIO_WritePin(GPIOA, MOT2_IN2_Pin,  GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)(-speed_R));
		}
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
        // 4. Configurar Giroscopio (+/- 250 dps)
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1B, 1, &data, 1, 100);
        data = 0x04; // Filtro de ~42Hz. Limpia basura del sensor. 0x02 agrega un retardo de 2 ms a la medicion el cual se suma al retardo de la lectura
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1A, 1, &data, 1, 100);
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x19, 1, &data, 1, 100);
    }
}
void MPU6050_Calibrate(void) {
    if (flagCalibrationIsReady == 0) {
        int32_t axS = 0, ayS = 0, azS = 0;
        int32_t gxS = 0, gyS = 0, gzS = 0;
        int num_samples = 200;
        uint8_t buffer[14];
        for (int i = 0; i < num_samples; i++) {
            if (HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x3B, 1, buffer, 14, 100) != HAL_OK) {
                Error_Handler(); // O prendé un LED rojo para avisarte
            }
            axS += (int16_t)(buffer[0] << 8 | buffer[1]);
            ayS += (int16_t)(buffer[2] << 8 | buffer[3]);
            azS += (int16_t)(buffer[4] << 8 | buffer[5]);
            gxS += (int16_t)(buffer[8] << 8 | buffer[9]);
            gyS += (int16_t)(buffer[10] << 8 | buffer[11]);
            gzS += (int16_t)(buffer[12] << 8 | buffer[13]);
            HAL_Delay(11);
        }
        accel_bias_x = (float)axS / num_samples;
        accel_bias_y = (float)ayS / num_samples;
        accel_bias_z = ((float)azS / num_samples) - 16384.0f;	// se le resta el valor de la gravedad en crudo
      //  gyro_bias_x = (float)gxS / num_samples;
        gyro_bias_y = (float)gyS / num_samples;
       // gyro_bias_z = (float)gzS / num_samples;
        flagCalibrationIsReady = 1;
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/*
	 * Frecuencia del Timer: 72MHz
	 * Conteo hasta el periodo (ARR): 999
	 * Prescaler : 719
	 * Resultado: interrupcion cada 10ms
	 * */
    if (htim->Instance == TIM4) {
    	if (hi2c1.State == HAL_I2C_STATE_READY) {
    		HAL_I2C_Mem_Read_DMA(&hi2c1, (0x68 << 1), 0x3B, 1, mpu_data, 14); // los datos tardan 0.38ms en ser leidos + 2ms retardo. Son 153 bits a 400k bits/s
    	}
        counterHB++;
        counterDataToQt++;
        if(counterHB > delayHB){
            counterHB = 0;
          //  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Heartbeat LED
        	}
        if(counterDataToQt > 10){
        	counterDataToQt = 0;
        	flagDataToQt=1;
        	flagDisplay=1;
        	}

    }
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C1) {
			axRaw 		= (int16_t)(mpu_data[0] << 8 | mpu_data[1]);
			ayRaw 		= (int16_t)(mpu_data[2] << 8 | mpu_data[3]);
			azRaw 		= (int16_t)(mpu_data[4] << 8 | mpu_data[5]);
			gyPitchRaw 	= (int16_t)(mpu_data[10] << 8 | mpu_data[11]);
			gzYawRaw 	= (int16_t)(mpu_data[12] << 8 | mpu_data[13]);
			flagPID = 1;
	        if (oled_update_requested) {
				oled_is_busy = 1; // Bloqueamos el while(1) para que no pise la memoria
				SSD1306_UpdatePage_DMA(oled_current_page);
				oled_current_page++;				// Incrementamos para la próxima vez
				if (oled_current_page >= 8) {
					oled_current_page = 0;// Ya mandamos toda la pantalla. Terminamos el proceso.
					oled_update_requested = 0;
				}
			}
    }
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C1) {
        if (oled_current_page == 0 && !oled_update_requested) {
            oled_is_busy = 0; // Liberamos para que el while(1) pueda armar el siguiente frame
        }
    }

}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {

        // --- 1. LÓGICA DE CAPTURA DE IP (TEXTO PLANO) ---
        if (esperando_digitos_ip == 1) {
            uint16_t ip_len = (Size > 15) ? 15 : Size;
            memset(ip_address, 0, sizeof(ip_address));
            strncpy(ip_address, (char*)rx_buffer_uart, ip_len);

            // Limpieza de caracteres de control
            for(int i = 0; i < 16; i++){
                if(ip_address[i] == '\r' || ip_address[i] == '\n' || ip_address[i] == ' '){
                    ip_address[i] = '\0';
                    break;
                }
            }
            ip_received_flag = 1;
            esperando_digitos_ip = 0;
            // No retornamos aquí por si hay un paquete UNER pegado justo después
        }
        else if (strncmp((char*)rx_buffer_uart, "IP:", 3) == 0) {
            if (Size == 3) {
                esperando_digitos_ip = 1;
            } else {
                uint16_t ip_len = (Size - 3 > 15) ? 15 : (Size - 3);
                memset(ip_address, 0, sizeof(ip_address));
                strncpy(ip_address, (char*)(rx_buffer_uart + 3), ip_len);
                // Limpieza...
                for(int i = 0; i < 16; i++){
                    if(ip_address[i] == '\r' || ip_address[i] == '\n'){
                        ip_address[i] = '\0';
                        break;
                    }
                }
                ip_received_flag = 1;
            }
            // Una vez procesada la IP, podríamos limpiar o continuar si hay más datos
        }

        // --- 2. LÓGICA DE PROTOCOLO UNER (BINARIO) ---
        // Usamos un for para recorrer el buffer buscando la cabecera 'UNER'
        for (int i = 0; i <= (Size - 7); ) {
            if (rx_buffer_uart[i]   == 'U' && rx_buffer_uart[i+1] == 'N' &&
                rx_buffer_uart[i+2] == 'E' && rx_buffer_uart[i+3] == 'R') {

                uint8_t len = rx_buffer_uart[i+4];
                // El protocolo indica: Length = 1(CMD) + N(Payload) + 1(Checksum)
                // Por lo tanto, el checksum está en: i + 5 (Header+Len+Token) + len
                uint8_t pos_checksum = i + 5 + len;

                if (pos_checksum < Size) {
                    uint8_t checksum_recibido = rx_buffer_uart[pos_checksum];
                    uint8_t checksum_calc = 0;

                    for(int k = i; k < pos_checksum; k++) {
                        checksum_calc ^= rx_buffer_uart[k];
                    }

                    if (checksum_calc == checksum_recibido) {
                        uint8_t cmd = rx_buffer_uart[i+6];
                        uint8_t *payload_ptr = &rx_buffer_uart[i+7];
                        int16_t payloadInt16 = 0;
                        float   payloadFloat = 0;

                        switch(cmd){
               						case CMD_SET_HB:
               							 delayHB = payload_ptr[0];
               							 break;
               						case CMD_CALIBRATE:
               							 Robot_Drive(0, 0);
               							 flagCalibrationIsReady=0;
               							 break;
               						default:
               						case CMD_ALIVE:
               							sendCMD(CMD_ACK, 0); // te devuelvo un alive
               							break;
               						case CMD_ACK:
               							break;
               						case CMD_ONOFFMOTORS:
               							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
               							BS_NEWPARAM_OK();
               							flagMotorsAreOn = payloadInt16;
               							break;
               						case CMD_TCP_CONNECTED:
               							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
               							if(payloadInt16){	//conectado
               								BS_tcpConnectSecuence();
               								flagWIFI=1;
               							}
               							else if(payloadInt16 ==0){				//desconectado
               								BS_Error();
               								flagWIFI=0;
               							}
               							break;
               						case CMD_CHANGE_DEADLINE_LEFT:
               							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
               							deadband_L = payloadInt16;
               							BS_NEWPARAM_OK();
               							break;
               						case CMD_CHANGE_DEADLINE_RIGHT:
               							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
               							deadband_R = payloadInt16;
               							BS_NEWPARAM_OK();
               							break;
               						case CMD_CHANGE_OLED_SCREEN:
               							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
               							flagOLED = payloadInt16;
               							BS_NEWPARAM_OK();
               							break;
               						case CMD_PID_PITCH_KP:
               							memcpy(&payloadFloat, payload_ptr, sizeof(float));
               							Kp = payloadFloat;
               							BS_NEWPARAM_OK();
               							break;
               						case CMD_PID_PITCH_KD:
               							memcpy(&payloadFloat, payload_ptr, sizeof(float));
               							Kd = payloadFloat;
               							BS_NEWPARAM_OK();
               							break;
               						case CMD_PID_PITCH_KI:
               							memcpy(&payloadFloat, payload_ptr, sizeof(float));
               							Ki = payloadFloat;
               							BS_NEWPARAM_OK();
               							break;
               						case CMD_CHANGE_SETPOINT:
               							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
               							setpoint = payloadInt16;
               							BS_NEWPARAM_OK();
               							break;
               						case CMD_PID_ALPHA:
               							memcpy(&payloadFloat, payload_ptr, sizeof(float));
               							ALPHA_PID = payloadFloat;
               							BS_NEWPARAM_ISNOTOK();
               							break;
               						case CMD_RC:{
               							flag_RC_active =  payload_ptr[0];
               							if (flag_RC_active) {
               								RC_setpoint = (float)((int8_t)payload_ptr[1]) / 10.0f;
               								RC_steering = (int16_t)((uint16_t)payload_ptr[2] << 8 | (uint16_t)payload_ptr[3]);
               							} else {
               								RC_setpoint = 0;
               								RC_steering = 0;
               								}
               							}
               							break;
               						case CMD_IR_INICIAR_CALIBRACION:
               							Iniciar_Calibracion_Linea();
               						  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Heartbeat LED
               							break;
               						case CMD_IR_DETENER_CALIBRACION:
               							Finalizar_Calibracion_Linea();
               						  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Heartbeat LED
               							break;
               					}
                        i = pos_checksum + 1; // Saltamos al final del paquete procesado
                        continue;
                    }
                }
            }
            i++; // Si no hay cabecera, avanzamos un byte
        }

        // --- 3. REARME DEL DMA ---
        // Importante: No uses memset(rx_buffer_uart, 0, 256) antes de reiniciar si no estás seguro
        // de que no quedaron bytes de un paquete incompleto al final.
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer_uart, 256);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // Desactivar interrupción de Half Transfer
    }
}
/*
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        // Recorremos el buffer buscando cabeceras "UNER"
        for (int i = 0; i <= (Size - 7); ) {
            if (rx_buffer_uart[i]   == 'U' && rx_buffer_uart[i+1] == 'N' &&
                rx_buffer_uart[i+2] == 'E' && rx_buffer_uart[i+3] == 'R') {

                uint8_t len = rx_buffer_uart[i+4];
                uint8_t pos_checksum = i + 5 + len;

                // 1. Validar que el paquete completo esté dentro de lo recibido
                if (pos_checksum < Size) {
                    uint8_t checksum_recibido = rx_buffer_uart[pos_checksum];
                    uint8_t checksum_calc = 0;

                    for(int k = i; k < pos_checksum; k++) {
                        checksum_calc ^= rx_buffer_uart[k];
                    }

                    if (checksum_calc == checksum_recibido) {
                        uint8_t cmd = rx_buffer_uart[i+6];
                        uint8_t *payload_ptr = &rx_buffer_uart[i+7];
                    	int16_t payloadInt16=0;
                    	float 	payloadFloat=0;
                        switch(cmd){
						case CMD_SET_HB:
							 delayHB = payload_ptr[0];
							 break;
						case CMD_CALIBRATE:
							 Robot_Drive(0, 0);
							 flagCalibrationIsReady=0;
							 break;
						default:
						case CMD_ALIVE:
							sendCMD(CMD_ACK, 0); // te devuelvo un alive
							break;
						case CMD_ACK:
							break;
						case CMD_ONOFFMOTORS:
							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
							BS_NEWPARAM_OK();
							flagMotorsAreOn = payloadInt16;
							break;
						case CMD_TCP_CONNECTED:
							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
							if(payloadInt16){	//conectado
								BS_tcpConnectSecuence();
								flagWIFI=1;
							}
							else if(payloadInt16 ==0){				//desconectado
								BS_Error();
								flagWIFI=0;
							}
							break;
						case CMD_CHANGE_DEADLINE_LEFT:
							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
							deadband_L = payloadInt16;
							BS_NEWPARAM_OK();
							break;
						case CMD_CHANGE_DEADLINE_RIGHT:
							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
							deadband_R = payloadInt16;
							BS_NEWPARAM_OK();
							break;
						case CMD_CHANGE_OLED_SCREEN:
							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
							flagOLED = payloadInt16;
							BS_NEWPARAM_OK();
							break;
						case CMD_PID_PITCH_KP:
							memcpy(&payloadFloat, payload_ptr, sizeof(float));
							Kp = payloadFloat;
							BS_NEWPARAM_OK();
							break;
						case CMD_PID_PITCH_KD:
							memcpy(&payloadFloat, payload_ptr, sizeof(float));
							Kd = payloadFloat;
							BS_NEWPARAM_OK();
							break;
						case CMD_PID_PITCH_KI:
							memcpy(&payloadFloat, payload_ptr, sizeof(float));
							Ki = payloadFloat;
							BS_NEWPARAM_OK();
							break;
						case CMD_CHANGE_SETPOINT:
							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
							setpoint = payloadInt16;
							BS_NEWPARAM_OK();
							break;
						case CMD_PID_ALPHA:
							memcpy(&payloadFloat, payload_ptr, sizeof(float));
							ALPHA_PID = payloadFloat;
							BS_NEWPARAM_ISNOTOK();
							break;
						case CMD_RC:{
							flag_RC_active =  payload_ptr[0];
							if (flag_RC_active) {
								RC_setpoint = (float)((int8_t)payload_ptr[1]) / 10.0f;
								RC_steering = (int16_t)((uint16_t)payload_ptr[2] << 8 | (uint16_t)payload_ptr[3]);
							} else {
								RC_setpoint = 0;
								RC_steering = 0;
								}
							}
							break;
						case CMD_IR_INICIAR_CALIBRACION:
							Iniciar_Calibracion_Linea();
							break;
						case CMD_IR_DETENER_CALIBRACION:
							Finalizar_Calibracion_Linea();
							break;
					}
							i = pos_checksum + 1;
							continue;
						}
                }
            }
            i++;
        }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer_uart, 256);
}

*/
/*
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)	{
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
						int16_t payloadInt16=0;
						float 	payloadFloat=0;

						switch(cmd) {
							case CMD_SET_HB:
								 delayHB = payload_ptr[0];
							break;
						case CMD_CALIBRATE:
							 Robot_Drive(0, 0);
							 flagCalibrationIsReady=0;
							 break;
						default:
						case CMD_ALIVE:
							sendCMD(CMD_ACK, 0); // te devuelvo un alive
							break;
						case CMD_ACK:
							break;
						case CMD_ONOFFMOTORS:
							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
							BS_NEWPARAM_OK();
							flagMotorsAreOn = payloadInt16;
							break;
						case CMD_TCP_CONNECTED:
							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
							if(payloadInt16){	//conectado
								BS_tcpConnectSecuence();
								flagWIFI=1;
							}
							else if(payloadInt16 ==0){				//desconectado
								BS_Error();
								flagWIFI=0;
							}
							break;
						case CMD_CHANGE_DEADLINE_LEFT:
							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
							deadband_L = payloadInt16;
							BS_NEWPARAM_OK();
							break;
						case CMD_CHANGE_DEADLINE_RIGHT:
							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
							deadband_R = payloadInt16;
							BS_NEWPARAM_OK();
							break;
						case CMD_CHANGE_OLED_SCREEN:
							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
							flagOLED = payloadInt16;
							BS_NEWPARAM_OK();
							break;
						case CMD_PID_PITCH_KP:
							memcpy(&payloadFloat, payload_ptr, sizeof(float));
							Kp = payloadFloat;
							BS_NEWPARAM_OK();
							break;
						case CMD_PID_PITCH_KD:
							memcpy(&payloadFloat, payload_ptr, sizeof(float));
							Kd = payloadFloat;
							BS_NEWPARAM_OK();
							break;
						case CMD_PID_PITCH_KI:
							memcpy(&payloadFloat, payload_ptr, sizeof(float));
							Ki = payloadFloat;
							BS_NEWPARAM_OK();
							break;
						case CMD_CHANGE_SETPOINT:
							memcpy(&payloadInt16, payload_ptr, sizeof(int16_t));
							setpoint = payloadInt16;
							BS_NEWPARAM_OK();
							break;
						case CMD_PID_ALPHA:
							memcpy(&payloadFloat, payload_ptr, sizeof(float));
							ALPHA_PID = payloadFloat;
							BS_NEWPARAM_ISNOTOK();
							break;
						case CMD_RC:
						    flag_RC_active =  payload_ptr[0];
						    if (flag_RC_active) {
						        RC_setpoint = (float)((int8_t)payload_ptr[1]) / 10.0f;
						        RC_steering = (int16_t)((uint16_t)payload_ptr[2] << 8 | (uint16_t)payload_ptr[3]);
						  //      timeout_rc = 0; // Reset del watchdog
						    } else {
						        RC_setpoint = 0;
						        RC_steering = 0;
						    }
						    break;
					}
					memset(rx_buffer_uart, 0, Size);
					break; // Salimos del for
				}
			}
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer_uart, 256);
	}
}*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // Si hubo ruido o error de trama (común al arrancar el ESP)
    if (huart->Instance == USART1){
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	if (SSD1306_Init() != 1) 		Error_Handler();
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_UpdateScreen();
	HAL_Delay(100);
	MPU6050_Init(&hi2c1);
	HAL_Delay(500);
	MPU6050_Calibrate();
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 8) != HAL_OK) {
		Error_Handler();
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer_uart, 256);  // Preparamos la recepción DMA para la Comunicación Inalámbrica (ESP8266)[cite: 15].
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // (Asumo que es el Enable del TB6612FNG o un LED)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim4);
	//Kalman_Init(&kalman_pitch);

	Iniciar_Calibracion_Linea();

	Leer_Linea_Digital() ;
	Finalizar_Calibracion_Linea();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	buzzerSecuence(&hBuzzer);
	//Funciones INDEPENDIENTES al modo del robot

	if(flagDataToQt){	flagDataToQt = 0;	DataToQt();		}
	if(flagDisplay){	flagDisplay=0;		screenScheduler();}
	if(flagPID){
		flagPID = 0;
		Procesar_Calibracion_Linea();
		Leer_Linea_Digital();

		PID_PITCH();
	}
	switch(telemetria.data.modo){
	case MODO_IDDLE:
		break;
	case MODO_RC:
		break;
	case MODO_FOLLOWLINE:
		break;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
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
  htim4.Init.Prescaler = 719;
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
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 2, 0);
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_10|MOT2_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOT1_IN1_Pin|MOT1_IN2_Pin|MOT2_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 MOT2_IN1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|MOT2_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOT1_IN1_Pin MOT1_IN2_Pin MOT2_IN2_Pin */
  GPIO_InitStruct.Pin = MOT1_IN1_Pin|MOT1_IN2_Pin|MOT2_IN2_Pin;
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
