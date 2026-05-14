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
  *main.c
 ├── Control del robot: PID, MPU6050, motores, OLED
 ├── Protocolo UNER: armar/desarmar paquetes
 ├── Heartbeat de seguridad
 ├── Telemetría cada 50 ms
 └── UART DMA/IDLE hacia ESP01

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

#include "ESP01.h"
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

typedef struct __attribute__((packed)) {
    int16_t     acc_x, acc_y, acc_z;
    int16_t     gyro_pitch, gyro_yaw;
    int16_t     pitch_cdeg;
    int16_t     roll_cdeg;
    int16_t     yaw_cdeg;
    int32_t     pos_x_mm;
    int32_t     pos_y_mm;
    int16_t     velocidad_mm_s;
    uint8_t     modo;
    uint16_t    IR[8];
    uint8_t     infoAdicional;
} PayloadDataV1_t;

typedef struct {
    uint8_t header[4];      // "UNER"
    uint8_t length;         // CMD + N_Payload + Checksum
    uint8_t token;          // ':'
    uint8_t cmd;            // ID del comando
    uint8_t payload[64];    // Buffer genérico (sobra espacio)
    // El checksum no lo ponemos en el struct fijo porque su posición varía
} UnerPacket_t;
enum {
	MODO_IDDLE 					= 0,
	MODO_RC						= 1,
	MODO_FL_INICIO				= 2,
	MODO_FL_BUSQUEDA_INICIAL  	= 3,
	MODO_FL_SIGUIENDO			= 4,
	MODO_FL_RESCATE				= 5,
	MODO_FL_PERDIDO_FAILSAFE  	= 6,
	MODO_FL_ESQUIVAR_OBSTACULO 	= 7,
	MODO_FL_INGRESO_A_90		= 8
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
       CMD_DEFINE_ZERO_SETPOINT	= 15, 		/*!< Ajustar  Setpoint de los motores*/
       CMD_ONOFFMOTORS 			= 16, 		/*!< Prender y apagar motores*/
       CMD_DATA 					= 17,		/*!< El robot manda datos de la unidad Sensitiva*/
       //PID SISTEMA BALANCEO
       CMD_PID_PITCH_KP      		= 20, 		/*!< Ajustar Término Proporcional del PID basado en grado de libertad Pitch*/
       CMD_PID_PITCH_KI      		= 21, 		/*!< Ajustar Término Integral del PID basado en grado de libertad Pitch*/
       CMD_PID_PITCH_KD      		= 22, 		/*!< Ajustar Término Derivativa del PID basado en grado de libertad Pitch*/
       CMD_PID_ALPHA 				= 23, 		/*!< Ajustar  Alpha del del filtro complementario utilizado en el PID basado en grado de libertad Pitch*/


       CMD_PID_PITCH_LIM_INCLI		= 24, 		/*!< Ajustar Término Proporcional del PID basado en grado de libertad Pitch*/
       CMD_PID_PITCH_CORECCION_RCSP= 25, 		/*!< Ajustar Término Integral del PID basado en grado de libertad Pitch*/

       CMD_RC_MOVE_5_CM             = 30, 		/*!< Ajustar Término Integral del PID basado en grado de libertad Pitch*/

       //PID SISTEMA DE GIRO
       CMD_PID_YAW_KP      		= 40, 		/*!< Ajustar Término Proporcional del PID basado en grado de libertad Yaw*/
       CMD_PID_YAW_KD      		= 41, 		/*!< Ajustar Término Integral del PID basado en grado de libertad Yaw*/
       CMD_PID_YAW_SP      		= 42, 		/*!< Ajustar Término Derivativa del PID basado en grado de libertad Yaw*/

       CMD_PID_YAW_MULTIPLICADOR  	= 43, 		/*!< Ajustar Término Proporcional del PID basado en grado de libertad Yaw*/
       //SENSORES
       CMD_IR_INICIAR_CALIBRACION	= 44, 		/*!< Ajustar Término Proporcional del PID basado en grado de libertad Yaw*/
       CMD_IR_DETENER_CALIBRACION  = 45, 		/*!< Ajustar Término Integral del PID basado en grado de libertad Yaw*/
       //NETWORK
       CMD_NETWORK_CHANGE_SSID		= 50,
       CMD_NETWORK_CHANGE_PASSWORD	= 51
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


// ================= [ ESP01 / UDP ALIVE TEST ] ================= //
typedef struct {
    _sESP01Handle Config;

    uint8_t uner_rx_ring[255];
    volatile uint16_t uner_rx_write;
    volatile uint16_t uner_rx_read;

    volatile uint8_t udp_started;
    volatile uint8_t udp_connected;
} ESP01_App_t;

ESP01_App_t ESP = {0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ================= [ PID ] ================= //
//#define 	ALPHA_PID 			0.98f    // Suaviza las vibraciones del acelerómetro
#define 	DT_PID 				0.01f
// ================= [ Periféricos ] ================= //
#define 	MPU6050_ADDR 	(0x68 << 1) // Dirección I2C desplazada

	#define IRCSAAB (estado_sensores[0] && estado_sensores[1] && estado_sensores[2] && estado_sensores[3])	// ESTA MACRO SIGNIFICA IR CURRENT STATE ARE ALL BLACK
	#define IRLSAAB (ultimo_estado_sensores[0] && ultimo_estado_sensores[1] && ultimo_estado_sensores[2] && ultimo_estado_sensores[3])	// ESTA MACRO SIGNIFICA IR LAST STATE ARE ALL BLACK
	#define IRCSAAW (!estado_sensores[0] && !estado_sensores[1] && !estado_sensores[2] && !estado_sensores[3])	// ESTA MACRO SIGNIFICA IR CURRENT STATE ARE ALL WHITE
	#define IRLSAAW (!ultimo_estado_sensores[0] && !ultimo_estado_sensores[1] && !ultimo_estado_sensores[2] && !ultimo_estado_sensores[3])	// ESTA MACRO SIGNIFICA IR LAST STATE ARE ALL WHITE
	#define	AIRAB	(estado_sensores[0] || estado_sensores[1] || estado_sensores[2] || estado_sensores[3])	// ESTA MACRO SIGNIFICA ANY IR ARE BLACK
// ================= [ Comunicación ] ================= //
#define 	RX_BUFFER_SIZE 		        64
#define 	UNER_HEADER_STR 	        "UNER"
#define 	UNER_TOKEN      	        ':'

#define     ESP01_UDP_LOCAL_PORT       8888
#define     ESP01_QT_REMOTE_PORT       8888
#define     ESP01_RX_DMA_SIZE          256
#define     UNER_RX_RING_SIZE          128
#define     UNER_V1_VERSION            1
#define     UNER_V1_MAX_PAYLOAD        64
#define     UNER_V1_FLAG_ACK_REQUIRED  0x01
#define     UNER_V1_FLAG_ACK           0x02

#define     ESP01_WIFI_PROFILE_UNIVERSITY  0
#define     ESP01_WIFI_PROFILE_HOME        1
#define     ESP01_WIFI_PROFILE_LAB         2
#define     ESP01_WIFI_PROFILE             ESP01_WIFI_PROFILE_LAB

#if ESP01_WIFI_PROFILE == ESP01_WIFI_PROFILE_HOME
#define     WIFI_SSID                  "InternetPlus_872f10"
#define     WIFI_PASSWORD              "wlan78d0ef"
#define     ESP01_QT_REMOTE_IP         "192.168.1.3"
#elif ESP01_WIFI_PROFILE == ESP01_WIFI_PROFILE_LAB
#define     WIFI_SSID                  "LabPrototip"
#define     WIFI_PASSWORD              "labproto"
#define     ESP01_QT_REMOTE_IP         "172.24.150.89"
#elif ESP01_WIFI_PROFILE == ESP01_WIFI_PROFILE_UNIVERSITY
#define     WIFI_SSID                  "FCAL"
#define     WIFI_PASSWORD              "fcalconcordia.06-2019"
#define     ESP01_QT_REMOTE_IP         "172.23.224.234"
#else
#error "Seleccionar un perfil WiFi valido para ESP01_WIFI_PROFILE"
#endif
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
volatile	uint8_t			currentMode = MODO_IDDLE;
// ================= [ Flags ] ================= //
volatile 	uint8_t 		flagPID 				= 	0;
volatile 	uint8_t 		flagDisplay				=	0;
volatile	uint8_t			flagSendUNER 			= 	0;
volatile	uint8_t			flagDataToQt 			= 	0;
volatile	uint8_t			flagWIFI				=   0;
volatile	uint8_t 		flagOLED 				= 	2;
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
			int16_t 		deadband_L = 130;			/*!< Zona Muerta del PWM para el motor 1*/
			int16_t 		deadband_R = 75; 			/*!< Zona Muerta del PWM para el motor 2*/
// =================[ Variables de Control PID PITCH] ================= //
			float 			Kp = 170.0f;					/*!< Término Proporcional: [30] Si hay inclinación aplica una fuerza proporcional. Si se usara solo P, el robot oscilaría de un lado a otro sin quedarse quieto.*/
			float 			Ki = 0.1f;					/*!< Término Integrativo: Elimina el error de estado estacionario*/
			float 			Kd = 2.5f;						/*!< Término Derivativo: [1.5] mide la velocidad a la que está cambiando el error. Actúa como un amortiguador*/
			float 			setpoint = 7.1f;				/*!< Este SetPoint,se usa para desbalancer o caminar */
			float 			setpointDeEquilibrio = 0.0f;	/*!< Set Point de equilibrio, el cero del robot, el punto en el qeu el robot queda a vertical*/
			float 			integral = 0;
			float 			last_error = 0;
			float           ALPHA_PID = 0.994f;
			float			correccionRCSP = 0.98;
			float 			limite_inclinacion = 3.0f;
// =================[ Variables de Control PID YAW] ================= //
			float 		Kp_yaw = 1200.0f;
			float 		Kd_yaw = 0.0f;
			float 		last_error_yaw = 0;
volatile    float 		FL_setpoint = 0.0f;
			float 		last_state_linea = 0.0f;
			float 		error_linea;
			uint32_t	timer_rescate=0;
volatile 	uint16_t 	adc_raw[4]= {0, 0, 0, 0};
volatile 	uint16_t 	adc_filtrado[8] = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000};
volatile 	uint8_t 	estado_sensores[4]	= {0, 0, 0, 0};
volatile 	uint8_t 	ultimo_estado_sensores[4] = {0, 0, 0, 0};
volatile 	uint16_t 	sensor_min[4]= {0, 0, 0, 0};
volatile 	uint16_t 	sensor_max[4]= {0, 0, 0, 0};
volatile 	uint16_t 	sensor_threshold[4]= {0, 0, 0, 0};
volatile 	uint8_t 	flag_calibrando_linea = 0; // Para saber en qué estado estamos
		// Umbrales de los IR de obstáculos (A calibrar luego)
		// Asumo que a mayor valor, más cerca está el obstáculo
		uint16_t UMBRAL_FRENTE_CHOQUE = 3700;  // Valor donde detecta la caja de frente
		uint16_t SETPOINT_PARED_DER = 	1000;    // Valor ideal para mantener la caja a 5cm
		uint16_t UMBRAL_PERDIDA_PARED = 3000;   // Si cae de este valor, la pared se terminó
uint8_t sentidoDeGiro=0; // 0 horario, 1 anti-horario
// =================[ Variables del Filtro del MPU6050 ] =================//
			float 			angle_y 	= 0;
			float           angle_roll  = 0;
			float           angle_yaw   = 0;
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
volatile 	float 			RC_setpoint 			= 0;
volatile	float 			RC_setpoint_base	 = 0.8f;
volatile 	float 			RC_slow_setpoint = 0;
volatile 	int16_t   		RC_steering = 0;
float 		paso = 0.08f; // Velocidad de inclinación
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
	float P =  0;
	float I =  0;
	float D =  0;
	float output = 0;

	float Kp_Agresivo = 0.0f;
// =================[ Digitalizador del IR ] =================//
// Máquina de estados de la evasión
typedef enum {
    EVA_DETECTADO,
    EVA_GIRAR_IZQ,
    EVA_SEGUIR_PARED,
    EVA_AVANZAR_ESQUINA,
    EVA_GIRAR_DER
} EstadoEvasion_t;

EstadoEvasion_t estado_evasion = EVA_DETECTADO;
uint32_t timer_evasion = 0; // Para contar milisegundos en las maniobras
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
volatile uint32_t timeout_rc=0;

char ip_address[16] = "0.0.0.0";
volatile uint8_t ip_received_flag = 0;
volatile uint8_t esp01_alive_received = 0;
volatile uint8_t esp01_oled_ready = 0;
volatile uint32_t esp01_tx_count = 0;
volatile uint32_t esp01_rx_count = 0;
volatile uint32_t esp01_payload_count = 0;
volatile uint32_t esp01_alive_count = 0;
volatile uint32_t esp01_ack_count = 0;
volatile uint8_t uner_ack_pending = 0;
volatile uint8_t uner_ack_cmd = 0;
volatile uint8_t uner_ack_seq = 0;
volatile uint8_t uner_ack_status = 0;
volatile uint8_t uner_telemetry_enabled = 0;
volatile uint8_t uner_tx_busy = 0;
volatile uint32_t uner_tx_ready_count = 0;
volatile uint32_t uner_tx_busy_count = 0;
volatile uint32_t uner_tx_ok_count = 0;
volatile uint32_t uner_tx_last_try_tick = 0;
volatile uint32_t uner_tx_last_ok_tick = 0;
char esp01_last_debug[18] = "-";
char esp01_last_rx[18] = "-";
uint8_t esperando_digitos_ip = 0; // Bandera para nuestra mini máquina de estados
float velocidad_objetivo = 0.0f; // Reemplaza a tu viejo RC_setpoint y FL_setpoint
float Kp_vel = 0.015f;           // Ganancia del lazo de velocidad (empezamos MUY bajo)
float showoutput=0;
int8_t prescaler=0;
float multiplicadorYaw 	 = 0.01;
float error=0;
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
void buzzerSecuence(Buzzer_Seq_t *seq);
void BS_tcpConnectSecuence(void);
void BS_Error(void);

void setESP01_CHPD(uint8_t val);
int ESP01_UART_Transmit(uint8_t val);
void ESP01_Data_Received(uint8_t value);
void onESP01ChangeState(_eESP01STATUS esp01State);
void onESP01Debug(const char *dbgStr);
void ESP01_App_Task(void);

void UNER_Rx_Task(void);
void UNER_ProcessByte(uint8_t b);
void UNER_ProcessByteV1(uint8_t b);
uint16_t UNER_Crc16Ccitt(const uint8_t *data, uint16_t len);
uint8_t UNER_SendV1(uint8_t cmd, uint8_t flags, const uint8_t *payload, uint8_t payload_len);
uint8_t UNER_SendAckV1(uint8_t acked_cmd, uint8_t acked_seq, uint8_t status);
uint8_t UNER_SendTelemetryV1(void);
uint8_t UNER_SendInt16(uint8_t cmd, int16_t value);
void UNER_HandlePacket(uint8_t cmd, uint8_t flags, uint8_t seq, uint8_t *payload, uint8_t payload_len);
void Telemetry_UpdateMPU(void);
void Filtrar_Sensores_IR(void);
/**
 * @brief buzzerSecuence:  				Máquina de estados para señales auditivas (Buzzer) no-bloqueante.
 * @param seq:			   				Estructura con tiempos de duración, intervalo y repeticiones.
 */
void buzzerSecuence(Buzzer_Seq_t *seq);
/**
 * @brief BS_tcpConnectSecuence:		Secuencia para la conexión del servidor.
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
float calcularErrorYawDiscreto(void);
float calcularErrorYawContinuo(void);
int16_t Calcular_PID_YAW(float error_linea);
/**
 * @brief sendCMD:						Envía un comando bajo el Protocolo UNER vía UART DMA.
 * @param cmd: 							Código del comando (CMD)
 * @param param: 						Parámetro de 16 bits (enviado en Little Endian ).
 */
void Procesar_Evasion_Obstaculo(void);
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

void buzzerSecuence(Buzzer_Seq_t *seq) {
    if (seq->repeat == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
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
        seq->repeat--;
    }
}

void BS_tcpConnectSecuence(void) {
    hBuzzer.duration = 100;
    hBuzzer.interval = 50;
    hBuzzer.repeat = 2;
    hBuzzer.state = 0;
    hBuzzer.last_tick = HAL_GetTick();
}

void BS_Error(void) {
    hBuzzer.duration = 500;
    hBuzzer.interval = 100;
    hBuzzer.repeat = 1;
    hBuzzer.state = 0;
    hBuzzer.last_tick = HAL_GetTick();
}

void BS_ACK_NOT_FOUND(void) {
    hBuzzer.duration = 200;
    hBuzzer.interval = 50;
    hBuzzer.repeat = 3;
    hBuzzer.state = 0;
    hBuzzer.last_tick = HAL_GetTick();
}

void BS_NEWPARAM_OK(void) {
    hBuzzer.duration = 80;
    hBuzzer.interval = 50;
    hBuzzer.repeat = 1;
    hBuzzer.state = 0;
    hBuzzer.last_tick = HAL_GetTick();
}

void BS_NEWPARAM_ISNOTOK(void) {
    hBuzzer.duration = 800;
    hBuzzer.interval = 1;
    hBuzzer.repeat = 1;
    hBuzzer.state = 0;
    hBuzzer.last_tick = HAL_GetTick();
}

/* ===================== [ ESP01 + UDP ALIVE ] ===================== */

void setESP01_CHPD(uint8_t val)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
#if 0
    /*
     * Ajustar si EN/CH_PD del ESP-01 está en otro GPIO.
     * Uso PB2 porque tu main ya lo levantaba durante inicialización.
     */
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

int ESP01_UART_Transmit(uint8_t val)
{
    /*
     * TX no bloqueante byte a byte hacia ESP01.
     * Más adelante se puede migrar a TX DMA.
     */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)) {
        huart1.Instance->DR = val;
        esp01_tx_count++;
        return 1;
    }

    return 0;
}

void ESP01_Data_Received(uint8_t value)
{
    static char line[8];
    static uint8_t index = 0;

    esp01_payload_count++;

    uint16_t next = ESP.uner_rx_write + 1;

    if (next >= UNER_RX_RING_SIZE) {
        next = 0;
    }

    if (next != ESP.uner_rx_read) {
        ESP.uner_rx_ring[ESP.uner_rx_write] = value;
        ESP.uner_rx_write = next;
    }

    if (index < sizeof(line)) {
        line[index++] = (char)value;
    } else {
        index = 0;
        return;
    }

    if (value == '\n') {
        if (index == 7 && memcmp(line, "ALIVE\r\n", 7) == 0) {
            esp01_alive_received = 1;
            esp01_alive_count++;
        }

        index = 0;
    }

    return;

#if 0
    /*
     * Bytes útiles TCP recibidos desde ESP01.c después de parsear +IPD.
     */
    uint16_t next = ESP.uner_rx_write + 1;

    if (next >= UNER_RX_RING_SIZE) {
        next = 0;
    }

    if (next != ESP.uner_rx_read) {
        ESP.uner_rx_ring[ESP.uner_rx_write] = value;
        ESP.uner_rx_write = next;
    }
#endif
}

void onESP01ChangeState(_eESP01STATUS esp01State)
{
    switch (esp01State) {

    case ESP01_WIFI_CONNECTED:
        flagWIFI = 1;
        break;

    case ESP01_WIFI_NEW_IP:
    {
        char *ip = ESP01_GetLocalIP();

        if (ip != NULL) {
            strncpy(ip_address, ip, sizeof(ip_address) - 1);
            ip_address[sizeof(ip_address) - 1] = '\0';
            ip_received_flag = 1;
        }

        if (!ESP.udp_started) {
            ESP.udp_started = 1;
            ESP01_StartUDP(ESP01_QT_REMOTE_IP, ESP01_QT_REMOTE_PORT, ESP01_UDP_LOCAL_PORT);
        }
        break;
    }

    case ESP01_UDPTCP_CONNECTED:
        flagWIFI = 1;
        ESP.udp_connected = 1;
        uner_tx_busy = 0;
        break;

    case ESP01_UDPTCP_DISCONNECTED:
        flagWIFI = 0;
        ESP.udp_connected = 0;
        ESP.udp_started = 0;
        uner_tx_busy = 0;
        break;

    case ESP01_WIFI_DISCONNECTED:
        flagWIFI = 0;
        ESP.udp_connected = 0;
        ESP.udp_started = 0;
        uner_tx_busy = 0;
        break;

    case ESP01_SEND_OK:
        uner_tx_busy = 0;
        uner_tx_ok_count++;
        uner_tx_last_ok_tick = HAL_GetTick();
        break;

    default:
        break;
    }
}

void onESP01Debug(const char *dbgStr)
{
    if (dbgStr == NULL) {
        return;
    }

    if (strncmp(dbgStr, "+&DBG", 5) == 0) {
        dbgStr += 5;
    }

    strncpy(esp01_last_debug, dbgStr, sizeof(esp01_last_debug) - 1);
    esp01_last_debug[sizeof(esp01_last_debug) - 1] = '\0';

    for (uint8_t i = 0; esp01_last_debug[i] != '\0'; i++) {
        if (esp01_last_debug[i] == '\r' || esp01_last_debug[i] == '\n') {
            esp01_last_debug[i] = '\0';
            break;
        }
    }
}

void ESP01_App_Task(void)
{
    static uint32_t last_oled_update = 0;
    static uint32_t last_mpu_update = 0;
    static uint32_t last_telemetry = 0;

    ESP01_Task();
    UNER_Rx_Task();

    if ((HAL_GetTick() - last_mpu_update) >= 10) {
        last_mpu_update = HAL_GetTick();
        Telemetry_UpdateMPU();
        Filtrar_Sensores_IR();
    }

    if ((HAL_GetTick() - last_oled_update) >= 500) {
        last_oled_update = HAL_GetTick();
        screenScheduler();
    }

    if (uner_ack_pending && ESP.udp_connected) {
        if (UNER_SendAckV1(uner_ack_cmd, uner_ack_seq, uner_ack_status)) {
            uner_ack_pending = 0;
            esp01_ack_count++;
        }
    }

    if (uner_telemetry_enabled && !uner_ack_pending && ESP.udp_connected && (HAL_GetTick() - last_telemetry) >= 80) {
        if (UNER_SendTelemetryV1()) {
            last_telemetry = HAL_GetTick();
        }
    }
}

/* ===================== [ UNER mínimo: ALIVE -> ACK ] ===================== */

uint16_t UNER_Crc16Ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

uint8_t UNER_SendV1(uint8_t cmd, uint8_t flags, const uint8_t *payload, uint8_t payload_len)
{
    static uint8_t seq = 0;
    uint8_t frame[4 + 1 + 1 + 1 + 1 + 1 + UNER_V1_MAX_PAYLOAD + 2];
    uint16_t idx = 0;
    uint32_t now = HAL_GetTick();

    if (payload_len > UNER_V1_MAX_PAYLOAD) {
        return 0;
    }

    if (uner_tx_busy) {
        uner_tx_busy_count++;
        return 0;
    }

    if ((now - uner_tx_last_try_tick) < 40) {
        return 0;
    }

    frame[idx++] = 'U';
    frame[idx++] = 'N';
    frame[idx++] = 'E';
    frame[idx++] = 'R';
    frame[idx++] = UNER_V1_VERSION;
    frame[idx++] = cmd;
    frame[idx++] = flags;
    frame[idx++] = seq;
    frame[idx++] = payload_len;

    if (payload != NULL && payload_len > 0) {
        memcpy(&frame[idx], payload, payload_len);
        idx += payload_len;
    }

    uint16_t crc = UNER_Crc16Ccitt(frame, idx);
    frame[idx++] = (uint8_t)(crc & 0xFF);
    frame[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    uner_tx_last_try_tick = now;

    if (ESP01_Send(frame, 0, idx, idx) == ESP01_SEND_READY) {
        seq++;
        uner_tx_busy = 1;
        uner_tx_ready_count++;
        return 1;
    }

    uner_tx_busy_count++;
    return 0;
}

uint8_t UNER_SendAckV1(uint8_t acked_cmd, uint8_t acked_seq, uint8_t status)
{
    uint8_t payload[3];

    payload[0] = acked_cmd;
    payload[1] = acked_seq;
    payload[2] = status;

    return UNER_SendV1(CMD_ACK, UNER_V1_FLAG_ACK, payload, sizeof(payload));
}

uint8_t UNER_SendTelemetryV1(void)
{
    PayloadDataV1_t payload;

    payload.acc_x = (int16_t)accelx;
    payload.acc_y = (int16_t)accely;
    payload.acc_z = (int16_t)accelz;
    payload.gyro_pitch = (int16_t)giro;
    payload.gyro_yaw = (int16_t)giro_z;
    payload.pitch_cdeg = (int16_t)(angle_y * 100.0f);
    payload.roll_cdeg = (int16_t)(angle_roll * 100.0f);
    payload.yaw_cdeg = (int16_t)(angle_yaw * 100.0f);
    payload.pos_x_mm = 0;
    payload.pos_y_mm = 0;
    payload.velocidad_mm_s = (int16_t)(velocidad_objetivo * 1000.0f);
    payload.modo = currentMode;

    for (uint8_t i = 0; i < 8; i++) {
        payload.IR[i] = adc_filtrado[i];
    }

    payload.infoAdicional = flagCalibrationIsReady;

    return UNER_SendV1(CMD_DATA, 0, (const uint8_t *)&payload, sizeof(payload));
}

uint8_t UNER_Send(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
    uint8_t frame[4 + 1 + 1 + 1 + 64 + 1];
    uint16_t idx = 0;

    if (payload_len > 64) {
        return 0;
    }

    frame[idx++] = 'U';
    frame[idx++] = 'N';
    frame[idx++] = 'E';
    frame[idx++] = 'R';

    /*
     * Len = CMD + Payload + Checksum.
     * Token ':' queda fuera del Len, igual que en tu código original.
     */
    frame[idx++] = (uint8_t)(1 + payload_len + 1);
    frame[idx++] = UNER_TOKEN;
    frame[idx++] = cmd;

    if (payload != NULL && payload_len > 0) {
        memcpy(&frame[idx], payload, payload_len);
        idx += payload_len;
    }

    uint8_t checksum = 0;

    for (uint16_t i = 0; i < idx; i++) {
        checksum ^= frame[i];
    }

    frame[idx++] = checksum;

    return (ESP01_Send(frame, 0, idx, idx) == ESP01_SEND_READY);
}

uint8_t UNER_SendInt16(uint8_t cmd, int16_t value)
{
    uint8_t payload[2];

    payload[0] = (uint8_t)(value & 0xFF);
    payload[1] = (uint8_t)((value >> 8) & 0xFF);

    return UNER_SendV1(cmd, 0, payload, 2);
}

void UNER_Rx_Task(void)
{
    while (ESP.uner_rx_read != ESP.uner_rx_write) {
        uint8_t b = ESP.uner_rx_ring[ESP.uner_rx_read];

        ESP.uner_rx_read++;

        if (ESP.uner_rx_read >= UNER_RX_RING_SIZE) {
            ESP.uner_rx_read = 0;
        }

        UNER_ProcessByteV1(b);
    }
}

void UNER_ProcessByteV1(uint8_t b)
{
    typedef enum {
        UNER_V1_ST_U,
        UNER_V1_ST_N,
        UNER_V1_ST_E,
        UNER_V1_ST_R,
        UNER_V1_ST_VERSION,
        UNER_V1_ST_CMD,
        UNER_V1_ST_FLAGS,
        UNER_V1_ST_SEQ,
        UNER_V1_ST_LEN,
        UNER_V1_ST_PAYLOAD,
        UNER_V1_ST_CRC0,
        UNER_V1_ST_CRC1
    } UNER_V1_State_t;

    static UNER_V1_State_t st = UNER_V1_ST_U;
    static uint8_t frame[4 + 1 + 1 + 1 + 1 + 1 + UNER_V1_MAX_PAYLOAD];
    static uint8_t idx = 0;
    static uint8_t payload_len = 0;
    static uint8_t payload_idx = 0;
    static uint8_t crc0 = 0;

    switch (st) {
    case UNER_V1_ST_U:
        if (b == 'U') {
            idx = 0;
            frame[idx++] = b;
            st = UNER_V1_ST_N;
        }
        break;

    case UNER_V1_ST_N:
        if (b == 'N') {
            frame[idx++] = b;
            st = UNER_V1_ST_E;
        } else {
            st = UNER_V1_ST_U;
        }
        break;

    case UNER_V1_ST_E:
        if (b == 'E') {
            frame[idx++] = b;
            st = UNER_V1_ST_R;
        } else {
            st = UNER_V1_ST_U;
        }
        break;

    case UNER_V1_ST_R:
        if (b == 'R') {
            frame[idx++] = b;
            st = UNER_V1_ST_VERSION;
        } else {
            st = UNER_V1_ST_U;
        }
        break;

    case UNER_V1_ST_VERSION:
        if (b == UNER_V1_VERSION) {
            frame[idx++] = b;
            st = UNER_V1_ST_CMD;
        } else {
            st = UNER_V1_ST_U;
        }
        break;

    case UNER_V1_ST_CMD:
        frame[idx++] = b;
        st = UNER_V1_ST_FLAGS;
        break;

    case UNER_V1_ST_FLAGS:
        frame[idx++] = b;
        st = UNER_V1_ST_SEQ;
        break;

    case UNER_V1_ST_SEQ:
        frame[idx++] = b;
        st = UNER_V1_ST_LEN;
        break;

    case UNER_V1_ST_LEN:
        payload_len = b;
        if (payload_len > UNER_V1_MAX_PAYLOAD) {
            st = UNER_V1_ST_U;
            break;
        }

        frame[idx++] = b;
        payload_idx = 0;
        st = (payload_len == 0) ? UNER_V1_ST_CRC0 : UNER_V1_ST_PAYLOAD;
        break;

    case UNER_V1_ST_PAYLOAD:
        frame[idx++] = b;
        payload_idx++;
        if (payload_idx >= payload_len) {
            st = UNER_V1_ST_CRC0;
        }
        break;

    case UNER_V1_ST_CRC0:
        crc0 = b;
        st = UNER_V1_ST_CRC1;
        break;

    case UNER_V1_ST_CRC1:
    {
        uint16_t crc_rx = (uint16_t)crc0 | ((uint16_t)b << 8);
        uint16_t crc_calc = UNER_Crc16Ccitt(frame, idx);

        if (crc_rx == crc_calc) {
            uint8_t cmd = frame[5];
            uint8_t flags = frame[6];
            uint8_t seq = frame[7];
            uint8_t *payload = &frame[9];

            UNER_HandlePacket(cmd, flags, seq, payload, payload_len);
        }

        st = UNER_V1_ST_U;
        break;
    }

    default:
        st = UNER_V1_ST_U;
        break;
    }
}

void UNER_ProcessByte(uint8_t b)
{
    typedef enum {
        UNER_ST_U,
        UNER_ST_N,
        UNER_ST_E,
        UNER_ST_R,
        UNER_ST_LEN,
        UNER_ST_TOKEN,
        UNER_ST_BODY
    } UNER_State_t;

    static UNER_State_t st = UNER_ST_U;
    static uint8_t len = 0;
    static uint8_t idx = 0;
    static uint8_t body[1 + 64 + 1];

    switch (st) {

    case UNER_ST_U:
        if (b == 'U') st = UNER_ST_N;
        break;

    case UNER_ST_N:
        st = (b == 'N') ? UNER_ST_E : UNER_ST_U;
        break;

    case UNER_ST_E:
        st = (b == 'E') ? UNER_ST_R : UNER_ST_U;
        break;

    case UNER_ST_R:
        st = (b == 'R') ? UNER_ST_LEN : UNER_ST_U;
        break;

    case UNER_ST_LEN:
        len = b;

        if (len < 2 || len > sizeof(body)) {
            st = UNER_ST_U;
        } else {
            idx = 0;
            st = UNER_ST_TOKEN;
        }
        break;

    case UNER_ST_TOKEN:
        if (b == UNER_TOKEN) {
            st = UNER_ST_BODY;
        } else {
            st = UNER_ST_U;
        }
        break;

    case UNER_ST_BODY:
        body[idx++] = b;

        if (idx >= len) {
            uint8_t checksum = 0;

            checksum ^= 'U';
            checksum ^= 'N';
            checksum ^= 'E';
            checksum ^= 'R';
            checksum ^= len;
            checksum ^= UNER_TOKEN;

            for (uint8_t i = 0; i < (len - 1); i++) {
                checksum ^= body[i];
            }

            if (checksum == body[len - 1]) {
                uint8_t cmd = body[0];
                uint8_t *payload = &body[1];
                uint8_t payload_len = len - 2;

                UNER_HandlePacket(cmd, 0, 0, payload, payload_len);
            }

            st = UNER_ST_U;
        }
        break;

    default:
        st = UNER_ST_U;
        break;
    }
}

void UNER_HandlePacket(uint8_t cmd, uint8_t flags, uint8_t seq, uint8_t *payload, uint8_t payload_len)
{
    (void)payload;
    (void)payload_len;

    switch (cmd) {

    case CMD_ALIVE:
        esp01_alive_count++;
        uner_telemetry_enabled = 1;
        uner_ack_cmd = CMD_ALIVE;
        uner_ack_seq = seq;
        uner_ack_status = 0;
        uner_ack_pending = 1;
        break;

    default:
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = cmd;
            uner_ack_seq = seq;
            uner_ack_status = 0;
            uner_ack_pending = 1;
        }
        break;
    }
}

void sendCMD(uint8_t cmd, uint16_t param)
{
    UNER_SendInt16(cmd, (int16_t)param);
}

/*
 * IMPORTANTE:
 * DataToQt queda desactivada para esta etapa.
 * No debe mandar directo por HAL_UART_Transmit_DMA(&huart1),
 * porque UART1 ahora se usa para comandos AT del ESP01.
 */
void DataToQt(void)
{
    /*
     * Próxima etapa:
     * reemplazar este cuerpo por UNER_Send(CMD_DATA, telemetria.buffer, sizeof(PayloadData_t)).
     */
}

void Telemetry_UpdateMPU(void)
{
    uint8_t buffer[14];

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buffer, sizeof(buffer), 2) != HAL_OK) {
        return;
    }

    axRaw = (int16_t)((buffer[0] << 8) | buffer[1]);
    ayRaw = (int16_t)((buffer[2] << 8) | buffer[3]);
    azRaw = (int16_t)((buffer[4] << 8) | buffer[5]);
    gyPitchRaw = (int16_t)((buffer[10] << 8) | buffer[11]);
    gzYawRaw = (int16_t)((buffer[12] << 8) | buffer[13]);

    float gyro_pitch_rate = -(((float)gyPitchRaw - gyro_bias_y) / 131.0f);
    float gyro_yaw_rate = (((float)gzYawRaw - gyro_bias_z) / 131.0f);
    float pitch_accel = atan2f((float)axRaw - accel_bias_x, (float)azRaw - accel_bias_z) * 57.2957f;
    float roll_accel = atan2f((float)ayRaw - accel_bias_y, (float)azRaw - accel_bias_z) * 57.2957f;

    giro = gyro_pitch_rate;
    giro_z = gyro_yaw_rate;
    accelGiro = pitch_accel;
    angle_y = ALPHA_PID * (angle_y + gyro_pitch_rate * DT_PID) + (1.0f - ALPHA_PID) * pitch_accel;
    angle_roll = ALPHA_PID * angle_roll + (1.0f - ALPHA_PID) * roll_accel;
    angle_yaw += gyro_yaw_rate * DT_PID;

    accelx = axRaw;
    accely = ayRaw;
    accelz = azRaw;

    telemetria.data.acc_x = axRaw;
    telemetria.data.acc_y = ayRaw;
    telemetria.data.acc_z = azRaw;
    telemetria.data.gyro_pitch = (int16_t)gyro_pitch_rate;
    telemetria.data.gyro_yaw = (int16_t)gyro_yaw_rate;
    telemetria.data.pitch_filtrado = angle_y;
    telemetria.data.yaw_filtrado = angle_yaw;
    telemetria.data.pos_x = 0.0f;
    telemetria.data.pos_y = 0.0f;
    telemetria.data.velocidad = velocidad_objetivo;
    telemetria.data.modo = currentMode;
    telemetria.data.infoAdicional = flagCalibrationIsReady;

    for (uint8_t i = 0; i < 8; i++) {
        telemetria.data.IR[i] = adc_filtrado[i];
    }
}

void screenScheduler(void){
    if (!esp01_oled_ready) {
        return;
    }

    SSD1306_Fill(SSD1306_COLOR_BLACK);
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("ESP01 UDP", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(0, 10);
    SSD1306_Puts("", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(24, 10);
    SSD1306_Puts(ip_address, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(0, 20);
    sprintf(msg, "W:%d U:%d", flagWIFI, ESP.udp_connected);
    SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(0, 30);
    sprintf(msg, "TX:%lu RX:%lu", (unsigned long)esp01_tx_count, (unsigned long)esp01_rx_count);
    SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(0, 40);
    sprintf(msg, "A:%lu K:%lu O:%lu",
            (unsigned long)esp01_alive_count,
            (unsigned long)esp01_ack_count,
            (unsigned long)uner_tx_ok_count);
    SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(0, 52);
    sprintf(msg, "P:%lu B:%lu",
            (unsigned long)esp01_payload_count,
            (unsigned long)uner_tx_busy_count);
    SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
    return;

	if (!oled_is_busy) {
		SSD1306_Fill(SSD1306_COLOR_BLACK);
			switch(flagOLED){
			case 0:
				uint8_t sensores= (estado_sensores[2] * 100) + (estado_sensores[1] * 10) + estado_sensores[0];
				uint8_t lastSensores = (ultimo_estado_sensores[2] * 100) + (ultimo_estado_sensores[1] * 10) + ultimo_estado_sensores[0];

//				sprintf(msg, "IP:%s", ip_address);
//				SSD1306_GotoXY(1, 0);
//				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

				sprintf(msg, "Depurar FL");
				SSD1306_GotoXY(1, 0);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

				sprintf(msg, "O:%2.2f", showoutput);
				SSD1306_GotoXY(1, 10);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);



				sprintf(msg, "|%d|%d|%3.2f", sensores, lastSensores, multiplicadorYaw);
				SSD1306_GotoXY(1, 30);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

				sprintf(msg, "|E:%3.3f|", error);
				SSD1306_GotoXY(1, 40);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

				oled_update_requested = 1;
				break;
			case 2:
				sprintf(msg, "IP:%s", ip_address);
				SSD1306_GotoXY(1, 0);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
				sprintf(msg, "%d | %d", adc_buffer[5], ((int16_t)(error_linea * 100.0f)));
				SSD1306_GotoXY(1, 20);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
				sprintf(msg, "%d|%3.3F ", adc_buffer[6], RC_setpoint);
				SSD1306_GotoXY(1, 30);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

				sprintf(msg, "%d ", adc_buffer[7]);
				SSD1306_GotoXY(1, 40);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

				sprintf(msg, "Mode: %d ", currentMode);
				SSD1306_GotoXY(1, 50);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

				oled_update_requested = 1;
				break;
			case 1:
				sprintf(msg, "Config PID");
				SSD1306_GotoXY(0, 0);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
				sprintf(msg, "P|PITCH|YAW|M%d", currentMode);
				SSD1306_GotoXY(0, 10);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
				sprintf(msg, " Kp | %3.2f | %3.2f",Kp , Kp_yaw);
				SSD1306_GotoXY(1, 20);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
				sprintf(msg, " Kd | %3.2f | %3.2f",Kd, Kd_yaw);
				SSD1306_GotoXY(1, 30);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

				sprintf(msg, " Ki | %3.2f | %3.2f ", Ki, setpointDeEquilibrio);
				SSD1306_GotoXY(1, 40);
				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

//				float u = setpoint + RC_setpoint;
//				sprintf(msg, "%3.2f ",u);
//				SSD1306_GotoXY(1, 40);
//				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
//				sprintf(msg, "SP%3.2f|RCSP%f ", setpoint, RC_setpoint);
//				SSD1306_GotoXY(1, 50);
//				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);


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
//				SSD1306_GotoXY(1, 20);
//				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
//				sprintf(msg, "DBL: %d",deadband_L);
//				SSD1306_GotoXY(1, 30);
//				SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
//				sprintf(msg, "DBR: %d",deadband_R);
//				if (flagWIFI) {
//					// Dibujamos el icono de WiFi (podes usar lineas o circulos)
//					SSD1306_Clear();
//					SSD1306_DrawLine(110, 8, 114, 4, SSD1306_COLOR_WHITE); // Onda 1
//					SSD1306_DrawLine(114, 4, 118, 8, SSD1306_COLOR_WHITE);
//					SSD1306_DrawLine(112, 10, 116, 10, SSD1306_COLOR_WHITE); // Punto base
//				} else {
//					// Icono tachado o texto simple
//					SSD1306_Clear();
//					SSD1306_GotoXY(105, 2);
//					SSD1306_Puts("X", &Font_7x10, SSD1306_COLOR_WHITE);
//					SSD1306_DrawLine(105, 2, 120, 12, SSD1306_COLOR_WHITE); // Tachado
//				}
				oled_update_requested = 1;// NO llamamos a UpdateScreen(). Simplemente levantamos la bandera para el Scheduler.
				break;
			case 3:
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
void Filtrar_Sensores_IR(void) {
    for(int i = 0; i < 8; i++) {
        // Ahora es 90% historia y 10% dato nuevo
        adc_filtrado[i] = (adc_filtrado[i] * 9 + adc_buffer[i]) / 10;
    }
}
void Iniciar_Calibracion_Linea(void) {
    flag_calibrando_linea = 1;
    	for(int i = 0; i < 3; i++) {
    		sensor_min[i] = 4095; // Valor máximo del ADC
    		sensor_max[i] = 0;    // Valor mínimo del ADC
    		}

}
void Procesar_Calibracion_Linea(void) {
    if (flag_calibrando_linea) {
        for(int i = 0; i < 3; i++) {
            // Buscamos si hay un nuevo récord de valor bajo (Blanco)
            if(adc_filtrado[i] < sensor_min[i]) {
                sensor_min[i] = adc_filtrado[i];
            }
            // Buscamos si hay un nuevo récord de valor alto (Negro)
            if(adc_filtrado[i] > sensor_max[i]) {
                sensor_max[i] = adc_filtrado[i];
            }
        }
    }
}
void Leer_Linea_Digital(void) {
    for(int i = 0; i < 3; i++) {
        // Si el valor analógico superó la mitad, está viendo la línea
        if(adc_filtrado[i] > sensor_threshold[i]) {
            estado_sensores[i] = 1; // NEGRO
        } else {
            estado_sensores[i] = 0; // BLANCO
        }
    }
}
void Finalizar_Calibracion_Linea(void) {
    flag_calibrando_linea = 0;
//    for(int i = 0; i < 3; i++) {
//          // El punto medio perfecto entre lo más blanco y lo más negro que vio
//          sensor_threshold[i] = (sensor_max[i] + sensor_min[i]) / 2;
//      }
    sensor_threshold[0] = (806 + 3419) / 2;
    sensor_threshold[1] = (283 + 3213) / 2;
    sensor_threshold[2] = (339 + 3456) / 2;
    sensor_threshold[3] = (1280 + 3634) / 2;
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
			// 1. Calculamos la inclinación real absoluta (sin importar si va para adelante o atrás)
			float abs_angle = (angle_y < 0) ? -angle_y : angle_y;
			if (abs_angle > limite_inclinacion) {
//			        RC_slow_setpoint = RC_slow_setpoint * correccionRCSP;
				RC_setpoint = RC_setpoint * correccionRCSP;
			}
			    else {
			        if (RC_slow_setpoint < RC_setpoint) RC_slow_setpoint += paso;
			        if (RC_slow_setpoint > RC_setpoint) RC_slow_setpoint -= paso;
			    }
	   error = angle_y - (setpoint + RC_slow_setpoint);

	   switch(currentMode){
	   case MODO_RC:
	   default:
		   error = angle_y - setpoint;
		   break;
	   case MODO_IDDLE:
		   error = angle_y - setpoint - RC_setpoint;
		   break;
	   }

	   integral += error * DT_PID;
	   if(integral > 2000) integral = 2000;
	   else if(integral < -2000) integral = -2000;
	   float P_base = Kp * error;
	   float abs_error = (error < 0) ? -error : error;
	   float P_agresivo = Kp_Agresivo * (error * abs_error);
	   P = P_base + P_agresivo;
//	   P =  Kp * error;
	   I =  Ki * integral;
	   //D = Kd * (error - last_error) / DT_PID; //Kd * gyro_filtrado_ema; //Kd * (error - last_error) / DT_PID;//float D =  Kd * gyro_rate; //
	   D =  Kd * gyro_rate; //
	   output = P + I + D ; // Funcion de transferencia
	   showoutput = output;
	   last_error = error;
	   if(flagMotorsAreOn){
		   int16_t outputLeft = 0;
		   int16_t outputRigth = 0;
		   switch(currentMode){
			   case MODO_IDDLE:
				   outputLeft = (int16_t)output;
				   outputRigth = (int16_t)output;
				   break;
			   case MODO_RC:
			   default:
				   outputLeft = (int16_t)output  + RC_steering;
				   outputRigth = (int16_t)output - RC_steering;
				   break;
		   }
		   Robot_Drive(outputLeft, outputRigth);
	   }
	   if(flagMotorsAreOn==0||angle_y > 35 ||angle_y < -35)	   Robot_Drive(0, 0);
}
float calcularErrorYawDiscreto(void) {
    float numerador = 0.0f;
    float denominador = 0.0f;
    numerador = (estado_sensores[3] * -2.5f) +
                (estado_sensores[2] * -1.2f) +
                (estado_sensores[1] * 1.2f) +
                (estado_sensores[0] * 2.5f);
    denominador = estado_sensores[3] + estado_sensores[2] + estado_sensores[1] + estado_sensores[0];
    if (denominador == 0) {
        return 0.0f; // Asumimos error 0 para que siga caminando derecho
    	}
    float last_state_linea = (numerador / denominador);;

    return last_state_linea;
}
float calcularErrorYawContinuo(void) {
    float val_norm[3] = {0}; // Solo necesitamos los índices 0, 1 y 2
    for(int i = 0; i < 3; i++) {
        // Evitar división por cero si la calibración falló
        if (sensor_max[i] == sensor_min[i]) {
            val_norm[i] = 0.0f;
            continue;
        }
        float rango = (float)(sensor_max[i] - sensor_min[i]);
        val_norm[i] = (float)(adc_filtrado[i] - sensor_min[i]) / rango;

        // Saturamos
        if(val_norm[i] > 1.0f) val_norm[i] = 1.0f;
        if(val_norm[i] < 0.0f) val_norm[i] = 0.0f;
    }
    float y_izq    = val_norm[2]; // Sensor Interior Izquierdo
    float y_centro = val_norm[1]; // Sensor Interior Derecho
    float y_der    = val_norm[0]; // Sensor Exterior Derecho

    float denominador = (y_izq + y_der - 2.0f * y_centro);
    float offset = 0.0f;
    if (denominador != 0.0f)    offset = (y_izq - y_der) / denominador;
    float error_continuo =  offset;
    last_state_linea = error_continuo;
    return error_continuo;
}

int16_t Calcular_PID_YAW(float error_linea) {
    // 1. Proporcional: Reacción instantánea al error
    float P_yaw = Kp_yaw * error_linea;
    // 2. Derivativa: Frena el giro si se está acercando rápido al centro
    // DT_PID es tu delta de tiempo (ej: 0.01 si corre a 10ms)
    float D_yaw = Kd_yaw * (error_linea - last_error_yaw) / DT_PID;
    // 3. Guardamos el error para la próxima iteración
    last_error_yaw = error_linea;
    // 4. Salida total
    float salida_yaw = P_yaw + D_yaw;
    // Limitamos la salida máxima para que un volantazo no tire el robot al piso
    if(salida_yaw > 400.0f) salida_yaw = 400.0f;
    if(salida_yaw < -400.0f) salida_yaw = -400.0f;

    return (int16_t)salida_yaw;
}
void Procesar_Evasion_Obstaculo(void) {
	/*
    // 1. CONDICIÓN DE SALIDA (Prioridad Absoluta)
    if (estado_sensores[0] || estado_sensores[1] || estado_sensores[2] || estado_sensores[3]) {
        currentMode = ;
        estado_evasion = EVA_DETECTADO;
        return;
    }
    // 2. MÁQUINA DE ESTADOS (Pared a la Derecha)
    switch(estado_evasion) {
        case EVA_DETECTADO:
            RC_setpoint = 0.0f;
            RC_steering = 0.0f;
            estado_evasion = EVA_GIRAR_IZQ; // <--- Giramos a la izquierda para dejar la caja a la derecha
            timer_evasion = HAL_GetTick();
            break;
        case EVA_GIRAR_IZQ:
            RC_setpoint = 0.0f;
            RC_steering = -150.0f; // Negativo (Izquierda)
            if (HAL_GetTick() - timer_evasion > 500) estado_evasion = EVA_SEGUIR_PARED;
            break;
        case EVA_SEGUIR_PARED:
            RC_setpoint = 1.0f;
            // Mini PID de Pared Derecha (Usando adc_buffer[6])
            // Lógica:
            // Si lee 800 (muy cerca), Error = 800 - 1000 = -200. Steering Negativo -> Dobla Izq (se aleja).
            // Si lee 1500 (muy lejos), Error = 1500 - 1000 = 500. Steering Positivo -> Dobla Der (se acerca).
            float error_pared = (float)(adc_buffer[6] - SETPOINT_PARED_DER);
            // Mantenemos el Kp MUY bajito porque el sensor es súper sensible
            RC_steering = error_pared * 0.01f;
            // ¿Llegó a la esquina? (El valor sube porque se va a infinito)
            if (adc_buffer[6] > UMBRAL_PERDIDA_PARED) {
                estado_evasion = EVA_AVANZAR_ESQUINA;
                timer_evasion = HAL_GetTick();
            }
            break;
        case EVA_AVANZAR_ESQUINA:
            RC_setpoint = 1.0f;
            RC_steering = 0.0f;
            if (HAL_GetTick() - timer_evasion > 100) { // Avanza un toque para no enganchar la rueda
                estado_evasion = EVA_GIRAR_DER; // <--- Ahora giramos a la derecha para rodear
                timer_evasion = HAL_GetTick();
            }
            break;
        case EVA_GIRAR_DER:
            RC_setpoint = 0.0f;
            RC_steering = 150.0f; // Positivo (Derecha)
            if (HAL_GetTick() - timer_evasion > 500) estado_evasion = EVA_SEGUIR_PARED;
            break;
    }*/
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
			HAL_GPIO_WritePin(GPIOA, MOT1_IN1_Pin, GPIO_PIN_RESET );
							HAL_GPIO_WritePin(GPIOA, MOT1_IN2_Pin,  GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)speed_L);
			} else {



				HAL_GPIO_WritePin(GPIOA, MOT1_IN1_Pin,  GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOA, MOT1_IN2_Pin,  GPIO_PIN_RESET  );
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)(-speed_L));
			}
	   	   }
	if (speed_R == 0) {// Motor 2 (PB3, PA15, PB5)
	    	HAL_GPIO_WritePin(GPIOB, MOT2_IN1_Pin, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, MOT2_IN2_Pin, GPIO_PIN_RESET);
	      //  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)speed_R);
	    } else {
		if (speed_R >= 0) {// Motor 2 (PB3, PA15, PB5)
			HAL_GPIO_WritePin(GPIOB, MOT2_IN1_Pin, GPIO_PIN_SET  );
						HAL_GPIO_WritePin(GPIOA, MOT2_IN2_Pin,  GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)speed_R);
		} else {


			HAL_GPIO_WritePin(GPIOB, MOT2_IN1_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOA, MOT2_IN2_Pin, GPIO_PIN_SET  );
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {

        ESP01_Timeout10ms();

        counterHB++;
        if (counterHB >= 50) {
            counterHB = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {

        for (uint16_t i = 0; i < Size; i++) {
            uint8_t value = rx_buffer_uart[i];

            esp01_rx_count++;

            if (value == '\r' || value == '\n') {
                size_t len = strlen(esp01_last_rx);

                if (len < (sizeof(esp01_last_rx) - 1)) {
                    esp01_last_rx[len] = '|';
                    esp01_last_rx[len + 1] = '\0';
                }
            } else if (value >= 32 && value <= 126) {
                size_t len = strlen(esp01_last_rx);

                if (len >= (sizeof(esp01_last_rx) - 1)) {
                    memmove(esp01_last_rx, &esp01_last_rx[1], sizeof(esp01_last_rx) - 2);
                    esp01_last_rx[sizeof(esp01_last_rx) - 2] = '\0';
                    len = strlen(esp01_last_rx);
                }

                esp01_last_rx[len] = (char)value;
                esp01_last_rx[len + 1] = '\0';
            }

            ESP01_WriteRX(value);
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer_uart, sizeof(rx_buffer_uart));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(huart);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer_uart, sizeof(rx_buffer_uart));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
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
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
      /* ================= [ ESP01 AT OFICIAL ] ================= */

      memset(ip_address, 0, sizeof(ip_address));
      strncpy(ip_address, "0.0.0.0", sizeof(ip_address) - 1);

      if (SSD1306_Init() == 1) {
          esp01_oled_ready = 1;
          screenScheduler();
      }

      MPU6050_Init(&hi2c1);
      HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 8);

      ESP.uner_rx_read = 0;
      ESP.uner_rx_write = 0;
      ESP.udp_started = 0;
      ESP.udp_connected = 0;

      ESP.Config.DoCHPD = setESP01_CHPD;
      ESP.Config.WriteUSARTByte = ESP01_UART_Transmit;
      ESP.Config.WriteByteToBufRX = ESP01_Data_Received;

      ESP01_Init(&ESP.Config);
      ESP01_AttachChangeState(&onESP01ChangeState);
      ESP01_AttachDebugStr(&onESP01Debug);

      HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer_uart, sizeof(rx_buffer_uart));
      __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

      ESP01_SetWIFI(WIFI_SSID, WIFI_PASSWORD);

      HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    ESP01_App_Task();
	    continue;
	    /*
	     * Telemetría desactivada en esta etapa.
	     * DataToQt() ya no debe escribir directo por UART1.
	     */
	    if (flagDataToQt) 	        flagDataToQt = 0; // DataToQt();
	    if (flagDisplay) {
	        flagDisplay = 0;
	        screenScheduler();
	    	}
	    if (flagPID) {
	        flagPID = 0;
	        Filtrar_Sensores_IR();
	        Leer_Linea_Digital();
	        switch (currentMode) {
	        case MODO_IDDLE:
	            break;
	        case MODO_RC:
	            break;
	        case MODO_FL_INICIO:
	            Procesar_Calibracion_Linea();
	            break;
	        case MODO_FL_BUSQUEDA_INICIAL:
	            RC_steering = 0.0f;
	            if (estado_sensores[0] || estado_sensores[1] || estado_sensores[2]) {
	                currentMode = MODO_FL_SIGUIENDO;
	            }
	            break;
	        case MODO_FL_SIGUIENDO:
	        {
	            if (adc_buffer[5] < UMBRAL_FRENTE_CHOQUE) {
	                break;
	            }

	            error_linea = calcularErrorYawContinuo();
	            RC_steering = Calcular_PID_YAW(error_linea);

	            float abs_steering = (RC_steering < 0) ? -RC_steering : RC_steering;

	            RC_setpoint = RC_setpoint_base - (abs_steering * multiplicadorYaw);

	            if (RC_setpoint < 0.2f) {
	                RC_setpoint = 0.2f;
	            }

	            float compensacion_anti_caida = abs_steering * multiplicadorYaw;
	            RC_setpoint = RC_setpoint - compensacion_anti_caida;
	            break;
	        }

	        case MODO_FL_RESCATE:
	            RC_setpoint_base = 0.0f;
	            error_linea = 0.0f;

	            if (last_state_linea < 0.0f) {
	                RC_steering = 0;
	            } else {
	                RC_steering = 0;
	            }

	            if (estado_sensores[0] || estado_sensores[1] || estado_sensores[2]) {
	                currentMode = MODO_FL_SIGUIENDO;
	                break;
	            }
	            break;

	        default:
	            break;
	        }

	        PID_PITCH();

	        if (estado_sensores[0] || estado_sensores[1] || estado_sensores[2]) {
	            ultimo_estado_sensores[0] = estado_sensores[0];
	            ultimo_estado_sensores[1] = estado_sensores[1];
	            ultimo_estado_sensores[2] = estado_sensores[2];
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
