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

  *   ============= [ KEY button operation ] =============
  *   The KEY input is configured as Pull-Up and active-low, so the released
  *   state is logic HIGH and the pressed state is logic LOW. The button task
  *   applies software debounce before accepting any press or release edge.
  *
  *   Current KEY gestures:
  *   - Short press: starts IR threshold calibration when idle, or finishes it
  *     when calibration is already active.
  *   - Double click: switches between RC mode and follow-line search mode.
  *   - Long press: toggles flagMotorsAreOn and immediately stops the motors
  *     when the flag is turned OFF.
  *
  *   OLED page flagOLED == 5 is dedicated to KEY validation and shows the
  *   current mode, motor state, calibration state, raw/stable button state,
  *   pending click state, hold time, last gesture, and last resulting action.
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
#include "line_sensors.h"
#include "control_systems.h"
#include "obstacle_follow.h"
#include "buzzer_app.h"
#include "mpu6050_app.h"
#include "uner_protocol.h"
#include "button_key.h"
#include "math.h"

#include "ESP01.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum {
	MODO_IDDLE 					= 0,
	MODO_RC						= 1,
	MODO_FL_INICIO				= 2,
	MODO_FL_BUSQUEDA_INICIAL  	= 3,
	MODO_FL_SIGUIENDO			= 4,
	MODO_FL_RESCATE				= 5,
	MODO_FL_PERDIDO_FAILSAFE  	= 6,
	MODO_OBSTACLE_FOLLOW 		= CONTROL_MODE_OBSTACLE_FOLLOW,
	MODO_FL_INGRESO_A_90		= 8
};
enum {
       CMD_ALIVE       			= 0, 		/*!< Busca confirmar conexión inalambrica						*/
       CMD_ACK 	      			= 1, 		/*!< Confirma conexión inalambrica								*/
       CMD_SET_HB      			= 2, 		/*!< Configurar intervalo de Heartbeat							*/
       CMD_CHANGE_MODE				= 3,	/*!< Cambiar entre los modos  IDLE, FOLLOW_LINE, RC				*/
       CMD_CALIBRATE   			= 5, 		/*!< Calibración de MPU6050										*/
       CMD_START       			= 6, 		/*!< Activar motores / Inicio de balanceo						*/
       CMD_STOP        			= 7, 		/*!< Parada de emergencia / Motores a 0							*/
       CMD_CHANGE_OLED_SCREEN  	= 9,		/*!< Cambia o apaga la pantalla OLED							*/
       //RADIO CONTROL
       CMD_RC    					= 10, 		/*!< Movimiento manual (adelante, atrás, giros)					*/
       //MOTORES GENERAL
       CMD_CHANGE_DEADLINE_LEFT 	= 12,		/*!< Ajustar  Deadband del motor izquierdo						*/
       CMD_CHANGE_DEADLINE_RIGHT = 13,		/*!< Ajustar  Deadband del motor derecho						*/
       CMD_CHANGE_SETPOINT 		= 14, 		/*!< Ajustar  Setpoint de los motores*/
	   CMD_CHANGE_SETPOINT_FL 		= 15, 		/*!< Ajustar  Setpoint de los motores*/
       CMD_DEFINE_ZERO_SETPOINT     = 18,       /*!< Ajustar setpoint cero */
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
       CMD_NETWORK_CHANGE_PASSWORD	= 51,
       CMD_TELEMETRY_START          = 60,
       CMD_TELEMETRY_STOP           = 61,
       CMD_SET_YAW_PD               = 63,
       CMD_SET_YAW_CONFIG           = 64,
       CMD_SET_FL_CONFIG            = 65,
       CMD_TURN_MANEUVER            = 66,
       CMD_OBSTACLE_FOLLOW          = 67,
       CMD_PID_INTEGRAL_LIMIT       = 68
       // CMD_TELEMETRY   			= 0xA0, 	/*!< Envío de ángulos, velocidad y sensores IR	*/
       // CMD_LOG_MSG     			= 0xA1,  	/*!< Envío de mensajes de texto para debug		*/
   };

ESP01_App_t ESP = {0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ================= [ PID ] ================= //
//#define 	ALPHA_PID 			0.98f    // Suaviza las vibraciones del acelerómetro
#define 	DT_PID 				0.01f
// ================= [ Comunicación ] ================= //
#define 	RX_BUFFER_SIZE 		        64
#define     ESP01_RX_DMA_SIZE          256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// ================= [ Typedef ] ================= //
			Buzzer_Seq_t 	hBuzzer = {0}; // Inicializamos en cero
// ================= [ Variables generales ] ================= //
			uint16_t 		delayHB	= 50; //ENTRE 1 Y 200
volatile	uint8_t			currentMode = MODO_IDDLE;
// ================= [ Flags ] ================= //
volatile	uint8_t			flagWIFI				=   0;
volatile	uint8_t 		flagOLED 				= 	1;
volatile	uint8_t			flagMotorsAreOn 		=	0;
volatile	uint8_t 		flagCalibrationIsReady 	= 	0; // Bandera para no activar el PID antes de tiempo
volatile	uint8_t			flag_RC_active			=	0;
volatile    uint8_t			flag10ms  				=	0;
// ================= [ Counters ] ================= //
volatile 	uint32_t 		counterHB=0;				/*!< Utilizado en la interrupción del Timer 4 para manejar el HeartBit*/
volatile 	uint32_t 		control_missed_slots = 0;
volatile 	uint32_t 		control_slots_serviced = 0;
// Nuevas variables para compensar la diferencia entre motores
			int16_t 		deadband_L = 130;			/*!< Zona Muerta del PWM para el motor 1*/
			int16_t 		deadband_R = 75; 			/*!< Zona Muerta del PWM para el motor 2*/
// =================[ Variables de Control PID PITCH] ================= //
			float 			Kp = 190.0f;					/*!< Término Proporcional: [30] Si hay inclinación aplica una fuerza proporcional. Si se usara solo P, el robot oscilaría de un lado a otro sin quedarse quieto.*/
			float 			Ki = 1700.0f;					/*!< Término Integrativo: Elimina el error de estado estacionario*/
			float 			Kd = 3.8f;						/*!< Término Derivativo: [1.5] mide la velocidad a la que está cambiando el error. Actúa como un amortiguador*/
			float			integral_limit = 3600.0f;
			float 			setpoint = 3.0f;				/*!< Este SetPoint,se usa para desbalancer o caminar */
			float 			setpointDeEquilibrio = 0.1f;	/*!< Set Point de equilibrio, el cero del robot, el punto en el qeu el robot queda a vertical*/
			float 			integral = 0;
			float 			last_error = 0;
			float           ALPHA_PID = 0.96f;
			float			correccionRCSP = 0.98;
			float 			limite_inclinacion = 3.0f;
// =================[ Variables de Control PID YAW] ================= //
			float 		Kp_yaw = 1500.0f;
			float 		Kd_yaw = 0.0f;
			float 		last_error_yaw = 0;
volatile    float 		FL_setpoint = 4.4f;
			float 		yaw_error_filter_alpha = 0.70f;
			float 		yaw_steering_step_max = 90.0f;
			float 		yaw_steering_limit = 500.0f;
			float 		last_state_linea = 0.0f;
			float 		error_linea;
volatile 	uint16_t 	adc_filtrado[8] = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000};
volatile 	uint8_t 	estado_sensores[4]	= {0, 0, 0, 0};
volatile 	uint8_t 	ultimo_estado_sensores[4] = {0, 0, 0, 0};
volatile 	uint16_t 	sensor_min[4]= {0, 0, 0, 0};
volatile 	uint16_t 	sensor_max[4]= {0, 0, 0, 0};
volatile 	uint16_t 	sensor_threshold[4]= {0, 0, 0, 0};
volatile    uint16_t    obstacle_ir_raw = 0;
volatile    uint16_t    obstacle_ir_filtered = 0;
volatile    uint16_t    obstacle_ir_baseline = 0;
volatile    uint16_t    obstacle_ir_enter_threshold = 0;
volatile    uint16_t    obstacle_ir_exit_threshold = 0;
volatile    uint8_t     obstacle_detected = 0;
volatile    uint8_t     obstacle_event_pending = 0;
volatile 	uint8_t 	flag_calibrando_linea = 0; // Para saber en qué estado estamos
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
volatile 	uint16_t 		adc_buffer[8]; // El buffer que llena el DMA
// =================[ Modo Radio Control ] =================//
volatile 	float 			RC_setpoint 			= 0;
volatile 	int16_t   		RC_steering = 0;
volatile 	int16_t   		FL_steering = 0;
volatile    uint16_t        FL_motion_phase_ms = 200;
volatile    uint16_t        FL_balance_phase_ms = 500;
volatile    uint16_t        forward_motion_balance_only_steering = FORWARD_MOTION_DEFAULT_BALANCE_ONLY_STEERING;
float 		paso = 0.1f; // Velocidad de inclinación
// =================[ Protocolo UNER ] =================//
volatile 	uint16_t 		accelx=0;	/*!< Utilizado para refrezcar la pantalla OLED*/
volatile 	uint16_t 		accely=0;
volatile 	uint16_t		accelz=0;
volatile 	float 			giro=0;
volatile 	float			giro_z=0;
volatile    uint32_t        imu_last_update_tick = 0;
volatile 	float 			imu_accel_forward_mps2 = 0.0f;
volatile 	float 			imu_velocity_mps = 0.0f;
// =================[ I2C Scheduler ] =================//
volatile 	uint8_t 		oled_update_requested = 0;
volatile 	uint8_t 		oled_current_page = 0;
volatile 	uint8_t 		oled_is_busy = 0; // Para saber si el display está ocupado
volatile 	uint32_t 		oled_pages_sent = 0;
volatile 	uint32_t 		oled_frames_done = 0;
volatile 	uint32_t 		oled_i2c_errors = 0;
volatile 	uint32_t 		oled_busy_skips = 0;
	float P =  0;
	float I =  0;
	float D =  0;
	float output = 0;

	float Kp_Agresivo = 0.0f;
char msg[20];
// Banderas y contadores
volatile int16_t axRaw, ayRaw, azRaw, gyPitchRaw, gzYawRaw;

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
volatile uint32_t uner_tx_recover_count = 0;
volatile uint8_t uner_recovering_udp = 0;
volatile uint32_t uner_next_telemetry_tick = 0;
volatile uint8_t mpu_calibration_requested = 0;
volatile uint32_t rc_last_packet_tick = 0;
char esp01_last_debug[18] = "-";
char esp01_last_rx[18] = "-";
float showoutput=0;
float multiplicadorYaw 	 = 0.01;
float error=0;

float ForwardMotion_Generate(float motion_setpoint,
                             int16_t steering,
                             uint32_t now,
                             uint32_t *phase_tick,
                             uint8_t *motion_phase)
{
    if (*phase_tick == 0U) {
        *phase_tick = now;
        *motion_phase = 1U;
    }

    uint16_t phase_ms = *motion_phase ? FL_motion_phase_ms : FL_balance_phase_ms;
    if ((uint32_t)(now - *phase_tick) >= phase_ms) {
        *phase_tick = now;
        *motion_phase = !(*motion_phase);
    }

    int16_t steering_limit = (int16_t)forward_motion_balance_only_steering;
    if (steering > steering_limit || steering < -steering_limit) {
        return 0.0f;
    }

    return *motion_phase ? motion_setpoint : 0.0f;
}
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
void setESP01_CHPD(uint8_t val);
int ESP01_UART_Transmit(uint8_t val);
void ESP01_Data_Received(uint8_t value);
void onESP01ChangeState(_eESP01STATUS esp01State);
void onESP01Debug(const char *dbgStr);
void ESP01_App_Task(void);
_eESP01STATUS ESP01_StartTransport(void);

void UNER_HandlePacket(uint8_t cmd, uint8_t flags, uint8_t seq, uint8_t *payload, uint8_t payload_len);
void Telemetry_UpdateMPU(void);
void screenScheduler(void);
void KEY_CalibrationTask(void);
void OLED_RequestUpdate(void);
void OLED_Service(void);
/**
 * @details 							Crea una respuesta en forma de impulso con los motores la cual es proporcional
 * 										al error (pitch) del robot.	Implementa un filtro complementario para fusionar
 * 										acelerómetro y giroscopio.
 * @note 								Frecuencia de ejecución dependiente de la llegada de datos del MPU (100Hz nominal).
 */
/**
 * @brief sendCMD:						Envía un comando bajo el Protocolo UNER vía UART DMA.
 * @param cmd: 							Código del comando (CMD)
 * @param param: 						Parámetro de 16 bits (enviado en Little Endian ).
 */
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
    float accel_x_mps2 = ((((float)axRaw - accel_bias_x) / 16384.0f) * 9.80665f);
    float pitch_rad = angle_y * 0.0174532925f;

    giro = gyro_pitch_rate;
    giro_z = gyro_yaw_rate;
    angle_y = ALPHA_PID * (angle_y + gyro_pitch_rate * DT_PID) + (1.0f - ALPHA_PID) * pitch_accel;
    angle_yaw += gyro_yaw_rate * DT_PID;
    imu_accel_forward_mps2 = accel_x_mps2 - (sinf(pitch_rad) * 9.80665f);

    imu_velocity_mps += imu_accel_forward_mps2 * DT_PID;

    accelx = axRaw;
    accely = ayRaw;
    accelz = azRaw;
    imu_last_update_tick = HAL_GetTick();
}

void screenScheduler(void){
    if (!esp01_oled_ready) {
        return;
    }

    if (oled_is_busy) {
        oled_busy_skips++;
        return;
    }

    SSD1306_Fill(SSD1306_COLOR_BLACK);
    uint8_t line_s0 = estado_sensores[0] ? 1U : 0U;
    uint8_t line_s1 = estado_sensores[1] ? 1U : 0U;
    uint8_t line_s2 = estado_sensores[2] ? 1U : 0U;
    uint8_t line_s3 = estado_sensores[3] ? 1U : 0U;
    const char *screen_desc = "SYS";

    if (flagOLED < 1U || flagOLED > 9U) {
        flagOLED = 1U;
    }

    switch (flagOLED) {
    case 1: screen_desc = "WIFI"; break;
    case 2: screen_desc = "ADC"; break;
    case 3: screen_desc = "PIDP"; break;
    case 4: screen_desc = "YAW"; break;
    case 5: screen_desc = "SYS"; break;
    case 6: screen_desc = "MPU"; break;
    case 7: screen_desc = "COMP"; break;
    case 8: screen_desc = "BOX"; break;
    case 9: screen_desc = "KEY"; break;
    default: break;
    }

    snprintf(msg, sizeof(msg), "S:%u | M:%u | %s", flagOLED, currentMode, screen_desc);
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

    if (flagOLED == 1) {
        SSD1306_GotoXY(0, 10);
        SSD1306_Puts("WIRELESS", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 20);
        snprintf(msg, sizeof(msg), "IP:%s", ip_address);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 30);
        snprintf(msg, sizeof(msg), "W:%d U:%u/%u", flagWIFI, ESP.udp_connected, ESP.udp_started);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 40);
        snprintf(msg, sizeof(msg), "TX:%lu RX:%lu", (unsigned long)esp01_tx_count, (unsigned long)esp01_rx_count);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 50);
        snprintf(msg, sizeof(msg), "P:%lu A:%lu B:%lu", (unsigned long)esp01_payload_count,
                 (unsigned long)esp01_ack_count, (unsigned long)uner_tx_busy_count);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        OLED_RequestUpdate();
        return;
    }

    if (flagOLED == 2) {
        for (int i = 0; i < 8; i++) {
            uint8_t barWidth = (uint8_t)((adc_filtrado[i] * 42U) / 4095U);
            uint8_t yPos = 9U + (uint8_t)(i * 7U);
            snprintf(msg, sizeof(msg), "%u:%4u", i, adc_filtrado[i]);
            SSD1306_GotoXY(0, yPos);
            SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
            SSD1306_DrawRectangle(48, yPos, 42, 5, SSD1306_COLOR_WHITE);
            SSD1306_DrawFilledRectangle(48, yPos, barWidth, 5, SSD1306_COLOR_WHITE);
        }
        OLED_RequestUpdate();
        return;
    }

    if (flagOLED == 3) {
        SSD1306_GotoXY(0, 10);
        snprintf(msg, sizeof(msg), "P:%4.1f SP:%4.1f", angle_y, setpoint);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 20);
        snprintf(msg, sizeof(msg), "E:%5.1f O:%5.0f", error, output);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 30);
        snprintf(msg, sizeof(msg), "P:%4.0f I:%4.0f", P, I);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 40);
        snprintf(msg, sizeof(msg), "D:%4.0f G:%4.1f", D, giro);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 50);
        snprintf(msg, sizeof(msg), "Lost:%lu S:%lu", (unsigned long)control_missed_slots,
                 (unsigned long)control_slots_serviced);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        OLED_RequestUpdate();
        return;
    }

    if (flagOLED == 4) {
        SSD1306_GotoXY(0, 10);
        snprintf(msg, sizeof(msg), "Yaw:%5.1f G:%4.1f", angle_yaw, giro_z);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 20);
        snprintf(msg, sizeof(msg), "L:%u%u%u%u E:%+.2f", line_s3, line_s2, line_s1, line_s0, error_linea);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 30);
        snprintf(msg, sizeof(msg), "St:%d Lim:%4.0f", FL_steering, yaw_steering_limit);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 40);
        snprintf(msg, sizeof(msg), "Kp:%4.0f Kd:%3.0f", Kp_yaw, Kd_yaw);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 50);
        snprintf(msg, sizeof(msg), "IP:%.15s", ip_address);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        OLED_RequestUpdate();
        return;
    }

    if (flagOLED == 5) {
        uint32_t now = HAL_GetTick();
        SSD1306_GotoXY(0, 10);
        snprintf(msg, sizeof(msg), "Up:%lus Mot:%u", (unsigned long)(now / 1000U), flagMotorsAreOn);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 20);
        snprintf(msg, sizeof(msg), "Cal:%u Ready:%u", flag_calibrando_linea, flagCalibrationIsReady);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 30);
        snprintf(msg, sizeof(msg), "V:%+.2f A:%+.2f", imu_velocity_mps, imu_accel_forward_mps2);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 40);
        snprintf(msg, sizeof(msg), "IP:%s", ip_address);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 50);
        snprintf(msg, sizeof(msg), "OLED:%lu E:%lu", (unsigned long)oled_frames_done,
                 (unsigned long)oled_i2c_errors);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        OLED_RequestUpdate();
        return;
    }

    if (flagOLED == 6) {
        SSD1306_GotoXY(0, 10);
        snprintf(msg, sizeof(msg), "P:%4.1f R:%4.1f", angle_y, angle_roll);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 20);
        snprintf(msg, sizeof(msg), "Y:%5.1f Gy:%4.1f", angle_yaw, giro_z);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 30);
        snprintf(msg, sizeof(msg), "AX:%d", axRaw);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(64, 30);
        snprintf(msg, sizeof(msg), "AY:%d", ayRaw);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 40);
        snprintf(msg, sizeof(msg), "AZ:%d", azRaw);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(64, 40);
        snprintf(msg, sizeof(msg), "GP:%d", gyPitchRaw);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 50);
        snprintf(msg, sizeof(msg), "GZ:%d C:%u", gzYawRaw, flagCalibrationIsReady);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        OLED_RequestUpdate();
        return;
    }

    if (flagOLED == 7) {
        static uint8_t compass_ready = 0;
        static float compass_north_yaw = 0.0f;
        if (!compass_ready) {
            compass_north_yaw = angle_yaw;
            compass_ready = 1U;
        }
        float rel_yaw = angle_yaw - compass_north_yaw;
        while (rel_yaw < 0.0f) rel_yaw += 360.0f;
        while (rel_yaw >= 360.0f) rel_yaw -= 360.0f;

        const int16_t cx = 39;
        const int16_t cy = 36;
        const int16_t r = 21;
        const float rad = rel_yaw * 0.01745329252f;
        const int16_t nx = (int16_t)(cx + sinf(rad) * 17.0f);
        const int16_t ny = (int16_t)(cy - cosf(rad) * 17.0f);
        const int16_t tx = (int16_t)(cx - sinf(rad) * 7.0f);
        const int16_t ty = (int16_t)(cy + cosf(rad) * 7.0f);

        SSD1306_DrawCircle(cx, cy, r, SSD1306_COLOR_WHITE);
        SSD1306_DrawCircle(cx, cy, r - 1, SSD1306_COLOR_WHITE);
        SSD1306_DrawLine(cx, cy - r, cx, cy - r + 4, SSD1306_COLOR_WHITE);
        SSD1306_DrawLine(cx + r - 4, cy, cx + r, cy, SSD1306_COLOR_WHITE);
        SSD1306_DrawLine(cx, cy + r - 4, cx, cy + r, SSD1306_COLOR_WHITE);
        SSD1306_DrawLine(cx - r, cy, cx - r + 4, cy, SSD1306_COLOR_WHITE);
        SSD1306_DrawLine(tx, ty, nx, ny, SSD1306_COLOR_WHITE);
        SSD1306_DrawFilledCircle(nx, ny, 2, SSD1306_COLOR_WHITE);
        SSD1306_DrawFilledCircle(cx, cy, 1, SSD1306_COLOR_WHITE);

        SSD1306_GotoXY(36, 12);
        SSD1306_Puts("N", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(13, 33);
        SSD1306_Puts("W", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(61, 33);
        SSD1306_Puts("E", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(36, 51);
        SSD1306_Puts("S", &Font_7x10, SSD1306_COLOR_WHITE);

        SSD1306_GotoXY(84, 18);
        SSD1306_Puts("YAW", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(82, 32);
        snprintf(msg, sizeof(msg), "%6.1f", rel_yaw);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(93, 44);
        SSD1306_Puts("deg", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(82, 54);
        snprintf(msg, sizeof(msg), "Raw:%4.0f", angle_yaw);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        OLED_RequestUpdate();
        return;
    }

    if (flagOLED == 8) {
        SSD1306_GotoXY(0, 10);
        snprintf(msg, sizeof(msg), "A:%u S:%u F:%u", obstacle_follow_active,
                 obstacle_follow_state, obstacle_right_face_state);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 20);
        snprintf(msg, sizeof(msg), "R:%4u F:%4u", obstacle_right_ir_raw, obstacle_right_ir_filtered);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 30);
        snprintf(msg, sizeof(msg), "Fr:%4u %u", obstacle_ir_filtered, obstacle_detected);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 40);
        snprintf(msg, sizeof(msg), "T:%4u E:%5d", obstacle_follow_target_adc, obstacle_follow_adc_error);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 50);
        snprintf(msg, sizeof(msg), "St:%4d T%u S%u", obstacle_follow_steering,
                 turn_maneuver_active, obstacle_follow_steering_saturated);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        OLED_RequestUpdate();
        return;
    }

    if (flagOLED == 9) {
        const char *event_label = "--";
        const char *action_label = "--";
        uint32_t now = HAL_GetTick();
        uint32_t event_age_s = (key_last_event == KEY_EVENT_NONE) ? 0U :
                               (uint32_t)((now - key_last_event_tick) / 1000U);

        switch (key_last_event) {
        case KEY_EVENT_SHORT_PRESS: event_label = "S"; break;
        case KEY_EVENT_DOUBLE_CLICK: event_label = "D"; break;
        case KEY_EVENT_LONG_PRESS: event_label = "L"; break;
        default: break;
        }

        switch (key_last_action) {
        case KEY_ACTION_CAL_START: action_label = "CAL+"; break;
        case KEY_ACTION_CAL_DONE: action_label = "CAL-"; break;
        case KEY_ACTION_MODE_RC: action_label = "RC"; break;
        case KEY_ACTION_MODE_FL: action_label = "FL"; break;
        case KEY_ACTION_MOTORS_ON: action_label = "ON"; break;
        case KEY_ACTION_MOTORS_OFF: action_label = "OFF"; break;
        default: break;
        }

        SSD1306_GotoXY(0, 10);
        snprintf(msg, sizeof(msg), "Btn:%c Raw:%c", key_stable_pressed ? 'P' : 'R',
                 key_raw_pressed ? 'P' : 'R');
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 20);
        snprintf(msg, sizeof(msg), "Pend:%u Long:%u", key_click_pending, key_long_press_done);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 30);
        snprintf(msg, sizeof(msg), "Hold:%lu/%ums", (unsigned long)key_press_duration_ms, KEY_LONG_PRESS_MS);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 40);
        snprintf(msg, sizeof(msg), "E:%s A:%s %lus", event_label, action_label, (unsigned long)event_age_s);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 50);
        snprintf(msg, sizeof(msg), "Cal:%u Mot:%u", flag_calibrando_linea, flagMotorsAreOn);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        OLED_RequestUpdate();
        return;
    }

    if (flagOLED == 0) {
        SSD1306_GotoXY(0, 0);
        SSD1306_Puts("LINE ERROR", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 14);
        sprintf(msg, "E:%+1.3f", error_linea);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 28);
        sprintf(msg, "L:%u%u%u%u", line_s3, line_s2, line_s1, line_s0);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 42);
        snprintf(msg, sizeof(msg), "R:%u %u", adc_buffer[0], adc_buffer[1]);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 54);
        snprintf(msg, sizeof(msg), "R:%u %u", adc_buffer[2], adc_buffer[3]);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        OLED_RequestUpdate();
        return;
    }

    if (flagOLED == 1) {
        SSD1306_GotoXY(0, 0);
        SSD1306_Puts("PITCH PID", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 10);
        snprintf(msg, sizeof(msg), "Kp:%5.1f", Kp);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 20);
        snprintf(msg, sizeof(msg), "Ki:%4.2f Kd:%3.1f", Ki, Kd);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 30);
        snprintf(msg, sizeof(msg), "SP:%4.1f A:%3.1f", setpoint, angle_y);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 40);
        snprintf(msg, sizeof(msg), "Out:%5.0f", showoutput);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 50);
        snprintf(msg, sizeof(msg), "Lost:%lu", (unsigned long)control_missed_slots);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        OLED_RequestUpdate();
        return;
    }

    if (flagOLED == 2) {
           SSD1306_GotoXY(0, 0);
           SSD1306_Puts("OBST FOLLOW DBG", &Font_7x10, SSD1306_COLOR_WHITE);
           SSD1306_GotoXY(0, 10);
           snprintf(msg, sizeof(msg), "A:%u S:%u F:%u",
                    obstacle_follow_active,
                    obstacle_follow_state,
                    obstacle_right_face_state);
           SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
           SSD1306_GotoXY(0, 20);
           snprintf(msg, sizeof(msg), "R:%4u F:%4u",
                    obstacle_right_ir_raw,
                    obstacle_right_ir_filtered);
           SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
           SSD1306_GotoXY(0, 30);
           snprintf(msg, sizeof(msg), "T:%4u E:%5d",
                    obstacle_follow_target_adc,
                    obstacle_follow_adc_error);
           SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
           SSD1306_GotoXY(0, 40);
           snprintf(msg, sizeof(msg), "Sd:%d O:%d",
                    obstacle_follow_side_steering,
                    obstacle_follow_steering);
           SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
           SSD1306_GotoXY(0, 52);
           int16_t yaw_dbg = (int16_t)angle_yaw;
           int16_t yaw_err_dbg = obstacle_follow_yaw_error_cdeg / 100;
           if (yaw_dbg > 999) yaw_dbg = 999;
           if (yaw_dbg < -999) yaw_dbg = -999;
           if (yaw_err_dbg > 99) yaw_err_dbg = 99;
           if (yaw_err_dbg < -99) yaw_err_dbg = -99;
           snprintf(msg, sizeof(msg), "Y:%d Ey:%d C:%u",
                    yaw_dbg,
                    yaw_err_dbg,
                    obstacle_follow_steering_saturated);
           SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);


           OLED_RequestUpdate();
           return;
       }
    if (flagOLED == 3) {
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
			  return;
    }

    if (flagOLED == 4) {
        SSD1306_GotoXY(0, 0);
        SSD1306_Puts("OBST IR FRONT", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 10);
        snprintf(msg, sizeof(msg), "RAW:%4u F:%4u", obstacle_ir_raw, obstacle_ir_filtered);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 22);
        snprintf(msg, sizeof(msg), "BASE:%4u", obstacle_ir_baseline);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 34);
        snprintf(msg, sizeof(msg), "ON:%4u OFF:%4u", obstacle_ir_enter_threshold, obstacle_ir_exit_threshold);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 46);
        snprintf(msg, sizeof(msg), "D:%u E:%u T:%u",
                 obstacle_detected,
                 obstacle_event_pending,
                 turn_maneuver_active);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        OLED_RequestUpdate();
        return;
    }

    if (flagOLED == 5) {
        const char *event_label = "--";
        const char *action_label = "--";
        uint32_t now = HAL_GetTick();
        uint32_t event_age_s = (key_last_event == KEY_EVENT_NONE) ? 0U :
                               (uint32_t)((now - key_last_event_tick) / 1000U);

        switch (key_last_event) {
        case KEY_EVENT_SHORT_PRESS:
            event_label = "S";
            break;
        case KEY_EVENT_DOUBLE_CLICK:
            event_label = "D";
            break;
        case KEY_EVENT_LONG_PRESS:
            event_label = "L";
            break;
        default:
            break;
        }

        switch (key_last_action) {
        case KEY_ACTION_CAL_START:
            action_label = "CAL+";
            break;
        case KEY_ACTION_CAL_DONE:
            action_label = "CAL-";
            break;
        case KEY_ACTION_MODE_RC:
            action_label = "RC";
            break;
        case KEY_ACTION_MODE_FL:
            action_label = "FL";
            break;
        case KEY_ACTION_MOTORS_ON:
            action_label = "ON";
            break;
        case KEY_ACTION_MOTORS_OFF:
            action_label = "OFF";
            break;
        default:
            break;
        }

        SSD1306_GotoXY(0, 0);
        SSD1306_Puts("KEY DEBUG", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 10);
        snprintf(msg, sizeof(msg), "M:%u Mot:%s", currentMode, flagMotorsAreOn ? "ON" : "OFF");
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 20);
        snprintf(msg, sizeof(msg), "Cal:%u Pend:%u", flag_calibrando_linea, key_click_pending);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 30);
        snprintf(msg, sizeof(msg), "Btn:%c Raw:%c L:%u",
                 key_stable_pressed ? 'P' : 'R',
                 key_raw_pressed ? 'P' : 'R',
                 key_long_press_done);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 40);
        snprintf(msg, sizeof(msg), "Hold:%lu/%ums",
                 (unsigned long)key_press_duration_ms,
                 KEY_LONG_PRESS_MS);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 52);
        snprintf(msg, sizeof(msg), "E:%s A:%s %lus",
                 event_label,
                 action_label,
                 (unsigned long)event_age_s);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        OLED_RequestUpdate();
        return;
    }

    if (flagOLED == 777) {
        SSD1306_GotoXY(0, 0);
        SSD1306_Puts("IR/MPU", &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 12);
        sprintf(msg, "P:%3.1f Y:%3.1f", angle_y, angle_yaw);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 24);
        sprintf(msg, "IR:%4u %4u", adc_filtrado[0], adc_filtrado[1]);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 36);
        sprintf(msg, "IR:%4u %4u", adc_filtrado[2], adc_filtrado[3]);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 48);
        sprintf(msg, "V:%2.2f A:%2.2f", imu_velocity_mps, imu_accel_forward_mps2);
        SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
        OLED_RequestUpdate();
        return;
    }

    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("ESP01 UDP", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(0, 10);
    SSD1306_Puts("", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(24, 10);
    SSD1306_Puts(ip_address, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(0, 20);
    sprintf(msg, "W:%d U:%d L:%u%u%u%u", flagWIFI, ESP.udp_connected, line_s3, line_s2, line_s1, line_s0);
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
    snprintf(msg, sizeof(msg), "B:%lu R:%lu %.3s",
             (unsigned long)uner_tx_busy_count,
             (unsigned long)uner_tx_recover_count,
             esp01_last_debug);
    SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);
    OLED_RequestUpdate();
    return;

	if (!oled_is_busy) {
		SSD1306_Fill(SSD1306_COLOR_BLACK);
			switch(flagOLED){
			case 0:
				uint8_t sensores= (estado_sensores[2] * 100) + (estado_sensores[1] * 10) + estado_sensores[0];
				uint8_t lastSensores = (ultimo_estado_sensores[2] * 100) + (ultimo_estado_sensores[1] * 10) + ultimo_estado_sensores[0];

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

				sprintf(msg, "M:%d ", currentMode);
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

void OLED_RequestUpdate(void)
{
    if (!oled_is_busy) {
        oled_current_page = 0;
    }
    oled_update_requested = 1;
}

void OLED_Service(void)
{
    if (!esp01_oled_ready || !oled_update_requested || oled_is_busy || flag10ms) {
        return;
    }

    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
        return;
    }

    SSD1306_UpdatePage_DMA(oled_current_page);
}
void UNER_HandlePacket(uint8_t cmd, uint8_t flags, uint8_t seq, uint8_t *payload, uint8_t payload_len)
{
    switch (cmd) {
    case CMD_ALIVE:
        esp01_alive_count++;
        uner_ack_cmd = CMD_ALIVE;
        uner_ack_seq = seq;
        uner_ack_status = 0;
        uner_ack_pending = 1;
        break;

    case CMD_TELEMETRY_START:
        uner_telemetry_enabled = 1;
        uner_next_telemetry_tick = HAL_GetTick();
        uner_ack_cmd = CMD_TELEMETRY_START;
        uner_ack_seq = seq;
        uner_ack_status = 0;
        uner_ack_pending = 1;
        break;
    case CMD_TELEMETRY_STOP:
        uner_telemetry_enabled = 0;
        uner_ack_cmd = CMD_TELEMETRY_STOP;
        uner_ack_seq = seq;
        uner_ack_status = 0;
        uner_ack_pending = 1;
        break;
    case CMD_SET_YAW_PD:
        if (payload_len >= 8) {
            Kp_yaw = UNER_ReadFloatLE(payload);
            Kd_yaw = UNER_ReadFloatLE(payload + 4);
            uner_ack_status = 0;
        } else {
            uner_ack_status = 2;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_SET_YAW_PD;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;
    case CMD_SET_YAW_CONFIG:
        if (payload_len >= 20) {
            float kp = UNER_ReadFloatLE(payload);
            float kd = UNER_ReadFloatLE(payload + 4);
            float curve_mul = UNER_ReadFloatLE(payload + 8);
            float filter_alpha = UNER_ReadFloatLE(payload + 12);
            float steering_step = UNER_ReadFloatLE(payload + 16);
            float steering_limit = yaw_steering_limit;

            if (payload_len >= 24) {
                steering_limit = UNER_ReadFloatLE(payload + 20);
            }

            if (kp >= 0.0f && kp <= 20000.0f &&
                kd >= 0.0f && kd <= 5000.0f &&
                curve_mul >= 0.0f && curve_mul <= 1.0f &&
                filter_alpha >= 0.0f && filter_alpha <= 0.995f &&
                steering_step >= 1.0f && steering_step <= 2000.0f &&
                steering_limit >= 0.0f && steering_limit <= 5000.0f) {
                Kp_yaw = kp;
                Kd_yaw = kd;
                multiplicadorYaw = curve_mul;
                yaw_error_filter_alpha = filter_alpha;
                yaw_steering_step_max = steering_step;
                yaw_steering_limit = steering_limit;
                uner_ack_status = 0;
            } else {
                uner_ack_status = 1;
            }
        } else {
            uner_ack_status = 2;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_SET_YAW_CONFIG;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;
    case CMD_SET_FL_CONFIG:
        if (payload_len >= 6) {
            float fl_sp = UNER_ReadFloatLE(payload);
            uint16_t motion_ms = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
            uint16_t balance_ms = motion_ms;
            uint16_t balance_only_steering = forward_motion_balance_only_steering;

            if (payload_len >= 8) {
                balance_ms = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8);
            }
            if (payload_len >= 10) {
                balance_only_steering = (uint16_t)payload[8] | ((uint16_t)payload[9] << 8);
            }

            if (fl_sp >= -10.0f && fl_sp <= 90.0f &&
                motion_ms >= 20U && motion_ms <= 2000U &&
                balance_ms >= 20U && balance_ms <= 5000U &&
                balance_only_steering <= 4000U) {
                FL_setpoint = fl_sp;
                FL_motion_phase_ms = motion_ms;
                FL_balance_phase_ms = balance_ms;
                forward_motion_balance_only_steering = balance_only_steering;
                uner_ack_status = 0;
            } else {
                uner_ack_status = 1;
            }
        } else {
            uner_ack_status = 2;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_SET_FL_CONFIG;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;
    case CMD_TURN_MANEUVER:
        if (payload_len >= 4) {
            int16_t target_cdeg = UNER_ReadInt16LE(payload);
            float target_angle_deg = ((float)target_cdeg) / 100.0f;
            uint8_t wheel_mode = payload[2];
            uint8_t wheel_select = payload[3];

            if (wheel_mode == TURN_MANEUVER_MODE_ARC) {
                if (payload_len >= 5) {
                    uner_ack_status = TurnManeuver_StartArc(target_angle_deg, wheel_select, payload[4]);
                } else {
                    uner_ack_status = TURN_MANEUVER_STATUS_PAYLOAD;
                }
            } else {
                uner_ack_status = TurnManeuver_Start(target_angle_deg, wheel_mode, wheel_select);
            }
        } else {
            uner_ack_status = TURN_MANEUVER_STATUS_PAYLOAD;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_TURN_MANEUVER;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;

    case CMD_OBSTACLE_FOLLOW:
        if (payload_len >= 1) {
            uint8_t action = payload[0];
            uint8_t side = (payload_len >= 2) ? payload[1] : OBSTACLE_FOLLOW_SIDE_RIGHT;

            if (action == 0U) {
                ObstacleFollow_Stop();
                if (currentMode == MODO_OBSTACLE_FOLLOW) {
                    currentMode = MODO_RC;
                }
                uner_ack_status = OBSTACLE_FOLLOW_STATUS_OK;
            } else if (action == 1U) {
                uner_ack_status = ObstacleFollow_Start(side);
            } else {
                uner_ack_status = OBSTACLE_FOLLOW_STATUS_RANGE;
            }
        } else {
            uner_ack_status = OBSTACLE_FOLLOW_STATUS_RANGE;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_OBSTACLE_FOLLOW;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;

    case CMD_START:
        Control_SetMotorsEnabled(1U);
        uner_ack_status = 0;
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_START;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;

    case CMD_STOP:
        Control_SetMotorsEnabled(0U);
        uner_ack_status = 0;
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_STOP;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;
    case CMD_CALIBRATE:
        flagCalibrationIsReady = 0;
        mpu_calibration_requested = 1;
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_CALIBRATE;
            uner_ack_seq = seq;
            uner_ack_status = 0;
            uner_ack_pending = 1;
        }
        break;
    case CMD_SET_HB:
        if (payload_len >= 1) {
            uint16_t requested_delay = payload[0];
            if (payload_len >= 2) {
                requested_delay = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
            }
            if (requested_delay >= 1 && requested_delay <= 200) {
                delayHB = requested_delay;
                uner_ack_status = 0;
            } else {
                uner_ack_status = 1;
            }
        } else {
            uner_ack_status = 2;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_SET_HB;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;
    case CMD_CHANGE_MODE:
        if (payload_len >= 2) {
            int16_t requested_mode = UNER_ReadInt16LE(payload);
            if (flag_calibrando_linea) {
                uner_ack_status = 1;
            } else if (requested_mode >= MODO_IDDLE && requested_mode <= MODO_FL_INGRESO_A_90) {
                if (requested_mode != MODO_RC && requested_mode != MODO_OBSTACLE_FOLLOW) {
                    TurnManeuver_Cancel();
                }
                if (requested_mode != MODO_OBSTACLE_FOLLOW) {
                    ObstacleFollow_Stop();
                    currentMode = (uint8_t)requested_mode;
                    uner_ack_status = 0;
                } else {
                    uner_ack_status = ObstacleFollow_Start(OBSTACLE_FOLLOW_SIDE_RIGHT);
                }
            } else {
                uner_ack_status = 1;
            }
        } else {
            uner_ack_status = 2;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_CHANGE_MODE;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;
    case CMD_ONOFFMOTORS:
        if (payload_len >= 1) {
            Control_SetMotorsEnabled((payload[0] != 0) ? 1U : 0U);
            uner_ack_status = 0;
        } else {
            uner_ack_status = 2;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_ONOFFMOTORS;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;
    case CMD_CHANGE_DEADLINE_LEFT:
        uner_ack_status = 2;
        if (payload_len >= 2) {
            deadband_L = UNER_ReadInt16LE(payload);
            uner_ack_status = 0;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_CHANGE_DEADLINE_LEFT;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;
    case CMD_CHANGE_DEADLINE_RIGHT:
        uner_ack_status = 2;
        if (payload_len >= 2) {
            deadband_R = UNER_ReadInt16LE(payload);
            uner_ack_status = 0;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_CHANGE_DEADLINE_RIGHT;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;
    case CMD_CHANGE_SETPOINT:
        if (payload_len >= 4) {
            setpoint = UNER_ReadFloatLE(payload);
            uner_ack_status = 0;
        } else if (payload_len >= 2) {
            setpoint = (float)UNER_ReadInt16LE(payload);
            uner_ack_status = 0;
        } else {
            uner_ack_status = 2;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_CHANGE_SETPOINT;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;
    case CMD_CHANGE_SETPOINT_FL:
        if (payload_len >= 4) {
            FL_setpoint = UNER_ReadFloatLE(payload);
            uner_ack_status = 0;
        } else if (payload_len >= 2) {
            FL_setpoint = (float)UNER_ReadInt16LE(payload);
            uner_ack_status = 0;
        } else {
            uner_ack_status = 2;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_CHANGE_SETPOINT_FL;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;

    case CMD_DEFINE_ZERO_SETPOINT:
        if (payload_len >= 4) {
            setpointDeEquilibrio = UNER_ReadFloatLE(payload);
        } else {
            setpointDeEquilibrio = angle_y;
        }
        uner_ack_status = 0;
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_DEFINE_ZERO_SETPOINT;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;

    case CMD_CHANGE_OLED_SCREEN:
        if (payload_len >= 1) {
            uint16_t requested_page = payload[0];
            if (payload_len >= 2) {
                requested_page = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
            }
            if (requested_page >= 1U && requested_page <= 9U) {
                flagOLED = (uint8_t)requested_page;
                uner_ack_status = 0;
            } else {
                uner_ack_status = 1;
            }
        } else {
            uner_ack_status = 2;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_CHANGE_OLED_SCREEN;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;
    case CMD_RC:
        if (payload_len >= 4) {
            if (currentMode != MODO_RC || turn_maneuver_active) {
                break;
            }
            uint8_t active = payload[0];
            int8_t setpoint_cmd = (int8_t)payload[1];
            int16_t steering_cmd = UNER_ReadInt16LE(&payload[2]);
            rc_last_packet_tick = HAL_GetTick();
            flag_RC_active = (active != 0) ? 1 : 0;
            if (flag_RC_active) {
                if (setpoint_cmd > 0) {
                    RC_setpoint = 1.0f;
                } else if (setpoint_cmd < 0) {
                    RC_setpoint = -1.0f;
                } else {
                    RC_setpoint = 0.0f;
                }
                RC_steering = steering_cmd;
            } else {
                RC_setpoint = 0.0f;
                RC_steering = 0;
            }
        }
        break;
    case CMD_PID_PITCH_KP:
    case CMD_PID_PITCH_KI:
    case CMD_PID_PITCH_KD:
    case CMD_PID_ALPHA:
    case CMD_PID_PITCH_LIM_INCLI:
    case CMD_PID_PITCH_CORECCION_RCSP:
    case CMD_PID_YAW_KP:
    case CMD_PID_YAW_KD:
    case CMD_PID_YAW_SP:
    case CMD_PID_YAW_MULTIPLICADOR:
    case CMD_PID_INTEGRAL_LIMIT:
        if (payload_len >= 4) {
            float value = UNER_ReadFloatLE(payload);
            switch (cmd) {
            case CMD_PID_PITCH_KP: Kp = value; break;
            case CMD_PID_PITCH_KI: Ki = value; break;
            case CMD_PID_PITCH_KD: Kd = value; break;
            case CMD_PID_ALPHA: ALPHA_PID = value; break;
            case CMD_PID_PITCH_LIM_INCLI: 		limite_inclinacion = value; break;
            case CMD_PID_PITCH_CORECCION_RCSP: 	correccionRCSP = value; break;
            case CMD_PID_YAW_KP: Kp_yaw = value; break;
            case CMD_PID_YAW_KD: Kd_yaw = value; break;
            case CMD_PID_YAW_SP: FL_setpoint = value; break;
            case CMD_PID_YAW_MULTIPLICADOR: multiplicadorYaw = value; break;
            case CMD_PID_INTEGRAL_LIMIT:
                integral_limit = value;
                if (integral_limit < 0.0f) {
                    integral_limit = 0.0f;
                } else if (integral_limit > 3600.0f) {
                    integral_limit = 3600.0f;
                }
                break;
            default: break;
            }
            uner_ack_status = 0;
        } else if (payload_len >= 2) {
            float value = (float)UNER_ReadInt16LE(payload);
            uint8_t status = 0;
            switch (cmd) {
            case CMD_PID_PITCH_LIM_INCLI: limite_inclinacion = value; break;
            case CMD_PID_PITCH_CORECCION_RCSP: correccionRCSP = value; break;
            case CMD_PID_YAW_SP: FL_setpoint = value; break;
            default: status = 1; break;
            }
            uner_ack_status = status;
        } else {
            uner_ack_status = 2;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = cmd;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;
    case CMD_IR_INICIAR_CALIBRACION:
        if (currentMode == MODO_IDDLE && !flag_calibrando_linea) {
            TurnManeuver_Cancel();
            ObstacleFollow_Stop();
            flag_RC_active = 0;
            RC_setpoint = 0.0f;
            RC_steering = 0;
            FL_steering = 0;
            currentMode = MODO_IDDLE;
            Iniciar_Calibracion_Linea();
            uner_ack_status = 0;
        } else {
            uner_ack_status = 1;
        }
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_IR_INICIAR_CALIBRACION;
            uner_ack_seq = seq;
            uner_ack_pending = 1;
        }
        break;
    case CMD_IR_DETENER_CALIBRACION:
        Finalizar_Calibracion_Linea();
        currentMode = MODO_FL_BUSQUEDA_INICIAL;
        if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
            uner_ack_cmd = CMD_IR_DETENER_CALIBRACION;
            uner_ack_seq = seq;
            uner_ack_status = 0;
            uner_ack_pending = 1;
        }
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
void ESP01_Generic_Functions(uint32_t now){
	if (uner_telemetry_enabled && !uner_ack_pending && ESP.udp_connected && !uner_tx_busy && ESP.uner_tx_count == 0 &&
		(int32_t)(now - uner_next_telemetry_tick) >= 0) {
		if (UNER_SendTelemetryV1()) {
			uner_next_telemetry_tick = now + UNER_TELEMETRY_PERIOD_MS;
		} else {
			uner_next_telemetry_tick = now + 50;
		}
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance == TIM4) {
        ESP01_Timeout10ms();
        if (flag10ms) {
            control_missed_slots++;
        } else {
            flag10ms=1;
        }
        counterHB++;
        if (counterHB >= delayHB) {
            counterHB = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
    }
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1 && oled_is_busy) {
        oled_is_busy = 0;
        oled_pages_sent++;
        oled_current_page++;
        if (oled_current_page >= 8U) {
            oled_current_page = 0;
            oled_update_requested = 0;
            oled_frames_done++;
        }
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1 && oled_is_busy) {
        oled_i2c_errors++;
        oled_is_busy = 0;
        oled_current_page = 0;
        oled_update_requested = 0;
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART1) {
        uint8_t value = ESP.AT_Rx_data;
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
        HAL_UART_Receive_IT(&huart1, &ESP.AT_Rx_data, 1);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart1, &ESP.AT_Rx_data, 1);
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
      MX_TIM2_Init();
      MX_TIM3_Init();
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
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
      Robot_Drive(0, 0);
      ESP.uner_rx_read = 0;
      ESP.uner_rx_write = 0;
      ESP.uner_tx_head = 0;
      ESP.uner_tx_tail = 0;
      ESP.uner_tx_count = 0;
      ESP.udp_started = 0;
      ESP.Config.DoCHPD = setESP01_CHPD;
      ESP.Config.WriteUSARTByte = ESP01_UART_Transmit;
      ESP.Config.WriteByteToBufRX = ESP01_Data_Received;
      ESP01_Init(&ESP.Config);
      ESP01_AttachChangeState(&onESP01ChangeState);
      ESP01_AttachDebugStr(&onESP01Debug);
      HAL_UART_Receive_IT(&huart1, &ESP.AT_Rx_data, 1);
      ESP01_SetWIFI(WIFI_SSID, WIFI_PASSWORD);
      ESP01_StartTransport();
      ESP.udp_started = 1;
      HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	static uint32_t last_oled_update = 0;
	  	static uint32_t last_ir_update = 0;
	  	static uint8_t last_motors_are_on = 0;
		uint32_t now = HAL_GetTick();

		if (flagMotorsAreOn != last_motors_are_on) {
			PID_PITCH_ResetState();
			last_motors_are_on = flagMotorsAreOn;
		}

		if (((now - last_ir_update) >= 1U) || flag10ms) {
			last_ir_update = now;
			Filtrar_Sensores_IR();
			ObstacleSensor_Task();
			Leer_Linea_Digital();
			if (flag_calibrando_linea){	Procesar_Calibracion_Linea();}
		}
		if(flag10ms){
			flag10ms = 0;
			Telemetry_UpdateMPU();
			if (!flag_calibrando_linea) {
				ObstacleFollow_Task();
				FollowLine_Task();
				if (currentMode == MODO_RC && flag_RC_active && (int32_t)(now - rc_last_packet_tick) > RC_TIMEOUT_MS) {
					flag_RC_active = 0;
					RC_setpoint = 0.0f;
					RC_steering = 0;
				}
				TurnManeuver_Task();
			}
			PID_PITCH();
			control_slots_serviced++;
			OLED_Service();
		}

	  	ESP01_App_Task();
	    KEY_CalibrationTask();
		UNER_Rx_Task();
		UNER_Tx_Task();
		if((now - last_oled_update)>=500){last_oled_update = now;screenScheduler();	}
		ESP01_Generic_Functions(now);
		ESP01_Task();
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
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 7;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 8;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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

  /*Configure GPIO pin : KEY PA0 */
  GPIO_InitStruct.Pin = KEY_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStruct);

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
