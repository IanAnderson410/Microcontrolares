#ifndef LINE_SENSORS_H
#define LINE_SENSORS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FL_LINE_SENSOR_COUNT 4
#define FL_LINE_MIN_SPREAD   120U

#define IRCSAAB (estado_sensores[0] && estado_sensores[1] && estado_sensores[2] && estado_sensores[3])
#define IRLSAAB (ultimo_estado_sensores[0] && ultimo_estado_sensores[1] && ultimo_estado_sensores[2] && ultimo_estado_sensores[3])
#define IRCSAAW (!estado_sensores[0] && !estado_sensores[1] && !estado_sensores[2] && !estado_sensores[3])
#define IRLSAAW (!ultimo_estado_sensores[0] && !ultimo_estado_sensores[1] && !ultimo_estado_sensores[2] && !ultimo_estado_sensores[3])
#define AIRAB   (estado_sensores[0] || estado_sensores[1] || estado_sensores[2] || estado_sensores[3])

extern volatile uint16_t adc_filtrado[8];
extern volatile uint8_t estado_sensores[4];
extern volatile uint8_t ultimo_estado_sensores[4];
extern volatile uint16_t sensor_min[4];
extern volatile uint16_t sensor_max[4];
extern volatile uint16_t sensor_threshold[4];
extern volatile uint8_t flag_calibrando_linea;
extern volatile uint8_t flagCalibrationIsReady;
extern volatile uint16_t adc_buffer[8];
extern float error_linea;
extern float last_state_linea;

void Filtrar_Sensores_IR(void);
void Iniciar_Calibracion_Linea(void);
void Finalizar_Calibracion_Linea(void);
void Procesar_Calibracion_Linea(void);
void Leer_Linea_Digital(void);
float calcularErrorYawDiscreto(void);
float calcularErrorYawContinuo(void);

#ifdef __cplusplus
}
#endif

#endif
