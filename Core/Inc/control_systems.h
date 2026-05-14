#ifndef CONTROL_SYSTEMS_H
#define CONTROL_SYSTEMS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_DT_PID     0.01f
#define CONTROL_MODE_IDLE  0U
#define CONTROL_MODE_RC    1U

extern volatile uint8_t currentMode;
extern volatile uint8_t flagMotorsAreOn;
extern volatile int16_t axRaw;
extern volatile int16_t ayRaw;
extern volatile int16_t azRaw;
extern volatile uint16_t accelx;
extern volatile uint16_t accely;
extern volatile uint16_t accelz;
extern volatile float giro;
extern volatile float RC_setpoint;
extern volatile float RC_slow_setpoint;
extern volatile int16_t RC_steering;

extern float angle_y;
extern float Kp;
extern float Ki;
extern float Kd;
extern float setpoint;
extern float integral;
extern float last_error;
extern float correccionRCSP;
extern float limite_inclinacion;
extern float paso;
extern float P;
extern float I;
extern float D;
extern float output;
extern float Kp_Agresivo;
extern float showoutput;
extern float error;
extern float Kp_yaw;
extern float Kd_yaw;
extern float last_error_yaw;

void PID_PITCH(void);
int16_t Calcular_PID_YAW(float error_linea);
void Robot_Drive(int16_t speed_L, int16_t speed_R);

#ifdef __cplusplus
}
#endif

#endif
