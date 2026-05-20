#ifndef CONTROL_SYSTEMS_H
#define CONTROL_SYSTEMS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_DT_PID     0.01f
#define CONTROL_MODE_IDLE  0U
#define CONTROL_MODE_RC    1U
#define CONTROL_MODE_FL_INICIO             2U
#define CONTROL_MODE_FL_BUSQUEDA_INICIAL   3U
#define CONTROL_MODE_FL_SIGUIENDO          4U
#define CONTROL_MODE_FL_RESCATE            5U
#define CONTROL_MODE_FL_PERDIDO_FAILSAFE   6U
#define CONTROL_MODE_FL_INGRESO_A_90       8U

extern volatile uint8_t currentMode;
extern volatile uint8_t flagMotorsAreOn;
extern volatile int16_t axRaw;
extern volatile int16_t ayRaw;
extern volatile int16_t azRaw;
extern volatile uint16_t accelx;
extern volatile uint16_t accely;
extern volatile uint16_t accelz;
extern volatile float giro;
extern volatile float giro_z;
extern volatile float RC_setpoint;
extern volatile int16_t RC_steering;
extern volatile float FL_setpoint;
extern volatile int16_t FL_steering;
extern volatile uint16_t FL_motion_phase_ms;
extern volatile uint16_t FL_balance_phase_ms;

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
extern float error_linea;
extern float last_state_linea;
extern float multiplicadorYaw;
extern float yaw_error_filter_alpha;
extern float yaw_steering_step_max;

void PID_PITCH(void);
void FollowLine_Task(void);
int16_t Calcular_PID_YAW(float error_linea);
void Robot_Drive(int16_t speed_L, int16_t speed_R);

#ifdef __cplusplus
}
#endif

#endif
