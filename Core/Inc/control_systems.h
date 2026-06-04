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
#define CONTROL_MODE_OBSTACLE_FOLLOW       7U
#define CONTROL_MODE_FL_INGRESO_A_90       8U
#define FORWARD_MOTION_DEFAULT_BALANCE_ONLY_STEERING 250U
#define TURN_MANEUVER_MODE_TWO_WHEELS      0U
#define TURN_MANEUVER_MODE_ONE_WHEEL       1U
#define TURN_MANEUVER_MODE_ARC             2U
#define TURN_MANEUVER_WHEEL_LEFT           0U
#define TURN_MANEUVER_WHEEL_RIGHT          1U
#define TURN_MANEUVER_STATUS_OK            0U
#define TURN_MANEUVER_STATUS_RANGE         1U
#define TURN_MANEUVER_STATUS_PAYLOAD       2U
#define TURN_MANEUVER_STATUS_MODE          3U
#define TURN_MANEUVER_STATUS_SENSOR        4U
#define TURN_MANEUVER_EXIT_NONE            0U
#define TURN_MANEUVER_EXIT_TARGET_REACHED  1U
#define TURN_MANEUVER_EXIT_TIMEOUT         2U
#define TURN_MANEUVER_EXIT_MOTORS_OFF      3U
#define TURN_MANEUVER_EXIT_MODE_CHANGE     4U
#define TURN_MANEUVER_EXIT_IMU_STALE       5U
#define TURN_MANEUVER_EXIT_PITCH_SAFETY    6U
#define TURN_MANEUVER_EXIT_EXTERNAL_CANCEL 7U
#define TURN_MANEUVER_STATE_IDLE           0U
#define TURN_MANEUVER_STATE_PREPARING      1U
#define TURN_MANEUVER_STATE_TURNING        2U

extern volatile uint8_t currentMode;
extern volatile uint8_t flagMotorsAreOn;
extern volatile uint8_t flagCalibrationIsReady;
extern volatile uint8_t flag_RC_active;
extern volatile int16_t axRaw;
extern volatile int16_t ayRaw;
extern volatile int16_t azRaw;
extern volatile uint16_t accelx;
extern volatile uint16_t accely;
extern volatile uint16_t accelz;
extern volatile float giro;
extern volatile float giro_z;
extern volatile uint32_t imu_last_update_tick;
extern volatile float imu_accel_forward_mps2;
extern volatile float imu_velocity_mps;
extern volatile float RC_setpoint;
extern volatile int16_t RC_steering;
extern volatile float FL_setpoint;
extern volatile int16_t FL_steering;
extern volatile float obstacle_follow_setpoint;
extern volatile int16_t obstacle_follow_steering;
extern volatile uint16_t FL_motion_phase_ms;
extern volatile uint16_t FL_balance_phase_ms;
extern volatile uint16_t forward_motion_balance_only_steering;

extern float angle_y;
extern float angle_yaw;
extern float Kp;
extern float Ki;
extern float Kd;
extern float integral_limit;
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
extern float yaw_steering_limit;
extern float turn_maneuver_forward_bias_deg;
extern uint16_t turn_maneuver_pre_bias_delay_ms;
extern volatile uint8_t turn_maneuver_active;
extern volatile uint8_t turn_maneuver_state;
extern volatile uint8_t turn_maneuver_mode;
extern volatile uint8_t turn_maneuver_wheel;
extern volatile float turn_maneuver_setpoint;
extern volatile int16_t turn_maneuver_steering;
extern volatile float turn_debug_target_deg;
extern volatile float turn_debug_turned_deg;
extern volatile float turn_debug_remaining_deg;
extern volatile float turn_debug_target_steering;
extern volatile float turn_debug_steering_ramp;
extern volatile float turn_debug_effective_limit;
extern volatile int16_t turn_debug_motor_left_cmd;
extern volatile int16_t turn_debug_motor_right_cmd;
extern volatile int16_t turn_debug_active_motor_cmd;
extern volatile int16_t turn_debug_pivot_motor_cmd;
extern volatile uint8_t turn_debug_steering_clamped;
extern volatile uint8_t turn_debug_motor_saturated;
extern volatile uint8_t turn_debug_exit_reason;
extern volatile uint16_t turn_debug_prepare_remaining_ms;

void PID_PITCH(void);
void PID_PITCH_ResetState(void);
void Control_SetMotorsEnabled(uint8_t enabled);
float ForwardMotion_Generate(float motion_setpoint,
                             int16_t steering,
                             uint32_t now,
                             uint32_t *phase_tick,
                             uint8_t *motion_phase);
void FollowLine_Task(void);
int16_t Calcular_PID_YAW(float error_linea);
uint8_t TurnManeuver_Start(float target_angle_deg, uint8_t wheel_mode, uint8_t wheel_select);
uint8_t TurnManeuver_StartArc(float target_angle_deg, uint8_t outer_wheel, uint8_t inner_wheel_percent);
void TurnManeuver_Cancel(void);
void TurnManeuver_CancelWithReason(uint8_t reason);
void TurnManeuver_Task(void);
void Robot_Drive(int16_t speed_L, int16_t speed_R);

#ifdef __cplusplus
}
#endif

#endif
