#include "control_systems.h"
#include "line_sensors.h"

#define FL_RECOVERY_STEERING         700
#define FL_SETPOINT_DECAY_INTERVAL_MS 50U
#define FL_SETPOINT_DECAY_SCALAR      0.995f
#define FL_SETPOINT_RESET_THRESHOLD   0.01f

void PID_PITCH(void)
{
    float gyro_rate = giro;
    float target_setpoint = setpoint;
    int16_t steering = 0;
    static float variable_x = 1.0f;

    accelx = axRaw;
    accely = ayRaw;
    accelz = azRaw;

    if (currentMode == CONTROL_MODE_RC) {
        float abs_angle = (angle_y < 0.0f) ? -angle_y : angle_y;
        if (abs_angle > limite_inclinacion) {
            RC_setpoint = RC_setpoint * correccionRCSP;
        } else {
            if (RC_slow_setpoint < RC_setpoint) RC_slow_setpoint += paso;
            if (RC_slow_setpoint > RC_setpoint) RC_slow_setpoint -= paso;
        }
    }
    switch (currentMode) {
    case CONTROL_MODE_RC:
        target_setpoint = setpoint + RC_slow_setpoint;
        steering = RC_steering;
        break;
    case CONTROL_MODE_FL_INICIO:
    case CONTROL_MODE_FL_BUSQUEDA_INICIAL:
    case CONTROL_MODE_FL_SIGUIENDO:
        target_setpoint = setpoint + (FL_setpoint * variable_x);
        steering = FL_steering;
        variable_x -= correccionRCSP;
        if (variable_x < 0.0f) {
            variable_x = 0.0f;
        }
        break;

    case CONTROL_MODE_FL_RESCATE:
    case CONTROL_MODE_FL_INGRESO_A_90:
        target_setpoint = setpoint + FL_forward_setpoint;
        steering = FL_steering;
        break;
    case CONTROL_MODE_FL_PERDIDO_FAILSAFE:
        target_setpoint = setpoint;
        steering = 0;
        break;
    case CONTROL_MODE_IDLE:
    default:
        RC_slow_setpoint = 0.0f;
        variable_x = 1.0f;
        target_setpoint = setpoint;
        steering = 0;
        break;
    }
    error = angle_y - target_setpoint;
    integral += error * CONTROL_DT_PID;
    if (integral > 2000.0f) {
        integral = 2000.0f;
    } else if (integral < -2000.0f) {
        integral = -2000.0f;
    }
    P = Kp * error;
    I = Ki * integral;
    D = Kd * gyro_rate;
    output = P + I + D;
    showoutput = output;
    last_error = error;

    if(flagMotorsAreOn){      Robot_Drive((int16_t)output + steering, (int16_t)output - steering);    }
    if (flagMotorsAreOn == 0 || angle_y > 65.0f || angle_y < -65.0f) {
        Robot_Drive(0, 0);
    }
}

void FollowLine_Task(void)
{
    static float fl_steering_slow = 0.0f;
    static float fl_dynamic_setpoint = 0.0f;
    static uint32_t fl_decay_tick = 0;
    static int8_t last_line_dir = 1;
    float target_steering = 0.0f;
    uint32_t now = HAL_GetTick();

    if (currentMode < CONTROL_MODE_FL_INICIO ||
        currentMode > CONTROL_MODE_FL_INGRESO_A_90 ||
        currentMode == CONTROL_MODE_FL_INICIO ||
        flag_calibrando_linea) {
        FL_forward_setpoint = 0.0f;
        FL_steering = 0;
        fl_steering_slow = 0.0f;
        fl_dynamic_setpoint = FL_setpoint;
        fl_decay_tick = now;
        return;
    }

    if (fl_dynamic_setpoint == 0.0f) {
        fl_dynamic_setpoint = FL_setpoint;
        fl_decay_tick = now;
    }

    if (AIRAB) {
        if (error_linea > 0.05f) {
            last_line_dir = 1;
        } else if (error_linea < -0.05f) {
            last_line_dir = -1;
        }
        target_steering = (float)Calcular_PID_YAW(error_linea);
        if ((uint32_t)(now - fl_decay_tick) >= FL_SETPOINT_DECAY_INTERVAL_MS) {
            fl_decay_tick = now;
            fl_dynamic_setpoint *= FL_SETPOINT_DECAY_SCALAR;
            if (fl_dynamic_setpoint <= FL_SETPOINT_RESET_THRESHOLD) {
                fl_dynamic_setpoint = FL_setpoint;
            }
        }
        FL_forward_setpoint = fl_dynamic_setpoint;
    } else {
        target_steering = (float)(last_line_dir * FL_RECOVERY_STEERING);
        FL_forward_setpoint = 0.0f;
    }

    float delta_steering = target_steering - fl_steering_slow;

    if (delta_steering > yaw_steering_step_max) {
        delta_steering = yaw_steering_step_max;
    } else if (delta_steering < -yaw_steering_step_max) {
        delta_steering = -yaw_steering_step_max;
    }

    fl_steering_slow += delta_steering;
    FL_steering = (int16_t)fl_steering_slow;
}

int16_t Calcular_PID_YAW(float error_linea)
{
    static float error_linea_filtrado = 0.0f;

    error_linea_filtrado = (yaw_error_filter_alpha * error_linea_filtrado) +
                           ((1.0f - yaw_error_filter_alpha) * error_linea);

    float P_yaw = Kp_yaw * error_linea_filtrado;
    float D_yaw = -Kd_yaw * giro_z;
    last_error_yaw = error_linea_filtrado;

    return (int16_t)(P_yaw + D_yaw);
}
