#include "control_systems.h"
#include "line_sensors.h"

#define FL_RECOVERY_STEERING         700
#define FL_STEERING_DEADBAND_ERROR   0.03f

void PID_PITCH(void)
{
    float gyro_rate = giro;
    float target_setpoint = setpoint;
    int16_t steering = 0;

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
        target_setpoint = setpoint + FL_forward_setpoint;
        steering = FL_steering;
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

    if (flagMotorsAreOn) {
        int16_t base_output = (int16_t)output;
        int16_t output_left = base_output;
        int16_t output_right = base_output;

        switch (currentMode) {
        case CONTROL_MODE_RC:
        case CONTROL_MODE_FL_INGRESO_A_90:
            output_left = base_output + steering;
            output_right = base_output - steering;
            break;

        case CONTROL_MODE_FL_BUSQUEDA_INICIAL:
        case CONTROL_MODE_FL_SIGUIENDO:
        case CONTROL_MODE_FL_RESCATE:
            if (steering > 0) {
                output_right = base_output - steering;
            } else if (steering < 0) {
                output_left = base_output + steering;
            }
            break;

        default:
            break;
        }

        Robot_Drive(output_left, output_right);
    }
    if (flagMotorsAreOn == 0 || angle_y > 65.0f || angle_y < -65.0f) {
        Robot_Drive(0, 0);
    }
}

void FollowLine_Task(void)
{
    static float fl_steering_slow = 0.0f;
    static int8_t last_line_dir = 1;
    float target_steering = 0.0f;

    if (currentMode < CONTROL_MODE_FL_INICIO ||
        currentMode > CONTROL_MODE_FL_INGRESO_A_90 ||
        currentMode == CONTROL_MODE_FL_INICIO ||
        flag_calibrando_linea) {
        FL_forward_setpoint = 0.0f;
        FL_steering = 0;
        fl_steering_slow = 0.0f;
        return;
    }

    if (AIRAB) {
        if (error_linea > 0.05f) {
            last_line_dir = 1;
        } else if (error_linea < -0.05f) {
            last_line_dir = -1;
        }

        if (error_linea > -FL_STEERING_DEADBAND_ERROR &&
            error_linea < FL_STEERING_DEADBAND_ERROR) {
            target_steering = 0.0f;
        } else {
            target_steering = (float)Calcular_PID_YAW(error_linea);
        }

        FL_forward_setpoint = FL_setpoint;
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

    if (AIRAB) {
        float abs_steering = (FL_steering < 0) ? -(float)FL_steering : (float)FL_steering;
        FL_forward_setpoint = FL_setpoint - (abs_steering * multiplicadorYaw);
        if (FL_forward_setpoint < 0.0f) {
            FL_forward_setpoint = 0.0f;
        }
    }
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
