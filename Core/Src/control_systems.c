#include "control_systems.h"

void PID_PITCH(void)
{
    float gyro_rate = giro;

    accelx = axRaw;
    accely = ayRaw;
    accelz = azRaw;

    float abs_angle = (angle_y < 0.0f) ? -angle_y : angle_y;
    if (abs_angle > limite_inclinacion) {
        RC_setpoint = RC_setpoint * correccionRCSP;
    } else {
        if (RC_slow_setpoint < RC_setpoint) {
            RC_slow_setpoint += paso;
        }
        if (RC_slow_setpoint > RC_setpoint) {
            RC_slow_setpoint -= paso;
        }
    }

    error = angle_y - (setpoint + RC_slow_setpoint);

    switch (currentMode) {
    case CONTROL_MODE_RC:
        error = angle_y - (setpoint + RC_slow_setpoint);
        break;
    case CONTROL_MODE_IDLE:
    default:
        error = angle_y - setpoint;
        break;
    }

    integral += error * CONTROL_DT_PID;
    if (integral > 2000.0f) {
        integral = 2000.0f;
    } else if (integral < -2000.0f) {
        integral = -2000.0f;
    }

    float P_base = Kp * error;
    float abs_error = (error < 0.0f) ? -error : error;
    float P_agresivo = Kp_Agresivo * (error * abs_error);

    P = P_base + P_agresivo;
    I = Ki * integral;
    D = Kd * gyro_rate;
    output = P + I + D;
    showoutput = output;
    last_error = error;

    if (flagMotorsAreOn) {
        int16_t outputLeft = 0;
        int16_t outputRigth = 0;

        switch (currentMode) {
        case CONTROL_MODE_IDLE:
            outputLeft = (int16_t)output;
            outputRigth = (int16_t)output;
            break;
        case CONTROL_MODE_RC:
        default:
            outputLeft = (int16_t)output + RC_steering;
            outputRigth = (int16_t)output - RC_steering;
            break;
        }

        Robot_Drive(outputLeft, outputRigth);
    }

    if (flagMotorsAreOn == 0 || angle_y > 35.0f || angle_y < -35.0f) {
        Robot_Drive(0, 0);
    }
}

int16_t Calcular_PID_YAW(float error_linea)
{
    float P_yaw = Kp_yaw * error_linea;
    float D_yaw = Kd_yaw * (error_linea - last_error_yaw) / CONTROL_DT_PID;
    last_error_yaw = error_linea;

    float salida_yaw = P_yaw + D_yaw;

    if (salida_yaw > 400.0f) {
        salida_yaw = 400.0f;
    }
    if (salida_yaw < -400.0f) {
        salida_yaw = -400.0f;
    }

    return (int16_t)salida_yaw;
}
