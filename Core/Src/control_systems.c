#include "control_systems.h"
#include "line_sensors.h"

#define FL_SETPOINT_STEP             0.04f
#define FL_ALIGN_SETPOINT            0.2f
#define FL_ALIGN_ENTER_ERROR         0.35f
#define FL_ALIGN_HARD_ERROR          0.75f
#define FL_ALIGN_STEERING_LIMIT      320
#define FL_ANGLE_90_ERROR            0.85f
#define FL_ANGLE_90_TICKS            12U
#define FL_ANGLE_90_SETPOINT         0.0f
#define FL_ANGLE_90_STEERING         420
#define FL_RECOVERY_SETPOINT         0.5f
#define FL_RECOVERY_STEERING         260

static int16_t clamp_i16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static int16_t rate_limit_i16(int16_t current, int16_t target, int16_t max_step)
{
    int16_t delta = target - current;

    if (delta > max_step) {
        delta = max_step;
    } else if (delta < -max_step) {
        delta = -max_step;
    }

    return current + delta;
}

void PID_PITCH(void)
{
    static float FL_slow_setpoint = 0.0f;
    static uint16_t FL_lost_ticks = 0;
    static uint8_t fl_90_ticks = 0;
    static int16_t fl_steering_limited = 0;
    float gyro_rate = giro;
    float target_setpoint = setpoint;
    int16_t steering = 0;
    float abs_line_error = (error_linea < 0.0f) ? -error_linea : error_linea;
    uint8_t lateral_only = (uint8_t)((estado_sensores[0] || estado_sensores[2]) &&
                                     !estado_sensores[1]);

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

    switch (currentMode) {
    case CONTROL_MODE_RC:
        target_setpoint = setpoint + RC_slow_setpoint;
        steering = RC_steering;
        FL_slow_setpoint = 0.0f;
        FL_lost_ticks = 0;
        break;
    case CONTROL_MODE_FL_INICIO:
    case CONTROL_MODE_FL_BUSQUEDA_INICIAL:
    case CONTROL_MODE_FL_SIGUIENDO:
    case CONTROL_MODE_FL_RESCATE:
    case CONTROL_MODE_FL_INGRESO_A_90:
        if (AIRAB) {
            FL_lost_ticks = 0;
            if (currentMode == CONTROL_MODE_FL_INGRESO_A_90) {
                FL_slow_setpoint = FL_ANGLE_90_SETPOINT;
                steering = (last_state_linea >= 0.0f) ? FL_ANGLE_90_STEERING : -FL_ANGLE_90_STEERING;

                RC_setpoint = FL_ANGLE_90_SETPOINT;
                fl_steering_limited = steering;
                RC_steering = fl_steering_limited;

                if (abs_line_error < FL_ALIGN_ENTER_ERROR || estado_sensores[1]) {
                    fl_90_ticks = 0;
                    currentMode = CONTROL_MODE_FL_SIGUIENDO;
                }
            } else if (currentMode == CONTROL_MODE_FL_BUSQUEDA_INICIAL ||
                currentMode == CONTROL_MODE_FL_INICIO) {
                FL_slow_setpoint = FL_ALIGN_SETPOINT;

                if (lateral_only && abs_line_error > FL_ANGLE_90_ERROR) {
                    fl_90_ticks++;
                    FL_slow_setpoint = FL_ANGLE_90_SETPOINT;
                    steering = (error_linea > 0.0f) ? FL_ANGLE_90_STEERING : -FL_ANGLE_90_STEERING;

                    if (fl_90_ticks >= FL_ANGLE_90_TICKS) {
                        currentMode = CONTROL_MODE_FL_INGRESO_A_90;
                    }
                } else if (abs_line_error > FL_ALIGN_HARD_ERROR) {
                    fl_90_ticks = 0;
                    steering = (error_linea > 0.0f) ? FL_ALIGN_STEERING_LIMIT : -FL_ALIGN_STEERING_LIMIT;
                } else {
                    fl_90_ticks = 0;
                    steering = clamp_i16((int16_t)(Calcular_PID_YAW(error_linea) * 0.6f),
                                         -FL_ALIGN_STEERING_LIMIT,
                                         FL_ALIGN_STEERING_LIMIT);
                }

                if (abs_line_error < FL_ALIGN_ENTER_ERROR) {
                    currentMode = CONTROL_MODE_FL_SIGUIENDO;
                }
                RC_setpoint = FL_slow_setpoint;
                fl_steering_limited = rate_limit_i16(fl_steering_limited, steering, (int16_t)yaw_steering_step_max);
                RC_steering = fl_steering_limited;
                steering = RC_steering;
            } else {
                int16_t target_steering = Calcular_PID_YAW(error_linea);
                RC_steering = rate_limit_i16(fl_steering_limited, target_steering, (int16_t)yaw_steering_step_max);
                fl_steering_limited = RC_steering;
                int16_t abs_steering = (RC_steering < 0) ? -RC_steering : RC_steering;
                RC_setpoint = FL_setpoint - ((float)abs_steering * multiplicadorYaw);
                if (RC_setpoint < 0.2f) {
                    RC_setpoint = 0.2f;
                }
                RC_setpoint -= ((float)abs_steering * multiplicadorYaw);
                if (RC_setpoint < 0.2f) {
                    RC_setpoint = 0.2f;
                }
                steering = RC_steering;
                FL_slow_setpoint = RC_setpoint;
            }
        } else {
            FL_lost_ticks++;
            fl_90_ticks = 0;
            currentMode = CONTROL_MODE_FL_RESCATE;
            RC_setpoint = FL_RECOVERY_SETPOINT;
            RC_steering = (last_state_linea >= 0.0f) ? FL_RECOVERY_STEERING : -FL_RECOVERY_STEERING;
            fl_steering_limited = RC_steering;
            FL_slow_setpoint = RC_setpoint;
            steering = RC_steering;
        }
        target_setpoint = setpoint + RC_setpoint;
        break;
    case CONTROL_MODE_FL_PERDIDO_FAILSAFE:
        FL_slow_setpoint = 0.0f;
        target_setpoint = setpoint;
        steering = 0;
        break;
    case CONTROL_MODE_IDLE:
    default:
        RC_slow_setpoint = 0.0f;
        FL_slow_setpoint = 0.0f;
        FL_lost_ticks = 0;
        fl_90_ticks = 0;
        fl_steering_limited = 0;
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
        case CONTROL_MODE_FL_PERDIDO_FAILSAFE:
            outputLeft = (int16_t)output;
            outputRigth = (int16_t)output;
            break;
        case CONTROL_MODE_RC:
        case CONTROL_MODE_FL_INICIO:
        case CONTROL_MODE_FL_BUSQUEDA_INICIAL:
        case CONTROL_MODE_FL_SIGUIENDO:
        case CONTROL_MODE_FL_RESCATE:
        case CONTROL_MODE_FL_INGRESO_A_90:
        default:
            outputLeft = (int16_t)output + steering;
            outputRigth = (int16_t)output - steering;
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
    static float error_linea_filtrado = 0.0f;

    error_linea_filtrado = (yaw_error_filter_alpha * error_linea_filtrado) +
                           ((1.0f - yaw_error_filter_alpha) * error_linea);

    float P_yaw = Kp_yaw * error_linea_filtrado;
    float D_yaw = -Kd_yaw * giro_z;
    last_error_yaw = error_linea_filtrado;

    float salida_yaw = P_yaw + D_yaw;

    if (salida_yaw > 400.0f) {
        salida_yaw = 400.0f;
    }
    if (salida_yaw < -400.0f) {
        salida_yaw = -400.0f;
    }

    return (int16_t)salida_yaw;
}
