#include "control_systems.h"
#include "line_sensors.h"
#include <math.h>

#define FL_RECOVERY_STEERING         700
#define TURN_MANEUVER_MIN_ANGLE_DEG  1.0f
#define TURN_MANEUVER_MAX_ANGLE_DEG  360.0f
#define TURN_MANEUVER_TOLERANCE_DEG  2.0f
#define TURN_MANEUVER_SLOW_ZONE_DEG  15.0f
#define TURN_MANEUVER_TIMEOUT_MS     6000U
#define TURN_MANEUVER_IMU_STALE_MS   100U
#define TURN_MANEUVER_SLOW_RATIO     0.35f
#define TURN_MANEUVER_SIGN_CHECK_MS  300U
#define TURN_MANEUVER_SIGN_CHECK_DEG 0.7f
#define TURN_MANEUVER_YAW_BOOST      1000.0f
#define ARC_MANEUVER_FORWARD_RATIO   0.50f
#define TURN_MANEUVER_PREP_MIN_MS    180U
#define TURN_MANEUVER_PREP_MAX_MS    650U
#define TURN_MANEUVER_PREP_BIAS      90.0f
#define TURN_MANEUVER_PREP_OUTPUT_OK 180.0f
#define TURN_MANEUVER_PREP_VEL_OK    0.05f

volatile uint8_t turn_maneuver_active = 0;
volatile uint8_t turn_maneuver_mode = TURN_MANEUVER_MODE_TWO_WHEELS;
volatile uint8_t turn_maneuver_wheel = TURN_MANEUVER_WHEEL_LEFT;
volatile float turn_maneuver_setpoint = 0.0f;
volatile int16_t turn_maneuver_steering = 0;

static float turn_maneuver_start_yaw_deg = 0.0f;
static float turn_maneuver_target_delta_deg = 0.0f;
static float turn_maneuver_error_filtered = 0.0f;
static float turn_maneuver_steering_slow = 0.0f;
static float turn_maneuver_arc_inner_ratio = 0.0f;
static uint32_t turn_maneuver_start_tick = 0;
static uint32_t turn_maneuver_prepare_tick = 0;
static int8_t turn_maneuver_drive_sign = 1;
static int8_t turn_maneuver_wheel_drive_sign = 1;
static uint8_t turn_maneuver_prepare_active = 0;
static uint8_t turn_maneuver_sign_checked = 0;

static float clamp_float(float value, float limit)
{
    if (limit <= 0.0f) {
        return value;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void PID_PITCH_ResetState(void)
{
    integral = 0.0f;
    last_error = 0.0f;
    P = 0.0f;
    I = 0.0f;
    D = 0.0f;
    output = 0.0f;
    showoutput = 0.0f;
    error = 0.0f;
}

void Control_SetMotorsEnabled(uint8_t enabled)
{
    enabled = enabled ? 1U : 0U;

    if (flagMotorsAreOn != enabled) {
        PID_PITCH_ResetState();
    }

    flagMotorsAreOn = enabled;

    if (!enabled) {
        TurnManeuver_Cancel();
        RC_setpoint = 0.0f;
        RC_steering = 0;
        Robot_Drive(0, 0);
    }
}

void PID_PITCH(void)
{
    float gyro_rate = giro;
    float target_setpoint = setpoint;
    int16_t steering = 0;
    static uint32_t rc_phase_tick = 0;
    static uint8_t rc_motion_phase = 1;
    static uint32_t fl_phase_tick = 0;
    static uint8_t fl_motion_phase = 1;
    uint32_t now = HAL_GetTick();

    accelx = axRaw;
    accely = ayRaw;
    accelz = azRaw;

    if (!flagMotorsAreOn) {
        PID_PITCH_ResetState();
        Robot_Drive(0, 0);
        return;
    }

    if (angle_y > 65.0f || angle_y < -65.0f) {
        Control_SetMotorsEnabled(0U);
        return;
    }

    switch (currentMode) {
    case CONTROL_MODE_RC:
        steering = RC_steering;
        if (flag_RC_active && RC_setpoint != 0.0f) {
            target_setpoint = setpoint + ForwardMotion_Generate(FL_setpoint * RC_setpoint,
                                                                steering,
                                                                now,
                                                                &rc_phase_tick,
                                                                &rc_motion_phase);
        } else {
            rc_phase_tick = 0U;
            rc_motion_phase = 1U;
            target_setpoint = setpoint;
        }
        break;

    case CONTROL_MODE_OBSTACLE_FOLLOW:
        rc_phase_tick = 0U;
        rc_motion_phase = 1U;
        target_setpoint = setpoint + obstacle_follow_setpoint;
        steering = obstacle_follow_steering;
        break;

    case CONTROL_MODE_FL_INICIO:
    case CONTROL_MODE_FL_BUSQUEDA_INICIAL:
    case CONTROL_MODE_FL_SIGUIENDO:
        rc_phase_tick = 0U;
        rc_motion_phase = 1U;
        steering = FL_steering;
        target_setpoint = setpoint + ForwardMotion_Generate(FL_setpoint, steering, now, &fl_phase_tick, &fl_motion_phase);
        break;

    case CONTROL_MODE_FL_RESCATE:
    case CONTROL_MODE_FL_INGRESO_A_90:
        rc_phase_tick = 0U;
        rc_motion_phase = 1U;
        target_setpoint = setpoint;
        steering = FL_steering;
        break;
    case CONTROL_MODE_FL_PERDIDO_FAILSAFE:
        rc_phase_tick = 0U;
        rc_motion_phase = 1U;
        target_setpoint = setpoint;
        steering = 0;
        break;
    case CONTROL_MODE_IDLE:
    default:
        rc_phase_tick = 0U;
        rc_motion_phase = 1U;
        fl_phase_tick = 0U;
        fl_motion_phase = 1U;
        target_setpoint = setpoint;
        steering = 0;
        break;
    }

    if (turn_maneuver_active) {
        target_setpoint = setpoint + turn_maneuver_setpoint;
        steering = turn_maneuver_steering;
    }

    error = angle_y - target_setpoint;
    integral += error * CONTROL_DT_PID;
    if (integral > integral_limit) {
        integral = integral_limit;
    } else if (integral < -integral_limit) {
        integral = -integral_limit;
    }
    P = Kp * error;
    I = Ki * integral;
    D = Kd * gyro_rate;
    output = P + I + D;
    showoutput = output;
    last_error = error;

    if (turn_maneuver_active && turn_maneuver_mode == TURN_MANEUVER_MODE_ONE_WHEEL) {
        int16_t pitch_output = (int16_t)output;

        if (turn_maneuver_wheel == TURN_MANEUVER_WHEEL_LEFT) {
            Robot_Drive(pitch_output + steering, pitch_output);
        } else {
            Robot_Drive(pitch_output, pitch_output - steering);
        }
    } else if (turn_maneuver_active && turn_maneuver_mode == TURN_MANEUVER_MODE_ARC) {
        int16_t pitch_output = (int16_t)output;
        int16_t inner_steering = (int16_t)((float)steering * turn_maneuver_arc_inner_ratio);

        if (turn_maneuver_wheel == TURN_MANEUVER_WHEEL_LEFT) {
            Robot_Drive(pitch_output + steering, pitch_output - inner_steering);
        } else {
            Robot_Drive(pitch_output + inner_steering, pitch_output - steering);
        }
    } else {
        Robot_Drive((int16_t)output + steering, (int16_t)output - steering);
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
        currentMode == CONTROL_MODE_OBSTACLE_FOLLOW ||
        flag_calibrando_linea) {
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
        target_steering = (float)Calcular_PID_YAW(error_linea);
    } else {
        target_steering = (float)(last_line_dir * FL_RECOVERY_STEERING);
    }

    float delta_steering = target_steering - fl_steering_slow;

    if (delta_steering > yaw_steering_step_max) {
        delta_steering = yaw_steering_step_max;
    } else if (delta_steering < -yaw_steering_step_max) {
        delta_steering = -yaw_steering_step_max;
    }

    fl_steering_slow += delta_steering;
    if (yaw_steering_limit > 0.0f) {
        if (fl_steering_slow > yaw_steering_limit) {
            fl_steering_slow = yaw_steering_limit;
        } else if (fl_steering_slow < -yaw_steering_limit) {
            fl_steering_slow = -yaw_steering_limit;
        }
    }
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

static uint8_t TurnManeuver_StartInternal(float target_angle_deg,
                                          uint8_t wheel_mode,
                                          uint8_t wheel_select,
                                          uint8_t inner_wheel_percent)
{
    float abs_angle = fabsf(target_angle_deg);
    uint32_t now = HAL_GetTick();

    if (abs_angle < TURN_MANEUVER_MIN_ANGLE_DEG || abs_angle > TURN_MANEUVER_MAX_ANGLE_DEG) {
        return TURN_MANEUVER_STATUS_RANGE;
    }

    if (wheel_mode > TURN_MANEUVER_MODE_ARC ||
        wheel_select > TURN_MANEUVER_WHEEL_RIGHT) {
        return TURN_MANEUVER_STATUS_RANGE;
    }

    if (wheel_mode == TURN_MANEUVER_MODE_ARC && inner_wheel_percent > 100U) {
        return TURN_MANEUVER_STATUS_RANGE;
    }

    if (!flagMotorsAreOn ||
        (wheel_mode != TURN_MANEUVER_MODE_ARC &&
         currentMode != CONTROL_MODE_RC &&
         currentMode != CONTROL_MODE_OBSTACLE_FOLLOW) ||
        (wheel_mode == TURN_MANEUVER_MODE_ARC && currentMode == CONTROL_MODE_IDLE)) {
        return TURN_MANEUVER_STATUS_MODE;
    }

    if (imu_last_update_tick == 0U ||
        (uint32_t)(now - imu_last_update_tick) > TURN_MANEUVER_IMU_STALE_MS) {
        return TURN_MANEUVER_STATUS_SENSOR;
    }

    turn_maneuver_start_yaw_deg = angle_yaw;
    turn_maneuver_target_delta_deg = target_angle_deg;
    turn_maneuver_error_filtered = 0.0f;
    turn_maneuver_steering_slow = 0.0f;
    turn_maneuver_start_tick = now;
    turn_maneuver_prepare_tick = now;
    turn_maneuver_drive_sign = 1;
    turn_maneuver_wheel_drive_sign = (target_angle_deg >= 0.0f) ? 1 : -1;
    if (wheel_mode == TURN_MANEUVER_MODE_ONE_WHEEL && wheel_select == TURN_MANEUVER_WHEEL_RIGHT) {
        turn_maneuver_wheel_drive_sign = -turn_maneuver_wheel_drive_sign;
    }
    turn_maneuver_prepare_active = (wheel_mode == TURN_MANEUVER_MODE_ONE_WHEEL) ? 1U : 0U;
    turn_maneuver_sign_checked = 0;
    turn_maneuver_mode = wheel_mode;
    turn_maneuver_wheel = wheel_select;
    turn_maneuver_arc_inner_ratio = ((float)inner_wheel_percent) / 100.0f;
    turn_maneuver_setpoint = (wheel_mode == TURN_MANEUVER_MODE_ARC) ? (FL_setpoint * ARC_MANEUVER_FORWARD_RATIO) : 0.0f;
    turn_maneuver_steering = turn_maneuver_prepare_active ?
                              (int16_t)(TURN_MANEUVER_PREP_BIAS * (float)turn_maneuver_wheel_drive_sign) :
                              0;
    turn_maneuver_active = 1;
    flag_RC_active = 0;
    RC_setpoint = 0.0f;
    RC_steering = 0;

    return TURN_MANEUVER_STATUS_OK;
}

uint8_t TurnManeuver_Start(float target_angle_deg, uint8_t wheel_mode, uint8_t wheel_select)
{
    return TurnManeuver_StartInternal(target_angle_deg, wheel_mode, wheel_select, 0U);
}

uint8_t TurnManeuver_StartArc(float target_angle_deg, uint8_t outer_wheel, uint8_t inner_wheel_percent)
{
    return TurnManeuver_StartInternal(target_angle_deg,
                                      TURN_MANEUVER_MODE_ARC,
                                      outer_wheel,
                                      inner_wheel_percent);
}

void TurnManeuver_Cancel(void)
{
    turn_maneuver_active = 0;
    turn_maneuver_setpoint = 0.0f;
    turn_maneuver_steering = 0;
    turn_maneuver_steering_slow = 0.0f;
    turn_maneuver_error_filtered = 0.0f;
    turn_maneuver_arc_inner_ratio = 0.0f;
    turn_maneuver_drive_sign = 1;
    turn_maneuver_wheel_drive_sign = 1;
    turn_maneuver_prepare_active = 0;
    turn_maneuver_sign_checked = 0;
    RC_setpoint = 0.0f;
    RC_steering = 0;
}

void TurnManeuver_Task(void)
{
    if (!turn_maneuver_active) return;

    uint32_t now = HAL_GetTick();
    if (!flagMotorsAreOn ||
        (turn_maneuver_mode != TURN_MANEUVER_MODE_ARC &&
         currentMode != CONTROL_MODE_RC &&
         currentMode != CONTROL_MODE_OBSTACLE_FOLLOW) ||
        (turn_maneuver_mode == TURN_MANEUVER_MODE_ARC && currentMode == CONTROL_MODE_IDLE)) {
        TurnManeuver_Cancel();
        return;
    }
    if (imu_last_update_tick == 0U ||
        (uint32_t)(now - imu_last_update_tick) > TURN_MANEUVER_IMU_STALE_MS) {
        Control_SetMotorsEnabled(0U);
        return;
    }
    if ((uint32_t)(now - turn_maneuver_start_tick) > TURN_MANEUVER_TIMEOUT_MS) {
        TurnManeuver_Cancel();
        return;
    }

    if (turn_maneuver_prepare_active) {
        uint32_t prep_elapsed = (uint32_t)(now - turn_maneuver_prepare_tick);
        float balance_effort = output;
        uint8_t balance_ok = (fabsf(balance_effort) <= TURN_MANEUVER_PREP_OUTPUT_OK ||
                              (balance_effort * (float)turn_maneuver_wheel_drive_sign) >= 0.0f) ? 1U : 0U;
        uint8_t velocity_ok = (fabsf(imu_velocity_mps) <= TURN_MANEUVER_PREP_VEL_OK ||
                               (imu_velocity_mps * (float)turn_maneuver_wheel_drive_sign) >= 0.0f) ? 1U : 0U;

        turn_maneuver_setpoint = 0.0f;
        turn_maneuver_steering = (int16_t)(TURN_MANEUVER_PREP_BIAS *
                                           (float)turn_maneuver_wheel_drive_sign);

        if ((prep_elapsed >= TURN_MANEUVER_PREP_MIN_MS && balance_ok && velocity_ok) ||
            prep_elapsed >= TURN_MANEUVER_PREP_MAX_MS) {
            turn_maneuver_prepare_active = 0U;
            turn_maneuver_start_yaw_deg = angle_yaw;
            turn_maneuver_start_tick = now;
            turn_maneuver_error_filtered = 0.0f;
            turn_maneuver_steering_slow = turn_maneuver_steering;
            turn_maneuver_sign_checked = 0U;
        } else {
            return;
        }
    }

    float turned_deg = angle_yaw - turn_maneuver_start_yaw_deg;
    float remaining_deg = turn_maneuver_target_delta_deg - turned_deg;

    if (!turn_maneuver_sign_checked &&
        (uint32_t)(now - turn_maneuver_start_tick) >= TURN_MANEUVER_SIGN_CHECK_MS &&
        fabsf(turned_deg) >= TURN_MANEUVER_SIGN_CHECK_DEG) {
        if ((turned_deg * turn_maneuver_target_delta_deg) < 0.0f) {
            turn_maneuver_drive_sign = -turn_maneuver_drive_sign;
            turn_maneuver_error_filtered = 0.0f;
            turn_maneuver_steering_slow = 0.0f;
        }
        turn_maneuver_sign_checked = 1;
    }

    if (fabsf(remaining_deg) <= TURN_MANEUVER_TOLERANCE_DEG) {
        TurnManeuver_Cancel();
        return;
    }

    float yaw_limit = yaw_steering_limit;
    float forward_ratio = 1.0f;
    if (fabsf(remaining_deg) <= TURN_MANEUVER_SLOW_ZONE_DEG) {
        yaw_limit *= TURN_MANEUVER_SLOW_RATIO;
        forward_ratio = TURN_MANEUVER_SLOW_RATIO;
    }

    float yaw_error = remaining_deg * multiplicadorYaw;
    turn_maneuver_error_filtered =
        (yaw_error_filter_alpha * turn_maneuver_error_filtered) +
        ((1.0f - yaw_error_filter_alpha) * yaw_error);

    float effective_Kp = Kp_yaw + 1500.0f;
    float target_steering = turn_maneuver_drive_sign *
                            (((effective_Kp * turn_maneuver_error_filtered) * TURN_MANEUVER_YAW_BOOST) -
                             (Kd_yaw * giro_z));
    target_steering = clamp_float(target_steering, yaw_limit);

    float delta_steering = target_steering - turn_maneuver_steering_slow;
    delta_steering = clamp_float(delta_steering, yaw_steering_step_max);
    turn_maneuver_steering_slow += delta_steering;
    turn_maneuver_steering_slow = clamp_float(turn_maneuver_steering_slow, yaw_limit);

    turn_maneuver_steering = (int16_t)turn_maneuver_steering_slow;
    turn_maneuver_setpoint = (turn_maneuver_mode == TURN_MANEUVER_MODE_ARC) ?
                              (FL_setpoint * ARC_MANEUVER_FORWARD_RATIO * forward_ratio) :
                              0.0f;
}
