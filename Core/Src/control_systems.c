#include "control_systems.h"
#include "line_sensors.h"
#include "obstacle_follow.h"
#include <math.h>

#define FL_RECOVERY_STEERING         700
#define FL_CIRCLE_FRONT_ADC_INDEX    5U
#define FL_CIRCLE_REAR_ADC_INDEX     4U
#define FL_CIRCLE_FRONT_RIGHT_ADC_INDEX 6U
#define FL_CIRCLE_FRONT_THRESHOLD    2500U	//IR FRONTAL
#define FL_CIRCLE_SIDE_THRESHOLD     2900U
#define FL_CIRCLE_CONFIRM_TICKS      5U
#define FL_CIRCLE_ALIGN_STEERING     -450
#define ACCEL_RUNAWAY_WINDOW_CYCLES  10U
#define ACCEL_RUNAWAY_SAFE_CENTER    550.0f
#define TURN_MANEUVER_MIN_ANGLE_DEG  1.0f
#define TURN_MANEUVER_MAX_ANGLE_DEG  360.0f
#define TURN_MANEUVER_TOLERANCE_DEG  2.0f
#define TURN_MANEUVER_TIMEOUT_MS     6000U
#define TURN_MANEUVER_IMU_STALE_MS   100U
#define TURN_MANEUVER_YAW_BOOST      1000.0f
#define TURN_MANEUVER_SETTLE_MS      300U
#define ARC_MANEUVER_FORWARD_RATIO   0.50f

volatile uint8_t turn_maneuver_active = 0;
volatile uint8_t turn_maneuver_state = TURN_MANEUVER_STATE_IDLE;
volatile uint8_t turn_maneuver_mode = TURN_MANEUVER_MODE_TWO_WHEELS;
volatile uint8_t turn_maneuver_wheel = TURN_MANEUVER_WHEEL_LEFT;
volatile float turn_maneuver_setpoint = 0.0f;
volatile int16_t turn_maneuver_steering = 0;
volatile float turn_debug_target_deg = 0.0f;
volatile float turn_debug_turned_deg = 0.0f;
volatile float turn_debug_remaining_deg = 0.0f;
volatile float turn_debug_target_steering = 0.0f;
volatile float turn_debug_steering_ramp = 0.0f;
volatile float turn_debug_effective_limit = 0.0f;
volatile int16_t turn_debug_motor_left_cmd = 0;
volatile int16_t turn_debug_motor_right_cmd = 0;
volatile int16_t turn_debug_active_motor_cmd = 0;
volatile int16_t turn_debug_pivot_motor_cmd = 0;
volatile uint8_t turn_debug_steering_clamped = 0;
volatile uint8_t turn_debug_motor_saturated = 0;
volatile uint8_t turn_debug_exit_reason = TURN_MANEUVER_EXIT_NONE;
volatile uint16_t turn_debug_prepare_remaining_ms = 0;

static float turn_maneuver_start_yaw_deg = 0.0f;
static float turn_maneuver_target_delta_deg = 0.0f;
static float turn_maneuver_error_filtered = 0.0f;
static float turn_maneuver_steering_slow = 0.0f;
static float turn_maneuver_arc_inner_ratio = 0.0f;
static float turn_maneuver_settle_setpoint = 0.0f;
static float turn_maneuver_settle_steering = 0.0f;
static uint32_t turn_maneuver_start_tick = 0;
static uint32_t turn_maneuver_prepare_start_tick = 0;
static uint32_t turn_maneuver_settle_start_tick = 0;
static uint8_t corner_turn_mode = TURN_MANEUVER_MODE_ONE_WHEEL;
static uint8_t corner_turn_wheel = TURN_MANEUVER_WHEEL_RIGHT;
static uint8_t corner_turn_inner_wheel_percent = 0U;
static int8_t corner_turn_direction = 1;
static float corner_turn_bias_deg = 5.0f;
static uint16_t corner_turn_pre_bias_delay_ms = 300U;
volatile uint8_t accel_runaway_enabled = 0U;
float accel_runaway_delta_threshold = 120.0f;
float accel_runaway_abs_threshold = 450.0f;
float accel_runaway_trend_limit = 4.0f;
float accel_runaway_abs_limit = 4.0f;
float accel_adaptive_offset_step_deg = 0.10f;
float accel_adaptive_offset_limit_deg = 10.0f;
float accel_adaptive_reset_abs_threshold = 120.0f;
float accel_adaptive_reset_count_limit = 5.0f;
volatile float accel_runaway_mean = 0.0f;
volatile float accel_runaway_delta = 0.0f;
volatile float accel_adaptive_equilibrium_offset_deg = 0.0f;
volatile uint8_t accel_runaway_trend_counter = 0U;
volatile uint8_t accel_runaway_abs_counter = 0U;
volatile uint8_t accel_runaway_brake_reason = 0U;
volatile uint8_t accel_adaptive_reset_counter = 0U;
static int32_t accel_runaway_sum = 0;
static uint8_t accel_runaway_cycle_count = 0U;
static float accel_runaway_prev_mean = 0.0f;
static int8_t accel_runaway_last_delta_sign = 0;
static uint8_t accel_runaway_has_prev_mean = 0U;
static float fl_steering_slow = 0.0f;
static int8_t fl_last_line_dir = 1;
static uint8_t fl_circle_front_confirm_count = 0U;
static uint8_t fl_circle_side_confirm_count = 0U;
static float fl_error_linea_filtrado = 0.0f;

static void FollowLine_ResetStateInternal(void)
{
    FL_steering = 0;
    fl_steering_slow = 0.0f;
    fl_last_line_dir = 1;
    fl_circle_front_confirm_count = 0U;
    fl_circle_side_confirm_count = 0U;
    fl_error_linea_filtrado = 0.0f;
    last_error_yaw = 0.0f;
}

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

static float TurnManeuver_GetActiveSetpoint(void)
{
    float maneuver_setpoint = turn_maneuver_forward_bias_deg;

    if (turn_maneuver_mode == TURN_MANEUVER_MODE_ARC) {
        maneuver_setpoint += FL_setpoint * ARC_MANEUVER_FORWARD_RATIO;
    }

    return maneuver_setpoint;
}

static int16_t sign_preserving_limit(float value, float limit)
{
    if (limit < 0.0f) {
        limit = -limit;
    }
    return (int16_t)clamp_float(value, limit);
}

static void TurnManeuver_ApplyOneWheelMix(int16_t pitch_output, int16_t steering)
{
    float active_yaw = (turn_maneuver_wheel == TURN_MANEUVER_WHEEL_LEFT) ?
                       (float)(-steering) :
                       (float)steering;
    int16_t active_cmd = sign_preserving_limit((float)pitch_output + active_yaw, 3599.0f);
    int16_t pivot_cmd = sign_preserving_limit((float)pitch_output, 3599.0f);

    if (turn_maneuver_wheel == TURN_MANEUVER_WHEEL_LEFT) {
        Robot_Drive(active_cmd, pivot_cmd);
    } else {
        Robot_Drive(pivot_cmd, active_cmd);
    }
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
    accel_runaway_sum = 0;
    accel_runaway_cycle_count = 0U;
    accel_runaway_prev_mean = 0.0f;
    accel_runaway_has_prev_mean = 0U;
    accel_runaway_delta = 0.0f;
    accel_adaptive_equilibrium_offset_deg = 0.0f;
    accel_runaway_trend_counter = 0U;
    accel_runaway_abs_counter = 0U;
    accel_adaptive_reset_counter = 0U;
    accel_runaway_last_delta_sign = 0;
    accel_runaway_brake_reason = 0U;
    turn_debug_motor_left_cmd = 0;
    turn_debug_motor_right_cmd = 0;
    turn_debug_active_motor_cmd = 0;
    turn_debug_pivot_motor_cmd = 0;
    turn_debug_motor_saturated = 0;
}

void Control_SetMotorsEnabled(uint8_t enabled)
{
    enabled = enabled ? 1U : 0U;

    if (flagMotorsAreOn != enabled) {
        PID_PITCH_ResetState();
    }

    flagMotorsAreOn = enabled;

    if (!enabled) {
        if (turn_maneuver_active) {
            TurnManeuver_CancelWithReason(TURN_MANEUVER_EXIT_MOTORS_OFF);
        }
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
    uint8_t accel_runaway_trigger = 0U;
    int8_t accel_adaptive_direction = 0;

    accelx = axRaw;
    accely = ayRaw;
    accelz = azRaw;

    if (!flagMotorsAreOn) {
        PID_PITCH_ResetState();
        Robot_Drive(0, 0);
        return;
    }

    if (angle_y > 65.0f || angle_y < -65.0f) {
        if (turn_maneuver_active) {
            TurnManeuver_CancelWithReason(TURN_MANEUVER_EXIT_PITCH_SAFETY);
        }
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
    case CONTROL_MODE_FL_CIRCLE_ALIGN:
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
        if (turn_maneuver_state != TURN_MANEUVER_STATE_SETTLING) {
            turn_maneuver_setpoint = TurnManeuver_GetActiveSetpoint();
        }
        target_setpoint = setpoint + turn_maneuver_setpoint;
        steering = turn_maneuver_steering;
    }

    if (!accel_runaway_enabled) {
        accel_runaway_sum = 0;
        accel_runaway_cycle_count = 0U;
        accel_runaway_prev_mean = 0.0f;
        accel_runaway_has_prev_mean = 0U;
        accel_runaway_delta = 0.0f;
        accel_runaway_trend_counter = 0U;
        accel_runaway_abs_counter = 0U;
        accel_adaptive_reset_counter = 0U;
        accel_runaway_last_delta_sign = 0;
        accel_runaway_brake_reason = 0U;
        accel_adaptive_equilibrium_offset_deg = 0.0f;
    } else {
        accel_runaway_sum += axRaw;
        accel_runaway_cycle_count++;
    }

    if (accel_runaway_enabled && accel_runaway_cycle_count >= ACCEL_RUNAWAY_WINDOW_CYCLES) {
        float current_mean = (float)accel_runaway_sum / (float)ACCEL_RUNAWAY_WINDOW_CYCLES;
        float delta = accel_runaway_has_prev_mean ? (current_mean - accel_runaway_prev_mean) : 0.0f;
        float abs_delta = fabsf(delta);
        float abs_offset = fabsf(current_mean - ACCEL_RUNAWAY_SAFE_CENTER);
        int8_t delta_sign = (delta > 0.0f) ? 1 : ((delta < 0.0f) ? -1 : 0);
        int8_t mean_sign = (current_mean > ACCEL_RUNAWAY_SAFE_CENTER) ? 1 : -1;
        uint8_t trend_limit = (uint8_t)accel_runaway_trend_limit;
        uint8_t abs_limit = (uint8_t)accel_runaway_abs_limit;
        uint8_t reset_limit = (uint8_t)accel_adaptive_reset_count_limit;

        accel_runaway_mean = current_mean;
        accel_runaway_delta = delta;
        accel_runaway_sum = 0;
        accel_runaway_cycle_count = 0U;

        if (accel_runaway_has_prev_mean &&
            accel_runaway_delta_threshold > 0.0f &&
            abs_delta >= accel_runaway_delta_threshold &&
            delta_sign != 0 &&
            (accel_runaway_last_delta_sign == 0 || delta_sign == accel_runaway_last_delta_sign)) {
            if (accel_runaway_trend_counter < 255U) {
                accel_runaway_trend_counter++;
            }
        } else if (accel_runaway_trend_counter > 0U) {
            accel_runaway_trend_counter--;
        }

        if (delta_sign != 0) {
            accel_runaway_last_delta_sign = delta_sign;
        }

        if (accel_runaway_abs_threshold > 0.0f && abs_offset >= accel_runaway_abs_threshold) {
            if (accel_runaway_abs_counter < 255U) {
                accel_runaway_abs_counter++;
            }
        } else if (accel_runaway_abs_counter > 0U) {
            accel_runaway_abs_counter--;
        }

        if (trend_limit > 0U && accel_runaway_trend_counter >= trend_limit) {
            accel_runaway_trigger = 1U;
            accel_adaptive_direction = (delta_sign != 0) ? delta_sign : mean_sign;
            accel_runaway_brake_reason = 1U;
        } else if (abs_limit > 0U && accel_runaway_abs_counter >= abs_limit) {
            accel_runaway_trigger = 1U;
            accel_adaptive_direction = mean_sign;
            accel_runaway_brake_reason = 2U;
        }

        if (!accel_runaway_trigger &&
            fabsf(accel_runaway_delta) < accel_runaway_delta_threshold &&
            abs_offset < accel_adaptive_reset_abs_threshold) {
            if (accel_adaptive_reset_counter < 255U) {
                accel_adaptive_reset_counter++;
            }
        } else {
            accel_adaptive_reset_counter = 0U;
        }

        if (reset_limit > 0U && accel_adaptive_reset_counter >= reset_limit) {
            accel_adaptive_equilibrium_offset_deg = 0.0f;
            accel_adaptive_reset_counter = 0U;
        }

        accel_runaway_prev_mean = current_mean;
        accel_runaway_has_prev_mean = 1U;
    }

    if (accel_runaway_trigger && accel_adaptive_direction != 0) {
        float limit = fabsf(accel_adaptive_offset_limit_deg);
        if (limit > ACCEL_ADAPTIVE_OFFSET_MAX_DEG) {
            limit = ACCEL_ADAPTIVE_OFFSET_MAX_DEG;
        }
        if (limit > 0.0f) {
            accel_adaptive_equilibrium_offset_deg +=
                ((float)accel_adaptive_direction * accel_adaptive_offset_step_deg);
            accel_adaptive_equilibrium_offset_deg =
                clamp_float(accel_adaptive_equilibrium_offset_deg, limit);
        } else {
            accel_adaptive_equilibrium_offset_deg = 0.0f;
        }
    }

    target_setpoint -= accel_adaptive_equilibrium_offset_deg;

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
        TurnManeuver_ApplyOneWheelMix(pitch_output, steering);
    } else if (turn_maneuver_active && turn_maneuver_mode == TURN_MANEUVER_MODE_ARC) {
        int16_t pitch_output = (int16_t)output;
        int16_t inner_steering = (int16_t)((float)steering * turn_maneuver_arc_inner_ratio);

        if (turn_maneuver_wheel == TURN_MANEUVER_WHEEL_LEFT) {
            Robot_Drive(pitch_output - steering, pitch_output + inner_steering);
        } else {
            Robot_Drive(pitch_output - inner_steering, pitch_output + steering);
        }
    } else {
        Robot_Drive((int16_t)output - steering, (int16_t)output + steering);
    }
}

void FollowLine_Task(void)
{
    float target_steering = 0.0f;

    if (currentMode == CONTROL_MODE_FL_CIRCLE_ALIGN) {
        fl_circle_front_confirm_count = 0U;
        fl_steering_slow = (float)FL_CIRCLE_ALIGN_STEERING;
        FL_steering = FL_CIRCLE_ALIGN_STEERING;

        if (adc_buffer[FL_CIRCLE_FRONT_RIGHT_ADC_INDEX] <= FL_CIRCLE_SIDE_THRESHOLD &&
            adc_buffer[FL_CIRCLE_REAR_ADC_INDEX] <= FL_CIRCLE_SIDE_THRESHOLD) {
            if (fl_circle_side_confirm_count < FL_CIRCLE_CONFIRM_TICKS) {
                fl_circle_side_confirm_count++;
            }
        } else {
            fl_circle_side_confirm_count = 0U;
        }

        if (fl_obstacle_avoidance_enabled &&
            fl_circle_side_confirm_count >= FL_CIRCLE_CONFIRM_TICKS) {
            FollowLine_ResetStateInternal();
            (void)ObstacleFollow_Start(OBSTACLE_FOLLOW_SIDE_RIGHT);
        }
        return;
    }

    if (currentMode < CONTROL_MODE_FL_INICIO ||
        currentMode > CONTROL_MODE_FL_CIRCLE_ALIGN ||
        currentMode == CONTROL_MODE_FL_INICIO ||
        currentMode == CONTROL_MODE_OBSTACLE_FOLLOW ||
        flag_calibrando_linea) {
        FollowLine_ResetStateInternal();
        return;
    }

    if (adc_buffer[FL_CIRCLE_FRONT_ADC_INDEX] < FL_CIRCLE_FRONT_THRESHOLD) {
        if (fl_circle_front_confirm_count < FL_CIRCLE_CONFIRM_TICKS) {
            fl_circle_front_confirm_count++;
        }
    } else {
        fl_circle_front_confirm_count = 0U;
    }

    if (fl_obstacle_avoidance_enabled &&
        fl_circle_front_confirm_count >= FL_CIRCLE_CONFIRM_TICKS) {
        FollowLine_ResetStateInternal();
        fl_steering_slow = (float)FL_CIRCLE_ALIGN_STEERING;
        FL_steering = FL_CIRCLE_ALIGN_STEERING;
        currentMode = CONTROL_MODE_FL_CIRCLE_ALIGN;
        return;
    }

    if (AIRAB) {
        if (error_linea > 0.05f) {
            fl_last_line_dir = 1;
        } else if (error_linea < -0.05f) {
            fl_last_line_dir = -1;
        }
        target_steering = (float)Calcular_PID_YAW(error_linea);
    } else {
        target_steering = (float)(fl_last_line_dir * FL_RECOVERY_STEERING);
    }
    target_steering = -target_steering;

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
    fl_error_linea_filtrado = (yaw_error_filter_alpha * fl_error_linea_filtrado) +
                           ((1.0f - yaw_error_filter_alpha) * error_linea);

    float P_yaw = Kp_yaw * fl_error_linea_filtrado;
    float D_yaw = -Kd_yaw * giro_z;
    last_error_yaw = fl_error_linea_filtrado;

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

    if (currentMode != CONTROL_MODE_OBSTACLE_FOLLOW &&
        (imu_last_update_tick == 0U ||
         (uint32_t)(now - imu_last_update_tick) > TURN_MANEUVER_IMU_STALE_MS)) {
        return TURN_MANEUVER_STATUS_SENSOR;
    }

    turn_maneuver_target_delta_deg = target_angle_deg;
    turn_maneuver_error_filtered = 0.0f;
    turn_maneuver_steering_slow = 0.0f;
    turn_maneuver_prepare_start_tick = now;
    turn_maneuver_mode = wheel_mode;
    turn_maneuver_wheel = wheel_select;
    turn_maneuver_arc_inner_ratio = ((float)inner_wheel_percent) / 100.0f;
    turn_maneuver_setpoint = TurnManeuver_GetActiveSetpoint();
    turn_maneuver_steering = 0;
    turn_debug_target_deg = target_angle_deg;
    turn_debug_turned_deg = 0.0f;
    turn_debug_remaining_deg = target_angle_deg;
    turn_debug_target_steering = 0.0f;
    turn_debug_steering_ramp = 0.0f;
    turn_debug_effective_limit = yaw_steering_limit;
    turn_debug_steering_clamped = 0;
    turn_debug_motor_left_cmd = 0;
    turn_debug_motor_right_cmd = 0;
    turn_debug_active_motor_cmd = 0;
    turn_debug_pivot_motor_cmd = 0;
    turn_debug_motor_saturated = 0;
    turn_debug_exit_reason = TURN_MANEUVER_EXIT_NONE;
    turn_debug_prepare_remaining_ms = turn_maneuver_pre_bias_delay_ms;
    if (turn_maneuver_pre_bias_delay_ms > 0U) {
        turn_maneuver_state = TURN_MANEUVER_STATE_PREPARING;
    } else {
        turn_maneuver_state = TURN_MANEUVER_STATE_TURNING;
        turn_maneuver_start_yaw_deg = angle_yaw;
        turn_maneuver_start_tick = now;
    }
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

void TurnManeuver_StoreCornerConfig(float target_angle_deg,
                                    uint8_t wheel_mode,
                                    uint8_t wheel_select,
                                    uint8_t inner_wheel_percent,
                                    float turn_bias_deg,
                                    uint16_t pre_bias_delay_ms)
{
    corner_turn_mode = wheel_mode;
    corner_turn_wheel = wheel_select;
    corner_turn_inner_wheel_percent = inner_wheel_percent;
    corner_turn_direction = (target_angle_deg < 0.0f) ? -1 : 1;
    corner_turn_bias_deg = turn_bias_deg;
    corner_turn_pre_bias_delay_ms = pre_bias_delay_ms;
}

uint8_t TurnManeuver_StartStoredCorner(float corner_angle_deg)
{
    float target_angle_deg = fabsf(corner_angle_deg) * (float)corner_turn_direction;

    turn_maneuver_forward_bias_deg = corner_turn_bias_deg;
    turn_maneuver_pre_bias_delay_ms = corner_turn_pre_bias_delay_ms;
    return TurnManeuver_StartInternal(target_angle_deg,
                                      corner_turn_mode,
                                      corner_turn_wheel,
                                      corner_turn_inner_wheel_percent);
}

void TurnManeuver_CancelWithReason(uint8_t reason)
{
    turn_debug_exit_reason = reason;
    turn_maneuver_active = 0;
    turn_maneuver_state = TURN_MANEUVER_STATE_IDLE;
    turn_maneuver_setpoint = 0.0f;
    turn_maneuver_steering = 0;
    turn_maneuver_steering_slow = 0.0f;
    turn_maneuver_error_filtered = 0.0f;
    turn_maneuver_arc_inner_ratio = 0.0f;
    turn_maneuver_settle_setpoint = 0.0f;
    turn_maneuver_settle_steering = 0.0f;
    turn_maneuver_settle_start_tick = 0U;
    turn_debug_target_steering = 0.0f;
    turn_debug_steering_ramp = 0.0f;
    turn_debug_prepare_remaining_ms = 0U;
    integral = 0.0f;
    I = 0.0f;
    RC_setpoint = 0.0f;
    RC_steering = 0;
}

void TurnManeuver_Cancel(void)
{
    TurnManeuver_CancelWithReason(TURN_MANEUVER_EXIT_EXTERNAL_CANCEL);
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
        TurnManeuver_CancelWithReason(!flagMotorsAreOn ?
                                      TURN_MANEUVER_EXIT_MOTORS_OFF :
                                      TURN_MANEUVER_EXIT_MODE_CHANGE);
        return;
    }
    if (currentMode != CONTROL_MODE_OBSTACLE_FOLLOW &&
        (imu_last_update_tick == 0U ||
         (uint32_t)(now - imu_last_update_tick) > TURN_MANEUVER_IMU_STALE_MS)) {
        TurnManeuver_CancelWithReason(TURN_MANEUVER_EXIT_IMU_STALE);
        Control_SetMotorsEnabled(0U);
        return;
    }

    if (turn_maneuver_state == TURN_MANEUVER_STATE_PREPARING) {
        uint32_t prepare_elapsed = now - turn_maneuver_prepare_start_tick;
        turn_maneuver_setpoint = TurnManeuver_GetActiveSetpoint();
        turn_maneuver_steering = 0;
        turn_maneuver_steering_slow = 0.0f;
        turn_debug_steering_ramp = 0.0f;

        if (prepare_elapsed < turn_maneuver_pre_bias_delay_ms) {
            turn_debug_prepare_remaining_ms =
                (uint16_t)(turn_maneuver_pre_bias_delay_ms - prepare_elapsed);
            return;
        }

        turn_debug_prepare_remaining_ms = 0U;
        turn_maneuver_state = TURN_MANEUVER_STATE_TURNING;
        turn_maneuver_start_yaw_deg = angle_yaw;
        turn_maneuver_start_tick = now;
        turn_maneuver_error_filtered = 0.0f;
    }

    if (currentMode != CONTROL_MODE_OBSTACLE_FOLLOW &&
        (uint32_t)(now - turn_maneuver_start_tick) > TURN_MANEUVER_TIMEOUT_MS) {
        TurnManeuver_CancelWithReason(TURN_MANEUVER_EXIT_TIMEOUT);
        return;
    }

    if (turn_maneuver_state == TURN_MANEUVER_STATE_SETTLING) {
        uint32_t settle_elapsed = now - turn_maneuver_settle_start_tick;
        if (settle_elapsed >= TURN_MANEUVER_SETTLE_MS) {
            TurnManeuver_CancelWithReason(TURN_MANEUVER_EXIT_TARGET_REACHED);
            return;
        }

        float scale = 1.0f - ((float)settle_elapsed / (float)TURN_MANEUVER_SETTLE_MS);
        turn_maneuver_setpoint = turn_maneuver_settle_setpoint * scale;
        turn_maneuver_steering_slow = turn_maneuver_settle_steering * scale;
        turn_maneuver_steering = (int16_t)turn_maneuver_steering_slow;
        turn_debug_steering_ramp = turn_maneuver_steering_slow;
        turn_debug_target_steering = 0.0f;
        return;
    }

    float turned_deg = angle_yaw - turn_maneuver_start_yaw_deg;
    float remaining_deg = turn_maneuver_target_delta_deg - turned_deg;
    turn_debug_target_deg = turn_maneuver_target_delta_deg;
    turn_debug_turned_deg = turned_deg;
    turn_debug_remaining_deg = remaining_deg;

    if (fabsf(remaining_deg) <= TURN_MANEUVER_TOLERANCE_DEG) {
        turn_debug_exit_reason = TURN_MANEUVER_EXIT_TARGET_REACHED;
        turn_maneuver_state = TURN_MANEUVER_STATE_SETTLING;
        turn_maneuver_settle_start_tick = now;
        turn_maneuver_settle_setpoint = TurnManeuver_GetActiveSetpoint();
        turn_maneuver_settle_steering = turn_maneuver_steering_slow;
        turn_maneuver_setpoint = turn_maneuver_settle_setpoint;
        return;
    }

    float yaw_limit = yaw_steering_limit;
    float yaw_error = remaining_deg * multiplicadorYaw;
    turn_maneuver_error_filtered =
        (yaw_error_filter_alpha * turn_maneuver_error_filtered) +
        ((1.0f - yaw_error_filter_alpha) * yaw_error);

    float effective_Kp = Kp_yaw + 1500.0f;
    float target_steering = -(((effective_Kp * turn_maneuver_error_filtered) * TURN_MANEUVER_YAW_BOOST) -
                              (Kd_yaw * giro_z));
    turn_debug_target_steering = target_steering;
    turn_debug_effective_limit = yaw_limit;
    turn_debug_steering_clamped = (yaw_limit > 0.0f && fabsf(target_steering) > yaw_limit) ? 1U : 0U;
    target_steering = clamp_float(target_steering, yaw_limit);

    float delta_steering = target_steering - turn_maneuver_steering_slow;
    delta_steering = clamp_float(delta_steering, yaw_steering_step_max);
    turn_maneuver_steering_slow += delta_steering;
    turn_maneuver_steering_slow = clamp_float(turn_maneuver_steering_slow, yaw_limit);

    turn_maneuver_steering = (int16_t)turn_maneuver_steering_slow;
    turn_debug_steering_ramp = turn_maneuver_steering_slow;
    turn_maneuver_setpoint = TurnManeuver_GetActiveSetpoint();
}
