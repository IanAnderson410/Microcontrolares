#include "obstacle_follow.h"
#include "control_systems.h"
#include "line_sensors.h"
#include <math.h>

#define OBSTACLE_RIGHT_ADC_INDEX             6U
#define OBSTACLE_RIGHT_TOO_CLOSE_THRESHOLD   600U
#define OBSTACLE_RIGHT_OK_LOW_THRESHOLD      600U
#define OBSTACLE_RIGHT_OK_HIGH_THRESHOLD     2700U
#define OBSTACLE_RIGHT_TOO_FAR_THRESHOLD     2700U
#define OBSTACLE_RIGHT_LOST_THRESHOLD        3900U
#define OBSTACLE_RIGHT_TARGET_ADC            1650U
#define OBSTACLE_FOLLOW_ALIGN_CONFIRM_TICKS  10U
#define OBSTACLE_FOLLOW_LOST_CONFIRM_TICKS   8U
#define OBSTACLE_FOLLOW_IMU_STALE_MS         100U
#define OBSTACLE_FOLLOW_YAW_LIMIT            260.0f
#define OBSTACLE_FOLLOW_SIDE_CORRECTION      200.0f
#define OBSTACLE_FOLLOW_BALANCE_ONLY_STEERING 250
#define OBSTACLE_FOLLOW_CORNER_TURN_DEG      90.0f
#define OBSTACLE_FOLLOW_RIGHT_STEER_SIGN     -1.0f

volatile uint8_t obstacle_follow_active = 0;
volatile uint8_t obstacle_follow_state = OBSTACLE_FOLLOW_STATE_IDLE;
volatile uint8_t obstacle_follow_side = OBSTACLE_FOLLOW_SIDE_RIGHT;
volatile uint8_t obstacle_right_face_state = OBSTACLE_RIGHT_FACE_LOST;
volatile uint16_t obstacle_right_ir_raw = 0;
volatile uint16_t obstacle_right_ir_filtered = 0;
volatile uint16_t obstacle_right_ir_baseline = 0;
volatile uint16_t obstacle_follow_target_adc = OBSTACLE_RIGHT_TARGET_ADC;
volatile int16_t obstacle_follow_adc_error = 0;
volatile int16_t obstacle_follow_yaw_error_cdeg = 0;
volatile float obstacle_follow_setpoint = 0.0f;
volatile int16_t obstacle_follow_steering = 0;
volatile int16_t obstacle_follow_side_steering = 0;
volatile uint8_t obstacle_follow_steering_saturated = 0;

static float obstacle_follow_yaw_reference = 0.0f;
static float obstacle_follow_yaw_error_filtered = 0.0f;
static float obstacle_follow_steering_slow = 0.0f;
static uint8_t obstacle_follow_align_count = 0;
static uint8_t obstacle_follow_lost_count = 0;
static uint32_t obstacle_follow_forward_phase_tick = 0;
static uint8_t obstacle_follow_motion_phase = 1;
static uint8_t obstacle_right_filter_ready = 0;

static float ObstacleFollow_ClampFloat(float value, float limit)
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

static void ObstacleFollow_UpdateRightSensor(void)
{
    obstacle_right_ir_raw = adc_buffer[OBSTACLE_RIGHT_ADC_INDEX];

    if (!obstacle_right_filter_ready) {
        obstacle_right_ir_filtered = obstacle_right_ir_raw;
        obstacle_right_ir_baseline = obstacle_right_ir_raw;
        obstacle_right_filter_ready = 1U;
    } else {
        obstacle_right_ir_filtered = (uint16_t)(((uint32_t)obstacle_right_ir_filtered * 7U +
                                                 (uint32_t)obstacle_right_ir_raw * 3U) / 10U);
    }

    if (obstacle_right_ir_filtered > OBSTACLE_RIGHT_LOST_THRESHOLD) {
        obstacle_right_ir_baseline = (uint16_t)(((uint32_t)obstacle_right_ir_baseline * 63U +
                                                 (uint32_t)obstacle_right_ir_filtered) / 64U);
    }

    adc_filtrado[OBSTACLE_RIGHT_ADC_INDEX] = obstacle_right_ir_filtered;

    if (obstacle_right_ir_filtered >= OBSTACLE_RIGHT_LOST_THRESHOLD) {
        obstacle_right_face_state = OBSTACLE_RIGHT_FACE_LOST;
    } else if (obstacle_right_ir_filtered > OBSTACLE_RIGHT_TOO_FAR_THRESHOLD) {
        obstacle_right_face_state = OBSTACLE_RIGHT_FACE_TOO_FAR;
    } else if (obstacle_right_ir_filtered < OBSTACLE_RIGHT_TOO_CLOSE_THRESHOLD) {
        obstacle_right_face_state = OBSTACLE_RIGHT_FACE_TOO_CLOSE;
    } else {
        obstacle_right_face_state = OBSTACLE_RIGHT_FACE_OK;
    }
}

static void ObstacleFollow_SetState(uint8_t state)
{
    obstacle_follow_state = state;
    obstacle_follow_align_count = 0;
    obstacle_follow_lost_count = 0;
}

static void ObstacleFollow_ClearOutput(void)
{
    obstacle_follow_steering = 0;
    obstacle_follow_side_steering = 0;
    obstacle_follow_adc_error = 0;
    obstacle_follow_yaw_error_cdeg = 0;
    obstacle_follow_setpoint = 0.0f;
    obstacle_follow_steering_saturated = 0;
    obstacle_follow_steering_slow = 0.0f;
    obstacle_follow_yaw_error_filtered = 0.0f;
    obstacle_follow_forward_phase_tick = 0;
    obstacle_follow_motion_phase = 1U;
    obstacle_follow_lost_count = 0;
}

uint8_t ObstacleFollow_Start(uint8_t side)
{
    if (side != OBSTACLE_FOLLOW_SIDE_RIGHT) {
        return OBSTACLE_FOLLOW_STATUS_RANGE;
    }
    if ((currentMode != CONTROL_MODE_RC && currentMode != CONTROL_MODE_OBSTACLE_FOLLOW) ||
        turn_maneuver_active) {
        return OBSTACLE_FOLLOW_STATUS_MODE;
    }

    obstacle_follow_side = side;
    obstacle_follow_active = 1U;
    currentMode = CONTROL_MODE_OBSTACLE_FOLLOW;
    flag_RC_active = 0U;
    RC_setpoint = 0.0f;
    RC_steering = 0;
    ObstacleFollow_ClearOutput();
    ObstacleFollow_UpdateRightSensor();
    obstacle_follow_yaw_reference = angle_yaw;
    ObstacleFollow_SetState(OBSTACLE_FOLLOW_STATE_FACE_ALIGN);

    return OBSTACLE_FOLLOW_STATUS_OK;
}

void ObstacleFollow_Stop(void)
{
    obstacle_follow_active = 0U;
    ObstacleFollow_ClearOutput();
    ObstacleFollow_SetState(OBSTACLE_FOLLOW_STATE_IDLE);
}

uint8_t ObstacleFollow_IsActive(void)
{
    return obstacle_follow_active;
}

void ObstacleFollow_Task(void)
{
    float target_steering = 0.0f;
    float side_steering = 0.0f;
    float target_setpoint = 0.0f;
    uint32_t now = HAL_GetTick();

    ObstacleFollow_UpdateRightSensor();
    obstacle_follow_adc_error = (int16_t)obstacle_right_ir_filtered - (int16_t)OBSTACLE_RIGHT_TARGET_ADC;

    if (!obstacle_follow_active) {
        return;
    }

    if (currentMode != CONTROL_MODE_OBSTACLE_FOLLOW ||
        (turn_maneuver_active && obstacle_follow_state != OBSTACLE_FOLLOW_STATE_CORNER_TURN)) {
        ObstacleFollow_Stop();
        return;
    }

    if (!flagMotorsAreOn ||
        imu_last_update_tick == 0U ||
        (uint32_t)(now - imu_last_update_tick) > OBSTACLE_FOLLOW_IMU_STALE_MS) {
        ObstacleFollow_ClearOutput();
        return;
    }

    obstacle_follow_setpoint = 0.0f;

    switch (obstacle_follow_state) {
    case OBSTACLE_FOLLOW_STATE_FACE_ALIGN:
        if (obstacle_right_face_state == OBSTACLE_RIGHT_FACE_OK ||
            obstacle_right_face_state == OBSTACLE_RIGHT_FACE_TOO_CLOSE ||
            obstacle_right_face_state == OBSTACLE_RIGHT_FACE_TOO_FAR) {
            if (obstacle_follow_align_count < OBSTACLE_FOLLOW_ALIGN_CONFIRM_TICKS) {
                obstacle_follow_align_count++;
            }
        } else {
            obstacle_follow_align_count = 0;
        }

        if (obstacle_follow_align_count >= OBSTACLE_FOLLOW_ALIGN_CONFIRM_TICKS) {
            obstacle_follow_yaw_reference = angle_yaw;
            ObstacleFollow_SetState(OBSTACLE_FOLLOW_STATE_FACE_FOLLOW);
        }
        break;

    case OBSTACLE_FOLLOW_STATE_FACE_FOLLOW:
        if (obstacle_right_face_state == OBSTACLE_RIGHT_FACE_LOST) {
            if (obstacle_follow_lost_count < OBSTACLE_FOLLOW_LOST_CONFIRM_TICKS) {
                obstacle_follow_lost_count++;
            }
            if (obstacle_follow_lost_count >= OBSTACLE_FOLLOW_LOST_CONFIRM_TICKS) {
                ObstacleFollow_ClearOutput();
                if (TurnManeuver_Start(-95, TURN_MANEUVER_MODE_ONE_WHEEL,
                                       TURN_MANEUVER_WHEEL_LEFT) == TURN_MANEUVER_STATUS_OK) {
                    ObstacleFollow_SetState(OBSTACLE_FOLLOW_STATE_CORNER_TURN);
                } else {
                    ObstacleFollow_SetState(OBSTACLE_FOLLOW_STATE_FACE_ALIGN);
                }
            }
            break;
        }
        obstacle_follow_lost_count = 0;

        if (obstacle_right_face_state == OBSTACLE_RIGHT_FACE_TOO_CLOSE) {
            side_steering = -OBSTACLE_FOLLOW_RIGHT_STEER_SIGN * OBSTACLE_FOLLOW_SIDE_CORRECTION;
        } else if (obstacle_right_face_state == OBSTACLE_RIGHT_FACE_TOO_FAR) {
            side_steering = OBSTACLE_FOLLOW_RIGHT_STEER_SIGN * OBSTACLE_FOLLOW_SIDE_CORRECTION;
        }
        target_steering += side_steering;
        break;

    case OBSTACLE_FOLLOW_STATE_CORNER_TURN:
        if (turn_maneuver_active) {
            obstacle_follow_steering = 0;
            obstacle_follow_side_steering = 0;
            return;
        }
        ObstacleFollow_ClearOutput();
        ObstacleFollow_SetState(OBSTACLE_FOLLOW_STATE_FACE_ALIGN);
        break;

    case OBSTACLE_FOLLOW_STATE_IDLE:
    default:
        ObstacleFollow_Stop();
        return;
    }

    if (obstacle_follow_state == OBSTACLE_FOLLOW_STATE_FACE_FOLLOW) {
        float yaw_error_deg = obstacle_follow_yaw_reference - angle_yaw;
        float yaw_error = yaw_error_deg * multiplicadorYaw;
        obstacle_follow_yaw_error_cdeg = (int16_t)(yaw_error_deg * 100.0f);
        obstacle_follow_yaw_error_filtered =
            (yaw_error_filter_alpha * obstacle_follow_yaw_error_filtered) +
            ((1.0f - yaw_error_filter_alpha) * yaw_error);

        target_steering += (Kp_yaw * obstacle_follow_yaw_error_filtered) - (Kd_yaw * giro_z);
    } else {
        obstacle_follow_yaw_error_cdeg = 0;
    }

    obstacle_follow_side_steering = (int16_t)side_steering;
    obstacle_follow_steering_saturated =
        (target_steering > OBSTACLE_FOLLOW_YAW_LIMIT || target_steering < -OBSTACLE_FOLLOW_YAW_LIMIT) ? 1U : 0U;
    target_steering = ObstacleFollow_ClampFloat(target_steering, OBSTACLE_FOLLOW_YAW_LIMIT);

    float delta_steering = target_steering - obstacle_follow_steering_slow;
    delta_steering = ObstacleFollow_ClampFloat(delta_steering, yaw_steering_step_max);
    obstacle_follow_steering_slow += delta_steering;
    obstacle_follow_steering_slow = ObstacleFollow_ClampFloat(obstacle_follow_steering_slow,
                                                              OBSTACLE_FOLLOW_YAW_LIMIT);

    obstacle_follow_steering = (int16_t)obstacle_follow_steering_slow;
    if (obstacle_follow_state == OBSTACLE_FOLLOW_STATE_FACE_FOLLOW) {
        if (obstacle_follow_forward_phase_tick == 0U) {
            obstacle_follow_forward_phase_tick = now;
            obstacle_follow_motion_phase = 1U;
        }

        uint16_t phase_ms = obstacle_follow_motion_phase ? FL_motion_phase_ms : FL_balance_phase_ms;
        if ((uint32_t)(now - obstacle_follow_forward_phase_tick) >= phase_ms) {
            obstacle_follow_forward_phase_tick = now;
            obstacle_follow_motion_phase = !obstacle_follow_motion_phase;
        }

        if (obstacle_follow_steering > OBSTACLE_FOLLOW_BALANCE_ONLY_STEERING ||
            obstacle_follow_steering < -OBSTACLE_FOLLOW_BALANCE_ONLY_STEERING) {
            target_setpoint = 0.0f;
        } else {
            target_setpoint = obstacle_follow_motion_phase ? FL_setpoint : 0.0f;
        }
    } else {
        obstacle_follow_forward_phase_tick = 0U;
        obstacle_follow_motion_phase = 1U;
    }

    obstacle_follow_setpoint = target_setpoint;
}
