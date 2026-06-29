#include "obstacle_follow.h"
#include "control_systems.h"
#include "line_sensors.h"
#include <math.h>

#define OBSTACLE_REAR_ADC_INDEX              		4U
#define OBSTACLE_FRONT_ADC_INDEX             		6U
#define OBSTACLE_RIGHT_TABLE_POINTS          		7U
#define OBSTACLE_RIGHT_TOO_CLOSE_MM          		30U
#define OBSTACLE_RIGHT_TOO_FAR_MM            		55U
#define OBSTACLE_RIGHT_LOST_THRESHOLD        		3600U
#define OBSTACLE_RIGHT_TARGET_ADC            		2431U
#define OBSTACLE_RIGHT_TARGET_MM_DEFAULT     		50
#define OBSTACLE_FOLLOW_ALIGN_CONFIRM_TICKS  		10
#define OBSTACLE_FOLLOW_LOST_CONFIRM_TICKS   		100		/*!< Si pierdo la lectura de ambos sensores al mismo tiemmpo por más de 2 segundos, paro la maniobra*/
#define OBSTACLE_FOLLOW_CORNER_EXIT_CONFIRM_TICKS 	20
#define OBSTACLE_FOLLOW_LINE_REACQUIRE_CONFIRM_TICKS 5U
#define OBSTACLE_FOLLOW_CORRECTION_HISTORY_SIZE 	5U
#define OBSTACLE_FOLLOW_IMU_STALE_MS         		100U
#define OBSTACLE_FOLLOW_YAW_LIMIT            		3600.0f	/*!< Límite para el Yaw. Actualmente desactivado*/
#define OBSTACLE_FOLLOW_WALL_KP_DEFAULT      		90.0f		/*!< Término proporcional del giro de posicionamiento paralelo al obstáculo*/
#define OBSTACLE_FOLLOW_DISTANCE_KP_DEFAULT  		2.0f		/*!< Término proporcional del giro ajustador de dsitacia*/
#define OBSTACLE_FOLLOW_WALL_STEER_LIMIT     		3600.0f
#define OBSTACLE_FOLLOW_RIGHT_STEER_SIGN     		-1.0f
#define OBSTACLE_REAR_ADC_OFFSET_MIN         		(-100000L)
#define OBSTACLE_REAR_ADC_OFFSET_MAX         100000L
//no se si se utiliza, evaluare quitarlo
#define OBSTACLE_FOLLOW_CORNER_TURN_DEG      90.0f
#define OBSTACLE_FOLLOW_LINE_EXIT_TURN_DEG   30.0f

static const uint16_t obstacle_rear_table_adc[OBSTACLE_RIGHT_TABLE_POINTS] = {
    159U, 206U, 251U, 784U, 1757U, 2138U, 2398U
};
static const uint16_t obstacle_front_table_adc[OBSTACLE_RIGHT_TABLE_POINTS] = {
    175U, 253U, 996U, 1737U, 2470U, 2758U, 2946U
};
static const uint16_t obstacle_right_table_mm[OBSTACLE_RIGHT_TABLE_POINTS] = {
    10U, 20U, 30U, 40U, 50U, 60U, 70U
};

volatile uint8_t obstacle_follow_active = 0;
volatile uint8_t obstacle_follow_state = OBSTACLE_FOLLOW_STATE_IDLE;
volatile uint8_t obstacle_follow_side = OBSTACLE_FOLLOW_SIDE_RIGHT;
volatile uint8_t obstacle_right_face_state = OBSTACLE_RIGHT_FACE_LOST;
volatile uint16_t obstacle_right_ir_raw = 0;
volatile uint16_t obstacle_right_ir_filtered = 0;
volatile uint16_t obstacle_right_ir_baseline = 0;
volatile uint16_t obstacle_rear_ir_raw = 0;
volatile uint16_t obstacle_rear_ir_filtered = 0;
volatile int32_t obstacle_rear_ir_adc_offset = 0;
volatile uint16_t obstacle_front_ir_raw = 0;
volatile uint16_t obstacle_front_ir_filtered = 0;
volatile uint16_t obstacle_follow_target_adc = OBSTACLE_RIGHT_TARGET_ADC;
volatile int16_t obstacle_follow_adc_error = 0;
volatile uint16_t obstacle_follow_target_mm = OBSTACLE_RIGHT_TARGET_MM_DEFAULT;
volatile uint16_t obstacle_right_distance_mm = OBSTACLE_RIGHT_TARGET_MM_DEFAULT;
volatile uint16_t obstacle_rear_distance_mm = OBSTACLE_RIGHT_TARGET_MM_DEFAULT;
volatile uint16_t obstacle_front_distance_mm = OBSTACLE_RIGHT_TARGET_MM_DEFAULT;
volatile int16_t obstacle_follow_distance_error_mm = 0;
volatile int16_t obstacle_follow_parallel_error_mm = 0;
volatile float obstacle_follow_wall_kp = OBSTACLE_FOLLOW_WALL_KP_DEFAULT;
volatile float obstacle_follow_distance_kp = OBSTACLE_FOLLOW_DISTANCE_KP_DEFAULT;
volatile int16_t obstacle_follow_wall_steering = 0;
volatile int16_t obstacle_follow_yaw_error_cdeg = 0;
volatile float obstacle_follow_setpoint = 0.0f;
volatile int16_t obstacle_follow_steering = 0;
volatile int16_t obstacle_follow_side_steering = 0;
volatile uint8_t obstacle_follow_steering_saturated = 0;
volatile uint8_t obstacle_follow_lost_turn_enabled = 1U;

static float obstacle_follow_yaw_reference = 0.0f;
static float obstacle_follow_yaw_error_filtered = 0.0f;
static float obstacle_follow_steering_slow = 0.0f;
static uint8_t obstacle_follow_align_count = 0;
static uint8_t obstacle_follow_lost_count = 0;
static uint32_t obstacle_follow_forward_phase_tick = 0;
static uint8_t obstacle_follow_motion_phase = 1;
static uint8_t obstacle_ir_filter_ready = 0;
static uint16_t obstacle_rear_ir_filter_state = 0;
static float obstacle_follow_last_valid_side_steering = 0.0f;
static uint8_t obstacle_follow_fixed_corner_active = 0U;
static uint8_t obstacle_follow_corner_exit_count = 0U;
static float obstacle_follow_fixed_corner_steering = 0.0f;
static float obstacle_follow_valid_side_steering_history[OBSTACLE_FOLLOW_CORRECTION_HISTORY_SIZE] = {0.0f};
static float obstacle_follow_valid_side_steering_sum = 0.0f;
static uint8_t obstacle_follow_valid_side_steering_index = 0U;
static uint8_t obstacle_follow_valid_side_steering_count = 0U;
static uint8_t obstacle_follow_line_reacquire_count = 0U;
static uint8_t obstacle_follow_line_reacquire_armed = 0U;
static uint8_t obstacle_follow_line_exit_turn_active = 0U;

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

// Interpola la tabla ADC-distancia para obtener una distancia aproximada en milimetros.
static uint16_t ObstacleFollow_EstimateDistanceMm(uint16_t adc, const uint16_t *adc_table)
{
    if (adc <= adc_table[0]) {
        return obstacle_right_table_mm[0];
    }

    for (uint8_t i = 1U; i < OBSTACLE_RIGHT_TABLE_POINTS; i++) {
        if (adc <= adc_table[i]) {
            uint32_t adc_low = adc_table[i - 1U];
            uint32_t adc_high = adc_table[i];
            uint32_t mm_low = obstacle_right_table_mm[i - 1U];
            uint32_t mm_high = obstacle_right_table_mm[i];
            uint32_t adc_span = adc_high - adc_low;

            if (adc_span == 0U) {
                return (uint16_t)mm_low;
            }

            return (uint16_t)(mm_low + (((uint32_t)adc - adc_low) * (mm_high - mm_low)) / adc_span);
        }
    }

    return obstacle_right_table_mm[OBSTACLE_RIGHT_TABLE_POINTS - 1U];
}

static uint16_t ObstacleFollow_ApplyRearAdcOffset(uint16_t filtered_adc)
{
    int32_t adjusted_adc = (int32_t)filtered_adc + obstacle_rear_ir_adc_offset;

    if (adjusted_adc < 0L) {
        return 0U;
    }
    if (adjusted_adc > 4095L) {
        return 4095U;
    }

    return (uint16_t)adjusted_adc;
}

static void ObstacleFollow_ResetValidSteeringHistory(void)
{
    for (uint8_t i = 0U; i < OBSTACLE_FOLLOW_CORRECTION_HISTORY_SIZE; i++) {
        obstacle_follow_valid_side_steering_history[i] = 0.0f;
    }
    obstacle_follow_valid_side_steering_sum = 0.0f;
    obstacle_follow_valid_side_steering_index = 0U;
    obstacle_follow_valid_side_steering_count = 0U;
    obstacle_follow_last_valid_side_steering = 0.0f;
}

static void ObstacleFollow_UpdateValidSteeringHistory(float side_steering)
{
    if (obstacle_follow_valid_side_steering_count < OBSTACLE_FOLLOW_CORRECTION_HISTORY_SIZE) {
        obstacle_follow_valid_side_steering_history[obstacle_follow_valid_side_steering_index] = side_steering;
        obstacle_follow_valid_side_steering_sum += side_steering;
        obstacle_follow_valid_side_steering_count++;
    } else {
        obstacle_follow_valid_side_steering_sum -=
            obstacle_follow_valid_side_steering_history[obstacle_follow_valid_side_steering_index];
        obstacle_follow_valid_side_steering_history[obstacle_follow_valid_side_steering_index] = side_steering;
        obstacle_follow_valid_side_steering_sum += side_steering;
    }

    obstacle_follow_valid_side_steering_index++;
    if (obstacle_follow_valid_side_steering_index >= OBSTACLE_FOLLOW_CORRECTION_HISTORY_SIZE) {
        obstacle_follow_valid_side_steering_index = 0U;
    }

    if (obstacle_follow_valid_side_steering_count > 0U) {
        obstacle_follow_last_valid_side_steering =
            obstacle_follow_valid_side_steering_sum / (float)obstacle_follow_valid_side_steering_count;
    }
}

static void ObstacleFollow_StartFixedCorner(void)
{
    float sign = 0.0f;
    float magnitude = fabsf(obstacle_follow_last_valid_side_steering);

    if (obstacle_follow_last_valid_side_steering > 0.0f) {
        sign = 1.0f;
    } else if (obstacle_follow_last_valid_side_steering < 0.0f) {
        sign = -1.0f;
    }

    obstacle_follow_fixed_corner_steering = magnitude * sign;
    obstacle_follow_fixed_corner_active = 1U;
    obstacle_follow_corner_exit_count = 0U;
}

// Actualiza y filtra los IR derechos usados para seguir la cara del obstaculo.
static void ObstacleFollow_UpdateRightSensor(void)
{
    obstacle_rear_ir_raw = adc_buffer[OBSTACLE_REAR_ADC_INDEX];
    obstacle_front_ir_raw = adc_buffer[OBSTACLE_FRONT_ADC_INDEX];
    obstacle_right_ir_raw = obstacle_front_ir_raw;

    if (!obstacle_ir_filter_ready) {
        obstacle_rear_ir_filter_state = obstacle_rear_ir_raw;
        obstacle_rear_ir_filtered = ObstacleFollow_ApplyRearAdcOffset(obstacle_rear_ir_filter_state);
        obstacle_front_ir_filtered = obstacle_front_ir_raw;
        obstacle_right_ir_filtered = obstacle_front_ir_filtered;
        obstacle_right_ir_baseline = obstacle_right_ir_raw;
        obstacle_ir_filter_ready = 1U;
    } else {
        obstacle_rear_ir_filter_state = (uint16_t)(((uint32_t)obstacle_rear_ir_filter_state * 7U +
                                                    (uint32_t)obstacle_rear_ir_raw * 3U) / 10U);
        obstacle_rear_ir_filtered = ObstacleFollow_ApplyRearAdcOffset(obstacle_rear_ir_filter_state);
        obstacle_front_ir_filtered = (uint16_t)(((uint32_t)obstacle_front_ir_filtered * 7U +
                                                 (uint32_t)obstacle_front_ir_raw * 3U) / 10U);
        obstacle_right_ir_filtered = obstacle_front_ir_filtered;
    }

    adc_filtrado[OBSTACLE_REAR_ADC_INDEX] = obstacle_rear_ir_filtered;
    adc_filtrado[OBSTACLE_FRONT_ADC_INDEX] = obstacle_front_ir_filtered;
    obstacle_rear_distance_mm = ObstacleFollow_EstimateDistanceMm(obstacle_rear_ir_filtered,
                                                                  obstacle_rear_table_adc);
    obstacle_front_distance_mm = ObstacleFollow_EstimateDistanceMm(obstacle_front_ir_filtered,
                                                                   obstacle_front_table_adc);
    obstacle_right_distance_mm = obstacle_front_distance_mm;

    if (obstacle_front_ir_filtered >= OBSTACLE_RIGHT_LOST_THRESHOLD) {
        obstacle_right_face_state = OBSTACLE_RIGHT_FACE_LOST;
    } else if (obstacle_front_distance_mm > OBSTACLE_RIGHT_TOO_FAR_MM &&
        obstacle_rear_distance_mm > OBSTACLE_RIGHT_TOO_FAR_MM) {
        obstacle_right_face_state = OBSTACLE_RIGHT_FACE_TOO_FAR;
    } else if (obstacle_front_distance_mm < OBSTACLE_RIGHT_TOO_CLOSE_MM &&
               obstacle_rear_distance_mm < OBSTACLE_RIGHT_TOO_CLOSE_MM) {
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
    obstacle_follow_distance_error_mm = 0;
    obstacle_follow_parallel_error_mm = 0;
    obstacle_follow_wall_steering = 0;
    obstacle_follow_yaw_error_cdeg = 0;
    obstacle_follow_setpoint = 0.0f;
    obstacle_follow_steering_saturated = 0;
    obstacle_follow_steering_slow = 0.0f;
    obstacle_follow_yaw_error_filtered = 0.0f;
    obstacle_follow_forward_phase_tick = 0;
    obstacle_follow_motion_phase = 1U;
    obstacle_follow_lost_count = 0;
    obstacle_follow_fixed_corner_active = 0U;
    obstacle_follow_corner_exit_count = 0U;
    obstacle_follow_fixed_corner_steering = 0.0f;
    obstacle_follow_line_reacquire_count = 0U;
    ObstacleFollow_ResetValidSteeringHistory();
}

// Activa el modo de seguimiento de obstaculo y toma la referencia inicial de yaw.
uint8_t ObstacleFollow_Start(uint8_t side)
{
    if (side != OBSTACLE_FOLLOW_SIDE_RIGHT) {
        return OBSTACLE_FOLLOW_STATUS_RANGE;
    }
    if ((currentMode != CONTROL_MODE_RC &&
         currentMode != CONTROL_MODE_OBSTACLE_FOLLOW &&
         currentMode != CONTROL_MODE_FL_CIRCLE_ALIGN) ||
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
    obstacle_follow_line_reacquire_armed = AIRAB ? 0U : 1U;
    obstacle_follow_line_exit_turn_active = 0U;
    ObstacleFollow_UpdateRightSensor();
    obstacle_follow_yaw_reference = angle_yaw;
    ObstacleFollow_SetState(OBSTACLE_FOLLOW_STATE_FACE_ALIGN);

    return OBSTACLE_FOLLOW_STATUS_OK;
}

// Detiene el modo de obstaculo y limpia sus salidas de control.
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

// Maquina de estados para alinearse, seguir la pared y retornar a la linea.
void ObstacleFollow_Task(void){
    float target_steering = 0.0f;
    float side_steering = 0.0f;
    float target_setpoint = 0.0f;
    float average_distance_mm = 0.0f;
    float proportional_correction = 0.0f;
    uint8_t front_lost = 0U;
    uint8_t rear_lost = 0U;
    uint32_t now = HAL_GetTick();

    ObstacleFollow_UpdateRightSensor();
    front_lost = (obstacle_front_ir_filtered >= OBSTACLE_RIGHT_LOST_THRESHOLD) ? 1U : 0U;
    rear_lost = (obstacle_rear_ir_filtered >= OBSTACLE_RIGHT_LOST_THRESHOLD) ? 1U : 0U;
    obstacle_follow_adc_error = (int16_t)obstacle_right_ir_filtered - (int16_t)OBSTACLE_RIGHT_TARGET_ADC;
    average_distance_mm = ((float)obstacle_front_distance_mm + (float)obstacle_rear_distance_mm) * 0.5f;
    obstacle_follow_distance_error_mm = (int16_t)(average_distance_mm - (float)obstacle_follow_target_mm);
    obstacle_follow_parallel_error_mm = (int16_t)obstacle_front_distance_mm - (int16_t)obstacle_rear_distance_mm;
    if (!obstacle_follow_active)   return;

    if (currentMode != CONTROL_MODE_OBSTACLE_FOLLOW ||
        (turn_maneuver_active && obstacle_follow_state != OBSTACLE_FOLLOW_STATE_CORNER_TURN)) {
        ObstacleFollow_Stop();
        return;
    }
    if (!flagMotorsAreOn){
        ObstacleFollow_ClearOutput();
        return;
    }

    obstacle_follow_setpoint = 0.0f;

    switch (obstacle_follow_state) {
    case OBSTACLE_FOLLOW_STATE_FACE_ALIGN:
        if (obstacle_follow_align_count < OBSTACLE_FOLLOW_ALIGN_CONFIRM_TICKS) {
            obstacle_follow_align_count++;
        }

        if (obstacle_follow_align_count >= OBSTACLE_FOLLOW_ALIGN_CONFIRM_TICKS) {
            obstacle_follow_yaw_reference = angle_yaw;
            ObstacleFollow_SetState(OBSTACLE_FOLLOW_STATE_FACE_FOLLOW);
        }
        break;

    case OBSTACLE_FOLLOW_STATE_FACE_FOLLOW:
        if (!obstacle_follow_line_reacquire_armed) {
            obstacle_follow_line_reacquire_armed = AIRAB ? 0U : 1U;
            obstacle_follow_line_reacquire_count = 0U;
        } else if (AIRAB) {
            if (obstacle_follow_line_reacquire_count < OBSTACLE_FOLLOW_LINE_REACQUIRE_CONFIRM_TICKS) {
                obstacle_follow_line_reacquire_count++;
            }
        } else {
            obstacle_follow_line_reacquire_count = 0U;
        }

        if (obstacle_follow_line_reacquire_count >= OBSTACLE_FOLLOW_LINE_REACQUIRE_CONFIRM_TICKS) {
            obstacle_follow_line_reacquire_count = 0U;
            obstacle_follow_steering = 0;
            obstacle_follow_side_steering = 0;
            obstacle_follow_setpoint = 0.0f;
            if (TurnManeuver_Start(OBSTACLE_FOLLOW_LINE_EXIT_TURN_DEG,
                                   TURN_MANEUVER_MODE_TWO_WHEELS,
                                   TURN_MANEUVER_WHEEL_RIGHT) == TURN_MANEUVER_STATUS_OK) {
                obstacle_follow_line_exit_turn_active = 1U;
                ObstacleFollow_SetState(OBSTACLE_FOLLOW_STATE_CORNER_TURN);
            } else {
                ObstacleFollow_Stop();
                currentMode = CONTROL_MODE_FL_BUSQUEDA_INICIAL;
            }
            break;
        }

        if (front_lost && rear_lost) {
            break;
        }

        if (obstacle_follow_fixed_corner_active) {
            if (obstacle_front_distance_mm <= 10U) {
                if (obstacle_follow_corner_exit_count < OBSTACLE_FOLLOW_CORNER_EXIT_CONFIRM_TICKS) {
                    obstacle_follow_corner_exit_count++;
                }
            } else {
                obstacle_follow_corner_exit_count = 0U;
            }

            if (obstacle_follow_corner_exit_count >= OBSTACLE_FOLLOW_CORNER_EXIT_CONFIRM_TICKS) {
                obstacle_follow_fixed_corner_active = 0U;
                obstacle_follow_corner_exit_count = 0U;
            } else {
                obstacle_follow_lost_count = 0;
                side_steering = obstacle_follow_fixed_corner_steering;
                target_steering += side_steering;
                break;
            }
        }

        if (front_lost && obstacle_rear_distance_mm > 10U) {
            break;
        }

        if (rear_lost) {
            obstacle_follow_lost_count = 0;
            ObstacleFollow_ClearOutput();
            if (obstacle_follow_lost_turn_enabled &&
                TurnManeuver_StartStoredCorner(OBSTACLE_FOLLOW_CORNER_TURN_DEG) ==
                TURN_MANEUVER_STATUS_OK) {
                ObstacleFollow_SetState(OBSTACLE_FOLLOW_STATE_CORNER_TURN);
            } else {
                ObstacleFollow_SetState(OBSTACLE_FOLLOW_STATE_FACE_ALIGN);
            }
            break;
        }
        obstacle_follow_lost_count = 0;

        // La correccion combina paralelismo de sensores y distancia media al obstaculo.
        proportional_correction = (obstacle_follow_wall_kp * (float)obstacle_follow_parallel_error_mm) -
                                  (obstacle_follow_distance_kp * (float)obstacle_follow_distance_error_mm);
        side_steering = -OBSTACLE_FOLLOW_RIGHT_STEER_SIGN *
                        proportional_correction;
        side_steering = ObstacleFollow_ClampFloat(side_steering, OBSTACLE_FOLLOW_WALL_STEER_LIMIT);
        ObstacleFollow_UpdateValidSteeringHistory(side_steering);
        target_steering += side_steering;
        break;

    case OBSTACLE_FOLLOW_STATE_CORNER_TURN:
        if (turn_maneuver_active) {
            obstacle_follow_steering = 0;
            obstacle_follow_side_steering = 0;
            return;
        }
        if (obstacle_follow_line_exit_turn_active) {
            obstacle_follow_line_exit_turn_active = 0U;
            ObstacleFollow_Stop();
            currentMode = CONTROL_MODE_FL_BUSQUEDA_INICIAL;
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

    obstacle_follow_yaw_error_cdeg = 0;
    obstacle_follow_yaw_error_filtered = 0.0f;

    obstacle_follow_side_steering = (int16_t)side_steering;
    obstacle_follow_wall_steering = obstacle_follow_side_steering;
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
        target_setpoint = ForwardMotion_Generate(-FL_setpoint,
                                                 obstacle_follow_steering,
                                                 now,
                                                 &obstacle_follow_forward_phase_tick,
                                                 &obstacle_follow_motion_phase);
    } else {
        obstacle_follow_forward_phase_tick = 0U;
        obstacle_follow_motion_phase = 1U;
    }

    obstacle_follow_setpoint = target_setpoint;
}
