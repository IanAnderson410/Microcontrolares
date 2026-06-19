#ifndef OBSTACLE_FOLLOW_H
#define OBSTACLE_FOLLOW_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OBSTACLE_FOLLOW_SIDE_RIGHT 1U

#define OBSTACLE_FOLLOW_STATUS_OK      0U
#define OBSTACLE_FOLLOW_STATUS_RANGE   1U
#define OBSTACLE_FOLLOW_STATUS_MODE    3U

typedef enum {
    OBSTACLE_FOLLOW_STATE_IDLE = 0,
    OBSTACLE_FOLLOW_STATE_FACE_ALIGN,
    OBSTACLE_FOLLOW_STATE_FACE_FOLLOW,
    OBSTACLE_FOLLOW_STATE_CORNER_TURN
} ObstacleFollowState_t;

typedef enum {
    OBSTACLE_RIGHT_FACE_LOST = 0,
    OBSTACLE_RIGHT_FACE_TOO_FAR,
    OBSTACLE_RIGHT_FACE_OK,
    OBSTACLE_RIGHT_FACE_TOO_CLOSE
} ObstacleRightFaceState_t;

extern volatile uint8_t obstacle_follow_active;
extern volatile uint8_t obstacle_follow_state;
extern volatile uint8_t obstacle_follow_side;
extern volatile uint8_t obstacle_right_face_state;
extern volatile uint16_t obstacle_right_ir_raw;
extern volatile uint16_t obstacle_right_ir_filtered;
extern volatile uint16_t obstacle_right_ir_baseline;
extern volatile uint16_t obstacle_rear_ir_raw;
extern volatile uint16_t obstacle_rear_ir_filtered;
extern volatile uint16_t obstacle_front_ir_raw;
extern volatile uint16_t obstacle_front_ir_filtered;
extern volatile uint16_t obstacle_follow_target_adc;
extern volatile int16_t obstacle_follow_adc_error;
extern volatile uint16_t obstacle_follow_target_mm;
extern volatile uint16_t obstacle_right_distance_mm;
extern volatile uint16_t obstacle_rear_distance_mm;
extern volatile uint16_t obstacle_front_distance_mm;
extern volatile int16_t obstacle_follow_distance_error_mm;
extern volatile int16_t obstacle_follow_parallel_error_mm;
extern volatile float obstacle_follow_wall_kp;
extern volatile int16_t obstacle_follow_wall_steering;
extern volatile int16_t obstacle_follow_yaw_error_cdeg;
extern volatile float obstacle_follow_setpoint;
extern volatile int16_t obstacle_follow_steering;
extern volatile int16_t obstacle_follow_side_steering;
extern volatile uint8_t obstacle_follow_steering_saturated;

uint8_t ObstacleFollow_Start(uint8_t side);
void ObstacleFollow_Stop(void);
void ObstacleFollow_Task(void);
uint8_t ObstacleFollow_IsActive(void);

#ifdef __cplusplus
}
#endif

#endif
