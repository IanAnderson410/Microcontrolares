#ifndef BUTTON_KEY_H
#define BUTTON_KEY_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KEY_GPIO_PORT        GPIOA
#define KEY_GPIO_PIN         GPIO_PIN_0
#define KEY_ACTIVE_STATE     GPIO_PIN_RESET
#define KEY_GPIO_PULL        GPIO_PULLUP
#define KEY_DEBOUNCE_MS      50U
#define KEY_LONG_PRESS_MS    1000U
#define KEY_DOUBLE_CLICK_MS  500U

#define KEY_EVENT_NONE          0U
#define KEY_EVENT_SHORT_PRESS   1U
#define KEY_EVENT_DOUBLE_CLICK  2U
#define KEY_EVENT_LONG_PRESS    3U

#define KEY_ACTION_NONE         0U
#define KEY_ACTION_CAL_START    1U
#define KEY_ACTION_CAL_DONE     2U
#define KEY_ACTION_MODE_RC      3U
#define KEY_ACTION_MODE_FL      4U
#define KEY_ACTION_MOTORS_ON    5U
#define KEY_ACTION_MOTORS_OFF   6U

extern volatile uint8_t key_stable_pressed;
extern volatile uint8_t key_raw_pressed;
extern volatile uint8_t key_click_pending;
extern volatile uint8_t key_long_press_done;
extern volatile uint8_t key_last_event;
extern volatile uint8_t key_last_action;
extern volatile uint32_t key_press_duration_ms;
extern volatile uint32_t key_last_event_tick;

void KEY_Init(void);
void KEY_CalibrationTask(void);

#ifdef __cplusplus
}
#endif

#endif
