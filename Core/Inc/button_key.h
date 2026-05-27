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

void KEY_Init(void);
void KEY_CalibrationTask(void);

#ifdef __cplusplus
}
#endif

#endif
