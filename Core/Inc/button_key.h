#ifndef BUTTON_KEY_H
#define BUTTON_KEY_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KEY_GPIO_PORT        GPIOA
#define KEY_GPIO_PIN         GPIO_PIN_0
#define KEY_ACTIVE_STATE     GPIO_PIN_SET
#define KEY_DEBOUNCE_MS      50U

void KEY_CalibrationTask(void);

#ifdef __cplusplus
}
#endif

#endif
