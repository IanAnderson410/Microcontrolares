#include "button_key.h"
#include "line_sensors.h"

#define KEY_MODE_FL_INICIO             2U
#define KEY_MODE_FL_BUSQUEDA_INICIAL   3U

extern volatile uint8_t currentMode;

void screenScheduler(void);

void KEY_CalibrationTask(void)
{
    static GPIO_PinState last_raw_state = GPIO_PIN_RESET;
    static GPIO_PinState stable_state = GPIO_PIN_RESET;
    static uint32_t last_change_tick = 0;

    GPIO_PinState raw_state = HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY_GPIO_PIN);
    uint32_t now = HAL_GetTick();

    if (raw_state != last_raw_state) {
        last_raw_state = raw_state;
        last_change_tick = now;
        return;
    }

    if ((now - last_change_tick) < KEY_DEBOUNCE_MS) {
        return;
    }

    if (raw_state == stable_state) {
        return;
    }

    stable_state = raw_state;

    if (stable_state != KEY_ACTIVE_STATE) {
        return;
    }

    if (flag_calibrando_linea) {
        Finalizar_Calibracion_Linea();
        currentMode = KEY_MODE_FL_BUSQUEDA_INICIAL;
    } else {
        Iniciar_Calibracion_Linea();
        currentMode = KEY_MODE_FL_INICIO;
    }

    screenScheduler();
}
