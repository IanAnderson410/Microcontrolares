#include "button_key.h"
#include "line_sensors.h"
#include "control_systems.h"

#define KEY_INACTIVE_STATE ((KEY_ACTIVE_STATE == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET)

volatile uint8_t key_stable_pressed = 0;
volatile uint8_t key_raw_pressed = 0;
volatile uint8_t key_click_pending = 0;
volatile uint8_t key_long_press_done = 0;
volatile uint8_t key_last_event = KEY_EVENT_NONE;
volatile uint8_t key_last_action = KEY_ACTION_NONE;
volatile uint32_t key_press_duration_ms = 0;
volatile uint32_t key_last_event_tick = 0;

void screenScheduler(void);

static void KEY_HandleSingleClick(void);
static void KEY_HandleDoubleClick(void);
static void KEY_HandleLongPress(void);

void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = KEY_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = KEY_GPIO_PULL;
    HAL_GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStruct);
}

void KEY_CalibrationTask(void)
{
    static uint8_t gpio_initialized = 0;
    static GPIO_PinState last_raw_state = KEY_INACTIVE_STATE;
    static GPIO_PinState stable_state = KEY_INACTIVE_STATE;
    static uint32_t last_change_tick = 0;
    static uint32_t press_start_tick = 0;
    static uint32_t first_click_tick = 0;
    static uint8_t click_pending = 0;
    static uint8_t long_press_handled = 0;

    uint32_t now = HAL_GetTick();
    if (!gpio_initialized) {
        KEY_Init();
        last_change_tick = now;
        gpio_initialized = 1;
    }

    GPIO_PinState raw_state = HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY_GPIO_PIN);
    key_raw_pressed = (raw_state == KEY_ACTIVE_STATE) ? 1U : 0U;
    key_stable_pressed = (stable_state == KEY_ACTIVE_STATE) ? 1U : 0U;
    key_click_pending = click_pending;
    key_long_press_done = long_press_handled;
    key_press_duration_ms = key_stable_pressed ? (uint32_t)(now - press_start_tick) : 0U;

    if (click_pending && stable_state != KEY_ACTIVE_STATE &&
        (uint32_t)(now - first_click_tick) >= KEY_DOUBLE_CLICK_MS) {
        click_pending = 0;
        key_click_pending = 0;
        KEY_HandleSingleClick();
    }

    if (raw_state != last_raw_state) {
        last_raw_state = raw_state;
        last_change_tick = now;
        return;
    }

    if ((now - last_change_tick) < KEY_DEBOUNCE_MS) {
        return;
    }

    if (raw_state == stable_state) {
        key_stable_pressed = (stable_state == KEY_ACTIVE_STATE) ? 1U : 0U;
        key_press_duration_ms = key_stable_pressed ? (uint32_t)(now - press_start_tick) : 0U;
        if (stable_state == KEY_ACTIVE_STATE && !long_press_handled &&
            (uint32_t)(now - press_start_tick) >= KEY_LONG_PRESS_MS) {
            click_pending = 0;
            long_press_handled = 1;
            key_click_pending = 0;
            key_long_press_done = 1;
            KEY_HandleLongPress();
        }
        return;
    }

    stable_state = raw_state;
    key_stable_pressed = (stable_state == KEY_ACTIVE_STATE) ? 1U : 0U;

    if (stable_state == KEY_ACTIVE_STATE) {
        press_start_tick = now;
        long_press_handled = 0;
        key_press_duration_ms = 0;
        key_long_press_done = 0;
    } else {
        uint32_t press_duration = now - press_start_tick;
        key_press_duration_ms = 0;

        if (!long_press_handled && press_duration >= KEY_LONG_PRESS_MS) {
            click_pending = 0;
            long_press_handled = 1;
            key_click_pending = 0;
            key_long_press_done = 1;
            KEY_HandleLongPress();
        } else if (!long_press_handled && press_duration < KEY_LONG_PRESS_MS) {
            if (click_pending &&
                (uint32_t)(now - first_click_tick) < KEY_DOUBLE_CLICK_MS) {
                click_pending = 0;
                key_click_pending = 0;
                KEY_HandleDoubleClick();
            } else {
                click_pending = 1;
                first_click_tick = now;
                key_click_pending = 1;
            }
        } else {
            click_pending = 0;
            key_click_pending = 0;
        }
    }

}

static void KEY_HandleSingleClick(void)
{
    key_last_event = KEY_EVENT_SHORT_PRESS;
    key_last_event_tick = HAL_GetTick();

    if (flag_calibrando_linea) {
        Finalizar_Calibracion_Linea();
        currentMode = CONTROL_MODE_FL_BUSQUEDA_INICIAL;
        key_last_action = KEY_ACTION_CAL_DONE;
    } else {
        Iniciar_Calibracion_Linea();
        currentMode = CONTROL_MODE_FL_INICIO;
        key_last_action = KEY_ACTION_CAL_START;
    }

    screenScheduler();
}

static void KEY_HandleDoubleClick(void)
{
    key_last_event = KEY_EVENT_DOUBLE_CLICK;
    key_last_event_tick = HAL_GetTick();

    if (currentMode >= CONTROL_MODE_FL_INICIO &&
        currentMode <= CONTROL_MODE_FL_INGRESO_A_90) {
        currentMode = CONTROL_MODE_RC;
        key_last_action = KEY_ACTION_MODE_RC;
    } else {
        currentMode = CONTROL_MODE_FL_BUSQUEDA_INICIAL;
        key_last_action = KEY_ACTION_MODE_FL;
    }

    screenScheduler();
}

static void KEY_HandleLongPress(void)
{
    key_last_event = KEY_EVENT_LONG_PRESS;
    key_last_event_tick = HAL_GetTick();

    Control_SetMotorsEnabled(flagMotorsAreOn ? 0U : 1U);
    key_last_action = flagMotorsAreOn ? KEY_ACTION_MOTORS_ON : KEY_ACTION_MOTORS_OFF;

    screenScheduler();
}
