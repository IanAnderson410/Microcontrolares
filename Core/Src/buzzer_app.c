#include "buzzer_app.h"

void buzzerSecuence(Buzzer_Seq_t *seq)
{
    // Ejecuta una secuencia no bloqueante: el loop principal sigue corriendo
    // mientras se alterna el pin del buzzer por tiempo.
    if (seq->repeat == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        return;
    }

    uint32_t current_tick = HAL_GetTick();

    if (seq->state == 0 && (current_tick - seq->last_tick >= seq->interval)) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        seq->state = 1;
        seq->last_tick = current_tick;
    } else if (seq->state == 1 && (current_tick - seq->last_tick >= seq->duration)) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        seq->state = 0;
        seq->last_tick = current_tick;
        seq->repeat--;
    }
}

void BS_tcpConnectSecuence(void)
{
    // Dos pulsos cortos: enlace de comunicacion disponible.
    hBuzzer.duration = 100;
    hBuzzer.interval = 50;
    hBuzzer.repeat = 2;
    hBuzzer.state = 0;
    hBuzzer.last_tick = HAL_GetTick();
}

void BS_Error(void)
{
    // Pulso largo: condicion de error general.
    hBuzzer.duration = 500;
    hBuzzer.interval = 100;
    hBuzzer.repeat = 1;
    hBuzzer.state = 0;
    hBuzzer.last_tick = HAL_GetTick();
}

void BS_ACK_NOT_FOUND(void)
{
    // Tres pulsos: no se recibio/confirmo un ACK esperado.
    hBuzzer.duration = 200;
    hBuzzer.interval = 50;
    hBuzzer.repeat = 3;
    hBuzzer.state = 0;
    hBuzzer.last_tick = HAL_GetTick();
}

void BS_NEWPARAM_OK(void)
{
    // Pulso breve: parametro aceptado.
    hBuzzer.duration = 80;
    hBuzzer.interval = 50;
    hBuzzer.repeat = 1;
    hBuzzer.state = 0;
    hBuzzer.last_tick = HAL_GetTick();
}

void BS_NEWPARAM_ISNOTOK(void)
{
    // Pulso sostenido: parametro rechazado o fuera de rango.
    hBuzzer.duration = 800;
    hBuzzer.interval = 1;
    hBuzzer.repeat = 1;
    hBuzzer.state = 0;
    hBuzzer.last_tick = HAL_GetTick();
}
