/*
 * wifi_anderson.c
 *
 *  Created on: 5 may 2026
 *      Author: ianan
 */


#include "wifi_anderson.h"

/* Inicializa la estructura y arranca el DMA en modo circular */
void WIFI_Init(RobotState_t *robot, UART_HandleTypeDef *huart) {
    robot->huart_ptr = huart;
    robot->setpoint_angulo = 0.0f;
    robot->motores_habilitados = false;
    robot->alive_counter_ms = 0;
    robot->conexion_perdida = true; // Asumimos desconectado hasta recibir el primer ALIVE
    robot->read_ptr = 0;
    // Arrancamos la recepción por DMA de forma circular
    HAL_UART_Receive_DMA(robot->huart_ptr, robot->rx_buffer, sizeof(robot->rx_buffer));
}

/* Incrementa el contador de timeout. Debe llamarse cada dt_ms (ej. 10 ms en tu TIM4) */
void WIFI_UpdateAlive(RobotState_t *robot, uint16_t dt_ms) {
    robot->alive_counter_ms += dt_ms;
    if (robot->alive_counter_ms >= TIMEOUT_ALIVE_MS) {
        // Se perdió la comunicación. Acción de seguridad crítica:
        robot->conexion_perdida = true;
        robot->setpoint_angulo = 0.0f;       // Forzamos el robot a equilibrarse en el lugar
        // robot->motores_habilitados = false; // Opcional: apagar motores (depende de tu estrategia de caída)
        // Evitamos que el contador se desborde
        robot->alive_counter_ms = TIMEOUT_ALIVE_MS;
    }
}

/* Se llama cuando parseamos exitosamente un comando ALIVE desde el buffer */
void WIFI_ResetAlive(RobotState_t *robot) {
    robot->alive_counter_ms = 0;
    robot->conexion_perdida = false;
}

/* Revisa el buffer circular buscando nuevos datos. Se llama en el main() */
void WIFI_CheckRxBuffer(RobotState_t *robot) {
    // Calculamos dónde está escribiendo el DMA actualmente
    uint16_t write_ptr = sizeof(robot->rx_buffer) - __HAL_DMA_GET_COUNTER(robot->huart_ptr->hdmarx);

    while (robot->read_ptr != write_ptr) {
        uint8_t incoming_byte = robot->rx_buffer[robot->read_ptr];

        /*
         * AQUÍ AGREGAREMOS LA MÁQUINA DE ESTADOS DEL PARSER
         * Buscaremos la cabecera 'UNER', verificaremos el Checksum
         * y leeremos el comando.
         */

        // Simulación temporal: si recibimos el byte 'A' (por ejemplo), es un ALIVE
        if (incoming_byte == 'A') {
            WIFI_ResetAlive(robot);
        }

        // Avanzamos el puntero de lectura de forma circular
        robot->read_ptr++;
        if (robot->read_ptr >= sizeof(robot->rx_buffer)) {
            robot->read_ptr = 0;
        }
    }
}
