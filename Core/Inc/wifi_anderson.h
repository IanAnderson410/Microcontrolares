/*
 * wifi_anderson.h
 *
 *  Created on: 5 may 2026
 *      Author: ianan
 */
#include <stdbool.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

#ifndef INC_WIFI_ANDERSON_H_
#define INC_WIFI_ANDERSON_H_

/* Definiciones de tiempos y seguridad */
#define TIMEOUT_ALIVE_MS    1500  // Tiempo máximo sin recibir ALIVE antes de apagar

/* Estructura principal del estado del robot */
typedef struct {
    float 		setpoint_angulo;
    bool 		motores_habilitados;
    uint16_t 	alive_counter_ms;
    bool 	conexion_perdida;

    /* Variables para el manejo del DMA Circular */
    UART_HandleTypeDef *huart_ptr;
    uint8_t 			rx_buffer[256];
    uint16_t			 read_ptr;
} RobotState_t;

/* Prototipos de funciones */
void WIFI_Init(RobotState_t *robot, UART_HandleTypeDef *huart);
void WIFI_UpdateAlive(RobotState_t *robot, uint16_t dt_ms);
void WIFI_ResetAlive(RobotState_t *robot);
void WIFI_CheckRxBuffer(RobotState_t *robot);


#endif /* INC_WIFI_ANDERSON_H_ */
