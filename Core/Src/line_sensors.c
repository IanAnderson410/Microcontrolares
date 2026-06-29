#include "line_sensors.h"

// Suaviza las lecturas ADC de los sensores IR antes de usarlas en control.
void Filtrar_Sensores_IR(void)
{
    for (int i = 0; i < 8; i++) {
        uint16_t adc_sample = adc_buffer[i];
        adc_filtrado[i] = (adc_filtrado[i] * 7 + adc_sample * 3) / 10;
    }
}

// Detecta presencia de obstaculo frontal con umbrales derivados de una linea base.
void ObstacleSensor_Task(void)
{
    static uint8_t baseline_ready = 0;
    static uint8_t obstacle_confirm_count = 0;
    static uint8_t obstacle_clear_count = 0;

    obstacle_ir_raw = adc_buffer[OBSTACLE_SENSOR_ADC_INDEX];

    if (!baseline_ready) {
        obstacle_ir_filtered = obstacle_ir_raw;
        obstacle_ir_baseline = obstacle_ir_raw;
        baseline_ready = 1U;
    } else {
        obstacle_ir_filtered = (uint16_t)(((uint32_t)obstacle_ir_filtered * 7U +
                                           (uint32_t)obstacle_ir_raw * 3U) / 10U);
    }

    obstacle_ir_enter_threshold = obstacle_ir_baseline + OBSTACLE_ENTER_DELTA;
    obstacle_ir_exit_threshold = obstacle_ir_baseline + OBSTACLE_EXIT_DELTA;

    if (!obstacle_detected) {
        if (obstacle_ir_filtered < obstacle_ir_exit_threshold) {
            obstacle_ir_baseline = (uint16_t)(((uint32_t)obstacle_ir_baseline * 63U +
                                               (uint32_t)obstacle_ir_filtered) / 64U);
        }

        if (obstacle_ir_filtered > obstacle_ir_enter_threshold) {
            if (obstacle_confirm_count < OBSTACLE_CONFIRM_SAMPLES) {
                obstacle_confirm_count++;
            }
        } else {
            obstacle_confirm_count = 0;
        }

        if (obstacle_confirm_count >= OBSTACLE_CONFIRM_SAMPLES) {
            obstacle_detected = 1U;
            obstacle_event_pending = 1U;
            obstacle_clear_count = 0;
        }
    } else {
        if (obstacle_ir_filtered < obstacle_ir_exit_threshold) {
            if (obstacle_clear_count < OBSTACLE_CLEAR_SAMPLES) {
                obstacle_clear_count++;
            }
        } else {
            obstacle_clear_count = 0;
        }

        if (obstacle_clear_count >= OBSTACLE_CLEAR_SAMPLES) {
            obstacle_detected = 0U;
            obstacle_confirm_count = 0;
        }
    }
}

uint8_t ObstacleSensor_ConsumeEvent(void)
{
    if (!obstacle_event_pending) {
        return 0U;
    }

    obstacle_event_pending = 0U;
    return 1U;
}

// Inicia la captura de minimos y maximos para calibrar los sensores de linea.
void Iniciar_Calibracion_Linea(void)
{
    flag_calibrando_linea = 1;
    flagCalibrationIsReady = 0;

    for (int i = 0; i < FL_LINE_SENSOR_COUNT; i++) {
        sensor_min[i] = 4095;
        sensor_max[i] = 0;
    }
}

// Actualiza extremos observados durante el recorrido de calibracion sobre la pista.
void Procesar_Calibracion_Linea(void)
{
    if (flag_calibrando_linea) {
        for (int i = 0; i < FL_LINE_SENSOR_COUNT; i++) {
            if (adc_filtrado[i] < sensor_min[i]) {
                sensor_min[i] = adc_filtrado[i];
            }

            if (adc_filtrado[i] > sensor_max[i]) {
                sensor_max[i] = adc_filtrado[i];
            }
        }
    }
}

// Convierte valores filtrados a estados digitales y actualiza el error lateral.
void Leer_Linea_Digital(void)
{
    for (int i = 0; i < FL_LINE_SENSOR_COUNT; i++) {
        estado_sensores[i] = (adc_filtrado[i] > sensor_threshold[i]) ? 1U : 0U;
    }

    if (AIRAB) {
        ultimo_estado_sensores[0] = estado_sensores[0];
        ultimo_estado_sensores[1] = estado_sensores[1];
        ultimo_estado_sensores[2] = estado_sensores[2];
        ultimo_estado_sensores[3] = estado_sensores[3];
        error_linea = calcularErrorYawContinuo();
        last_state_linea = error_linea;
    } else {
        error_linea = 0.0f;
    }
}

// Calcula umbrales finales de linea a partir de los extremos medidos.
void Finalizar_Calibracion_Linea(void)
{
    static const uint16_t fallback_threshold[FL_LINE_SENSOR_COUNT] = {
        (806 + 3419) / 2,
        (283 + 3213) / 2,
        (339 + 3456) / 2,
        2048
    };

    flag_calibrando_linea = 0;

    for (int i = 0; i < FL_LINE_SENSOR_COUNT; i++) {
        if ((sensor_max[i] > sensor_min[i]) && ((sensor_max[i] - sensor_min[i]) > FL_LINE_MIN_SPREAD)) {
            sensor_threshold[i] = (sensor_max[i] + sensor_min[i]) / 2;
        } else {
            sensor_threshold[i] = fallback_threshold[i];
        }
    }

    flagCalibrationIsReady = 1;
}

float calcularErrorYawDiscreto(void)
{
    float numerador = (estado_sensores[3] * -2.5f) +
                      (estado_sensores[2] * -1.2f) +
                      (estado_sensores[1] * 1.2f) +
                      (estado_sensores[0] * 2.5f);
    float denominador = estado_sensores[3] + estado_sensores[2] +
                        estado_sensores[1] + estado_sensores[0];

    if (denominador == 0.0f) {
        return 0.0f;
    }

    return numerador / denominador;
}

// Estima el desplazamiento lateral usando la intensidad relativa de cada sensor.
float calcularErrorYawContinuo(void)
{
    static const float sensor_pos[FL_LINE_SENSOR_COUNT] = {
        2.5f,
        1.2f,
        -1.2f,
        -2.5f
    };
    float val_norm[FL_LINE_SENSOR_COUNT] = {0};
    float numerador = 0.0f;
    float denominador = 0.0f;

    for (int i = 0; i < FL_LINE_SENSOR_COUNT; i++) {
        if (sensor_max[i] == sensor_min[i]) {
            val_norm[i] = 0.0f;
            continue;
        }

        float rango = (float)(sensor_max[i] - sensor_min[i]);
        val_norm[i] = (float)(adc_filtrado[i] - sensor_min[i]) / rango;

        if (val_norm[i] > 1.0f) {
            val_norm[i] = 1.0f;
        }
        if (val_norm[i] < 0.0f) {
            val_norm[i] = 0.0f;
        }

        numerador += val_norm[i] * sensor_pos[i];
        denominador += val_norm[i];
    }

    float offset = 0.0f;
    if (denominador > 0.0f) {
        offset = -((numerador / denominador) / 2.5f);
    }
    last_state_linea = offset;
    return offset;
}
