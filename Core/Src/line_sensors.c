#include "line_sensors.h"

void Filtrar_Sensores_IR(void)
{
    for (int i = 0; i < 8; i++) {
        adc_filtrado[i] = (adc_filtrado[i] * 9 + adc_buffer[i]) / 10;
    }
}

void Iniciar_Calibracion_Linea(void)
{
    flag_calibrando_linea = 1;
    flagCalibrationIsReady = 0;

    for (int i = 0; i < FL_LINE_SENSOR_COUNT; i++) {
        sensor_min[i] = 4095;
        sensor_max[i] = 0;
    }
}

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

void Leer_Linea_Digital(void)
{
    uint16_t min_value = adc_filtrado[0];
    uint16_t max_value = adc_filtrado[0];

    for (int i = 1; i < FL_LINE_SENSOR_COUNT; i++) {
        if (adc_filtrado[i] < min_value) {
            min_value = adc_filtrado[i];
        }
        if (adc_filtrado[i] > max_value) {
            max_value = adc_filtrado[i];
        }
    }

    if ((uint16_t)(max_value - min_value) < FL_LINE_MIN_SPREAD) {
        for (int i = 0; i < FL_LINE_SENSOR_COUNT; i++) {
            estado_sensores[i] = 0;
        }
        return;
    }

    uint16_t mid_value = (uint16_t)((max_value + min_value) / 2U);
    uint8_t high_count = 0;

    for (int i = 0; i < FL_LINE_SENSOR_COUNT; i++) {
        if (adc_filtrado[i] >= mid_value) {
            high_count++;
        }
    }

    uint8_t low_count = FL_LINE_SENSOR_COUNT - high_count;
    uint8_t line_is_high = (high_count <= low_count) ? 1U : 0U;

    for (int i = 0; i < FL_LINE_SENSOR_COUNT; i++) {
        if (line_is_high) {
            estado_sensores[i] = (adc_filtrado[i] >= mid_value) ? 1U : 0U;
        } else {
            estado_sensores[i] = (adc_filtrado[i] < mid_value) ? 1U : 0U;
        }
    }
}

void Finalizar_Calibracion_Linea(void)
{
    static const uint16_t fallback_threshold[FL_LINE_SENSOR_COUNT] = {
        (806 + 3419) / 2,
        (283 + 3213) / 2,
        (339 + 3456) / 2,
        (1280 + 3634) / 2
    };

    flag_calibrando_linea = 0;

    for (int i = 0; i < FL_LINE_SENSOR_COUNT; i++) {
        if ((sensor_max[i] > sensor_min[i]) && ((sensor_max[i] - sensor_min[i]) > 80)) {
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

float calcularErrorYawContinuo(void)
{
    float val_norm[3] = {0};

    for (int i = 0; i < 3; i++) {
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
    }

    float y_izq = val_norm[2];
    float y_centro = val_norm[1];
    float y_der = val_norm[0];

    float denominador = (y_izq + y_der - 2.0f * y_centro);
    float offset = 0.0f;
    if (denominador != 0.0f) {
        offset = (y_izq - y_der) / denominador;
    }

    last_state_linea = offset;
    return offset;
}
