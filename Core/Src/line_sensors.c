#include "line_sensors.h"

void Filtrar_Sensores_IR(void)
{
    for (int i = 0; i < FL_LINE_SENSOR_COUNT; i++) {
        uint16_t adc_sample = adc_buffer[i];
        adc_filtrado[i] = (adc_filtrado[i] * 7 + adc_sample * 3) / 10;
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
