#include "uner_protocol.h"
#include "control_systems.h"
#include "line_sensors.h"
#include <string.h>

#define UNER_CMD_ACK 1
#define UNER_CMD_DATA 17

typedef struct __attribute__((packed)) {
    int16_t     acc_x, acc_y, acc_z;
    int16_t     gyro_pitch, gyro_yaw;
    int16_t     pitch_cdeg;
    int16_t     roll_cdeg;
    int16_t     yaw_cdeg;
    int32_t     pos_x_mm;
    int32_t     pos_y_mm;
    int16_t     velocidad_mm_s;
    uint8_t     modo;
    uint16_t    IR[8];
    uint8_t     infoAdicional;
} PayloadDataV1_t;

extern volatile float giro_z;
extern float angle_roll;
extern float angle_yaw;
extern float error_linea;
extern float velocidad_objetivo;

uint16_t UNER_Crc16Ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

uint8_t UNER_SendV1(uint8_t cmd, uint8_t flags, const uint8_t *payload, uint8_t payload_len)
{
    static uint8_t seq = 0;
    uint8_t frame[4 + 1 + 1 + 1 + 1 + 1 + UNER_V1_MAX_PAYLOAD + 2];
    uint16_t idx = 0;

    if (payload_len > UNER_V1_MAX_PAYLOAD) {
        return 0;
    }

    frame[idx++] = 'U';
    frame[idx++] = 'N';
    frame[idx++] = 'E';
    frame[idx++] = 'R';
    frame[idx++] = UNER_V1_VERSION;
    frame[idx++] = cmd;
    frame[idx++] = flags;
    frame[idx++] = seq;
    frame[idx++] = payload_len;

    if (payload != NULL && payload_len > 0) {
        memcpy(&frame[idx], payload, payload_len);
        idx += payload_len;
    }

    uint16_t crc = UNER_Crc16Ccitt(frame, idx);
    frame[idx++] = (uint8_t)(crc & 0xFF);
    frame[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    if (UNER_QueueTx(frame, idx)) {
        seq++;
        uner_tx_ready_count++;
        return 1;
    }

    uner_tx_busy_count++;
    return 0;
}

uint8_t UNER_QueueTx(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || len > UNER_TX_FRAME_MAX) {
        return 0;
    }

    if (ESP.uner_tx_count >= UNER_TX_QUEUE_DEPTH) {
        return 0;
    }

    memcpy(ESP.uner_tx_frame[ESP.uner_tx_tail], data, len);
    ESP.uner_tx_len[ESP.uner_tx_tail] = (uint8_t)len;
    ESP.uner_tx_tail++;
    if (ESP.uner_tx_tail >= UNER_TX_QUEUE_DEPTH) {
        ESP.uner_tx_tail = 0;
    }
    ESP.uner_tx_count++;

    return 1;
}

void UNER_Tx_Task(void)
{
    uint8_t len;
    _eESP01STATUS status;
    uint32_t now = HAL_GetTick();

    if (ESP.uner_tx_count == 0 || !ESP.udp_connected) {
        return;
    }

    if (uner_tx_last_try_tick != 0 && (now - uner_tx_last_try_tick) < 20) {
        return;
    }

    if (uner_tx_last_try_tick != 0 && (now - uner_tx_last_try_tick) > UNER_TX_RECOVERY_MS) {
        ESP.uner_tx_head = 0;
        ESP.uner_tx_tail = 0;
        ESP.uner_tx_count = 0;
        ESP.udp_connected = 0;
        ESP.udp_started = 0;
        uner_tx_recover_count++;
        uner_tx_last_try_tick = now;
        ESP01_SetWIFI(WIFI_SSID, WIFI_PASSWORD);
        ESP01_StartTransport();
        ESP.udp_started = 1;
        return;
    }

    len = ESP.uner_tx_len[ESP.uner_tx_head];
    status = ESP01_Send(ESP.uner_tx_frame[ESP.uner_tx_head], 0, len, len);

    if (status == ESP01_SEND_READY) {
        uner_tx_last_try_tick = now;
        uner_tx_busy = 1;
        ESP.uner_tx_head++;
        if (ESP.uner_tx_head >= UNER_TX_QUEUE_DEPTH) {
            ESP.uner_tx_head = 0;
        }
        ESP.uner_tx_count--;
    } else if (status == ESP01_SEND_BUSY) {
        uner_tx_busy_count++;
    }
}

uint8_t UNER_SendAckV1(uint8_t acked_cmd, uint8_t acked_seq, uint8_t status)
{
    uint8_t payload[3];

    payload[0] = acked_cmd;
    payload[1] = acked_seq;
    payload[2] = status;

    return UNER_SendV1(UNER_CMD_ACK, UNER_V1_FLAG_ACK, payload, sizeof(payload));
}

uint8_t UNER_SendTelemetryV1(void)
{
    PayloadDataV1_t payload;

    payload.acc_x = (int16_t)accelx;
    payload.acc_y = (int16_t)accely;
    payload.acc_z = (int16_t)accelz;
    payload.gyro_pitch = (int16_t)giro;
    payload.gyro_yaw = (int16_t)giro_z;
    payload.pitch_cdeg = (int16_t)(angle_y * 100.0f);
    payload.roll_cdeg = (int16_t)(angle_roll * 100.0f);
    payload.yaw_cdeg = (int16_t)(angle_yaw * 100.0f);
    payload.pos_x_mm = (int32_t)(error_linea * 1000.0f);
    payload.pos_y_mm = (int32_t)RC_steering;
    payload.velocidad_mm_s = (int16_t)(velocidad_objetivo * 1000.0f);
    payload.modo = currentMode;
    payload.infoAdicional = (uint8_t)((flagCalibrationIsReady ? 0x01 : 0x00) |
                                      (flag_calibrando_linea ? 0x02 : 0x00) |
                                      ((estado_sensores[0] & 0x01) << 2) |
                                      ((estado_sensores[1] & 0x01) << 3) |
                                      ((estado_sensores[2] & 0x01) << 4) |
                                      ((estado_sensores[3] & 0x01) << 5));

    for (uint8_t i = 0; i < 8; i++) {
        payload.IR[i] = adc_filtrado[i];
    }

    return UNER_SendV1(UNER_CMD_DATA, 0, (const uint8_t *)&payload, sizeof(payload));
}

uint8_t UNER_Send(const uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
    uint8_t frame[4 + 1 + 1 + 1 + 64 + 1];
    uint16_t idx = 0;

    if (payload_len > 64) {
        return 0;
    }

    frame[idx++] = 'U';
    frame[idx++] = 'N';
    frame[idx++] = 'E';
    frame[idx++] = 'R';
    frame[idx++] = (uint8_t)(1 + payload_len + 1);
    frame[idx++] = UNER_TOKEN;
    frame[idx++] = cmd;

    if (payload != NULL && payload_len > 0) {
        memcpy(&frame[idx], payload, payload_len);
        idx += payload_len;
    }

    uint8_t checksum = 0;
    for (uint16_t i = 0; i < idx; i++) {
        checksum ^= frame[i];
    }

    frame[idx++] = checksum;

    return (ESP01_Send(frame, 0, idx, idx) == ESP01_SEND_READY);
}

uint8_t UNER_SendInt16(uint8_t cmd, int16_t value)
{
    uint8_t payload[2];

    payload[0] = (uint8_t)(value & 0xFF);
    payload[1] = (uint8_t)((value >> 8) & 0xFF);

    return UNER_SendV1(cmd, 0, payload, 2);
}

void UNER_Rx_Task(void)
{
    while (ESP.uner_rx_read != ESP.uner_rx_write) {
        uint8_t b = ESP.uner_rx_ring[ESP.uner_rx_read];

        ESP.uner_rx_read++;

        if (ESP.uner_rx_read >= UNER_RX_RING_SIZE) {
            ESP.uner_rx_read = 0;
        }

        UNER_ProcessByteV1(b);
    }
}

void UNER_ProcessByteV1(uint8_t b)
{
    typedef enum {
        UNER_V1_ST_U,
        UNER_V1_ST_N,
        UNER_V1_ST_E,
        UNER_V1_ST_R,
        UNER_V1_ST_VERSION,
        UNER_V1_ST_CMD,
        UNER_V1_ST_FLAGS,
        UNER_V1_ST_SEQ,
        UNER_V1_ST_LEN,
        UNER_V1_ST_PAYLOAD,
        UNER_V1_ST_CRC0,
        UNER_V1_ST_CRC1
    } UNER_V1_State_t;

    static UNER_V1_State_t st = UNER_V1_ST_U;
    static uint8_t frame[4 + 1 + 1 + 1 + 1 + 1 + UNER_V1_MAX_PAYLOAD];
    static uint8_t idx = 0;
    static uint8_t payload_len = 0;
    static uint8_t payload_idx = 0;
    static uint8_t crc0 = 0;

    switch (st) {
    case UNER_V1_ST_U:
        if (b == 'U') {
            idx = 0;
            frame[idx++] = b;
            st = UNER_V1_ST_N;
        }
        break;
    case UNER_V1_ST_N:
        if (b == 'N') {
            frame[idx++] = b;
            st = UNER_V1_ST_E;
        } else {
            st = UNER_V1_ST_U;
        }
        break;
    case UNER_V1_ST_E:
        if (b == 'E') {
            frame[idx++] = b;
            st = UNER_V1_ST_R;
        } else {
            st = UNER_V1_ST_U;
        }
        break;
    case UNER_V1_ST_R:
        if (b == 'R') {
            frame[idx++] = b;
            st = UNER_V1_ST_VERSION;
        } else {
            st = UNER_V1_ST_U;
        }
        break;
    case UNER_V1_ST_VERSION:
        if (b == UNER_V1_VERSION) {
            frame[idx++] = b;
            st = UNER_V1_ST_CMD;
        } else {
            st = UNER_V1_ST_U;
        }
        break;
    case UNER_V1_ST_CMD:
        frame[idx++] = b;
        st = UNER_V1_ST_FLAGS;
        break;
    case UNER_V1_ST_FLAGS:
        frame[idx++] = b;
        st = UNER_V1_ST_SEQ;
        break;
    case UNER_V1_ST_SEQ:
        frame[idx++] = b;
        st = UNER_V1_ST_LEN;
        break;
    case UNER_V1_ST_LEN:
        payload_len = b;
        if (payload_len > UNER_V1_MAX_PAYLOAD) {
            st = UNER_V1_ST_U;
            break;
        }

        frame[idx++] = b;
        payload_idx = 0;
        st = (payload_len == 0) ? UNER_V1_ST_CRC0 : UNER_V1_ST_PAYLOAD;
        break;
    case UNER_V1_ST_PAYLOAD:
        frame[idx++] = b;
        payload_idx++;
        if (payload_idx >= payload_len) {
            st = UNER_V1_ST_CRC0;
        }
        break;
    case UNER_V1_ST_CRC0:
        crc0 = b;
        st = UNER_V1_ST_CRC1;
        break;
    case UNER_V1_ST_CRC1:
    {
        uint16_t crc_rx = (uint16_t)crc0 | ((uint16_t)b << 8);
        uint16_t crc_calc = UNER_Crc16Ccitt(frame, idx);

        if (crc_rx == crc_calc) {
            uint8_t cmd = frame[5];
            uint8_t flags = frame[6];
            uint8_t seq = frame[7];
            uint8_t *payload = &frame[9];

            UNER_HandlePacket(cmd, flags, seq, payload, payload_len);
        }

        st = UNER_V1_ST_U;
        break;
    }
    default:
        st = UNER_V1_ST_U;
        break;
    }
}

void UNER_ProcessByte(uint8_t b)
{
    typedef enum {
        UNER_ST_U,
        UNER_ST_N,
        UNER_ST_E,
        UNER_ST_R,
        UNER_ST_LEN,
        UNER_ST_TOKEN,
        UNER_ST_BODY
    } UNER_State_t;

    static UNER_State_t st = UNER_ST_U;
    static uint8_t len = 0;
    static uint8_t idx = 0;
    static uint8_t body[1 + 64 + 1];

    switch (st) {
    case UNER_ST_U:
        if (b == 'U') st = UNER_ST_N;
        break;
    case UNER_ST_N:
        st = (b == 'N') ? UNER_ST_E : UNER_ST_U;
        break;
    case UNER_ST_E:
        st = (b == 'E') ? UNER_ST_R : UNER_ST_U;
        break;
    case UNER_ST_R:
        st = (b == 'R') ? UNER_ST_LEN : UNER_ST_U;
        break;
    case UNER_ST_LEN:
        len = b;
        if (len < 2 || len > sizeof(body)) {
            st = UNER_ST_U;
        } else {
            idx = 0;
            st = UNER_ST_TOKEN;
        }
        break;
    case UNER_ST_TOKEN:
        st = (b == UNER_TOKEN) ? UNER_ST_BODY : UNER_ST_U;
        break;
    case UNER_ST_BODY:
        body[idx++] = b;
        if (idx >= len) {
            uint8_t checksum = 0;

            checksum ^= 'U';
            checksum ^= 'N';
            checksum ^= 'E';
            checksum ^= 'R';
            checksum ^= len;
            checksum ^= UNER_TOKEN;

            for (uint8_t i = 0; i < (len - 1); i++) {
                checksum ^= body[i];
            }

            if (checksum == body[len - 1]) {
                uint8_t cmd = body[0];
                uint8_t *payload = &body[1];
                uint8_t payload_len = len - 2;

                UNER_HandlePacket(cmd, 0, 0, payload, payload_len);
            }

            st = UNER_ST_U;
        }
        break;
    default:
        st = UNER_ST_U;
        break;
    }
}

int16_t UNER_ReadInt16LE(const uint8_t *payload)
{
    return (int16_t)((uint16_t)payload[0] | ((uint16_t)payload[1] << 8));
}

float UNER_ReadFloatLE(const uint8_t *payload)
{
    uint32_t raw = (uint32_t)payload[0] |
                   ((uint32_t)payload[1] << 8) |
                   ((uint32_t)payload[2] << 16) |
                   ((uint32_t)payload[3] << 24);
    float value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

void sendCMD(uint8_t cmd, uint16_t param)
{
    UNER_SendInt16(cmd, (int16_t)param);
}
