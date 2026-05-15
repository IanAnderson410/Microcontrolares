#ifndef UNER_PROTOCOL_H
#define UNER_PROTOCOL_H

#include "main.h"
#include "ESP01.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UNER_HEADER_STR          "UNER"
#define UNER_TOKEN               ':'
#define UNER_RX_RING_SIZE        ESP01_UNER_RX_RING_SIZE
#define UNER_TX_QUEUE_DEPTH      ESP01_UNER_TX_QUEUE_DEPTH
#define UNER_TX_FRAME_MAX        ESP01_UNER_TX_FRAME_MAX
#define UNER_TELEMETRY_PERIOD_MS 100
#define UNER_TX_RECOVERY_MS      1000
#define UNER_V1_VERSION          1
#define UNER_V1_MAX_PAYLOAD      64
#define UNER_V1_FLAG_ACK_REQUIRED 0x01
#define UNER_V1_FLAG_ACK          0x02

extern ESP01_App_t ESP;
extern volatile uint8_t uner_tx_busy;
extern volatile uint32_t uner_tx_ready_count;
extern volatile uint32_t uner_tx_busy_count;
extern volatile uint32_t uner_tx_ok_count;
extern volatile uint32_t uner_tx_last_try_tick;
extern volatile uint32_t uner_tx_last_ok_tick;
extern volatile uint32_t uner_tx_recover_count;
extern volatile uint8_t uner_recovering_udp;
extern volatile uint32_t uner_next_telemetry_tick;

uint16_t UNER_Crc16Ccitt(const uint8_t *data, uint16_t len);
uint8_t UNER_SendV1(uint8_t cmd, uint8_t flags, const uint8_t *payload, uint8_t payload_len);
uint8_t UNER_QueueTx(const uint8_t *data, uint16_t len);
void UNER_Tx_Task(void);
uint8_t UNER_SendAckV1(uint8_t acked_cmd, uint8_t acked_seq, uint8_t status);
uint8_t UNER_SendTelemetryV1(void);
uint8_t UNER_Send(const uint8_t cmd, const uint8_t *payload, uint8_t payload_len);
uint8_t UNER_SendInt16(uint8_t cmd, int16_t value);
void UNER_Rx_Task(void);
void UNER_ProcessByteV1(uint8_t b);
void UNER_ProcessByte(uint8_t b);
int16_t UNER_ReadInt16LE(const uint8_t *payload);
float UNER_ReadFloatLE(const uint8_t *payload);
void UNER_HandlePacket(uint8_t cmd, uint8_t flags, uint8_t seq, uint8_t *payload, uint8_t payload_len);
void sendCMD(uint8_t cmd, uint16_t param);

#ifdef __cplusplus
}
#endif

#endif
