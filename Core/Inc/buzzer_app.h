#ifndef BUZZER_APP_H
#define BUZZER_APP_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t duration;
    uint16_t interval;
    uint8_t repeat;
    uint32_t last_tick;
    uint8_t state;
} Buzzer_Seq_t;

extern Buzzer_Seq_t hBuzzer;

void buzzerSecuence(Buzzer_Seq_t *seq);
void BS_tcpConnectSecuence(void);
void BS_Error(void);
void BS_ACK_NOT_FOUND(void);
void BS_NEWPARAM_OK(void);
void BS_NEWPARAM_ISNOTOK(void);

#ifdef __cplusplus
}
#endif

#endif
