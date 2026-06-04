#ifndef MPU6050_APP_H
#define MPU6050_APP_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ADDR (0x68 << 1)
#define MPU6050_DATA_REG 0x3BU
#define MPU6050_DMA_BUFFER_SIZE 14U
#define MPU6050_DMA_TIMEOUT_MS 2U
#define MPU6050_STALE_TIMEOUT_MS 25U
#define MPU6050_MOTOR_READY_SAMPLES 3U
#define MPU6050_MAX_CONSECUTIVE_MISSES 3U

typedef enum {
    MPU_ACQ_IDLE = 0,
    MPU_ACQ_REQUEST_PENDING,
    MPU_ACQ_DMA_ACTIVE,
    MPU_ACQ_DATA_READY,
    MPU_ACQ_CONSUMING,
    MPU_ACQ_ERROR,
    MPU_ACQ_TIMEOUT
} MPU6050_AcqState_t;

extern I2C_HandleTypeDef hi2c1;
extern volatile uint8_t flagCalibrationIsReady;
extern float accel_bias_x;
extern float accel_bias_y;
extern float accel_bias_z;
extern float gyro_bias_x;
extern float gyro_bias_y;
extern float gyro_bias_z;
extern uint8_t mpu_data[MPU6050_DMA_BUFFER_SIZE];
extern volatile MPU6050_AcqState_t mpu_acq_state;
extern volatile uint8_t mpu_data_ready;
extern volatile uint8_t mpu_recovery_requested;
extern volatile uint8_t mpu_calibration_active;
extern volatile uint8_t mpu_filter_resync_required;
extern volatile uint32_t mpu_requested_slot;
extern volatile uint32_t mpu_active_slot;
extern volatile uint32_t mpu_ready_slot;
extern volatile uint32_t mpu_last_success_slot;
extern volatile uint32_t mpu_request_tick;
extern volatile uint32_t mpu_dma_start_tick;
extern volatile uint32_t mpu_last_valid_tick;
extern volatile uint32_t mpu_last_i2c_error;
extern volatile uint32_t mpu_dma_start_count;
extern volatile uint32_t mpu_dma_complete_count;
extern volatile uint32_t mpu_dma_error_count;
extern volatile uint32_t mpu_failed_sample_count;
extern volatile uint32_t mpu_stale_count;
extern volatile uint32_t mpu_hal_busy_count;
extern volatile uint32_t mpu_timeout_count;
extern volatile uint32_t mpu_i2c_recovery_count;
extern volatile uint32_t mpu_consecutive_valid_samples;
extern volatile uint32_t mpu_consecutive_missed_samples;
extern volatile uint32_t mpu_late_sample_count;

void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Calibrate(void);
void MPU6050_RequestSample(uint32_t slot, uint32_t now);
HAL_StatusTypeDef MPU6050_StartPendingDMA(I2C_HandleTypeDef *hi2c, uint32_t now);
void MPU6050_NotifyDmaComplete(void);
void MPU6050_NotifyI2CError(uint32_t error_code);
uint8_t MPU6050_CheckActiveTimeout(uint32_t now);
void MPU6050_DropPendingSample(void);
uint8_t MPU6050_TakeReadySample(uint32_t current_slot, uint32_t now);
void MPU6050_ReleaseConsumedSample(void);
void MPU6050_RecordMissedControlSample(void);
void MPU6050_MarkRecoveryComplete(void);
uint8_t MPU6050_IsBusPriorityActive(void);
uint8_t MPU6050_IsMotorEnableAllowed(uint32_t now);
void MPU6050_CalibrationProcessSample(const uint8_t *buffer);

#ifdef __cplusplus
}
#endif

#endif
