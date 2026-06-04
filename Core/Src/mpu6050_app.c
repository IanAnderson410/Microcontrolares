#include "mpu6050_app.h"

uint8_t mpu_data[MPU6050_DMA_BUFFER_SIZE];
volatile MPU6050_AcqState_t mpu_acq_state = MPU_ACQ_IDLE;
volatile uint8_t mpu_data_ready = 0;
volatile uint8_t mpu_recovery_requested = 0;
volatile uint8_t mpu_calibration_active = 0;
volatile uint8_t mpu_filter_resync_required = 1;
volatile uint32_t mpu_requested_slot = 0;
volatile uint32_t mpu_active_slot = 0;
volatile uint32_t mpu_ready_slot = 0;
volatile uint32_t mpu_last_success_slot = 0;
volatile uint32_t mpu_request_tick = 0;
volatile uint32_t mpu_dma_start_tick = 0;
volatile uint32_t mpu_last_valid_tick = 0;
volatile uint32_t mpu_last_i2c_error = HAL_I2C_ERROR_NONE;
volatile uint32_t mpu_dma_start_count = 0;
volatile uint32_t mpu_dma_complete_count = 0;
volatile uint32_t mpu_dma_error_count = 0;
volatile uint32_t mpu_failed_sample_count = 0;
volatile uint32_t mpu_stale_count = 0;
volatile uint32_t mpu_hal_busy_count = 0;
volatile uint32_t mpu_timeout_count = 0;
volatile uint32_t mpu_i2c_recovery_count = 0;
volatile uint32_t mpu_consecutive_valid_samples = 0;
volatile uint32_t mpu_consecutive_missed_samples = 0;
volatile uint32_t mpu_late_sample_count = 0;

static int32_t calibration_ax_sum = 0;
static int32_t calibration_ay_sum = 0;
static int32_t calibration_az_sum = 0;
static int32_t calibration_gy_sum = 0;
static int32_t calibration_gz_sum = 0;
static uint16_t calibration_sample_count = 0;

void MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t check, data;

    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x75, 1, &check, 1, 100);
    if (check == 0x68) {
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x6B, 1, &data, 1, 100);

        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1C, 1, &data, 1, 100);

        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1B, 1, &data, 1, 100);

        data = 0x02;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1A, 1, &data, 1, 100);

        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x19, 1, &data, 1, 100);
    }
}

void MPU6050_Calibrate(void)
{
    calibration_ax_sum = 0;
    calibration_ay_sum = 0;
    calibration_az_sum = 0;
    calibration_gy_sum = 0;
    calibration_gz_sum = 0;
    calibration_sample_count = 0;
    flagCalibrationIsReady = 0;
    mpu_calibration_active = 1;
}

void MPU6050_RequestSample(uint32_t slot, uint32_t now)
{
    if (mpu_acq_state == MPU_ACQ_IDLE) {
        mpu_requested_slot = slot;
        mpu_request_tick = now;
        mpu_acq_state = MPU_ACQ_REQUEST_PENDING;
    } else {
        mpu_failed_sample_count++;
        mpu_stale_count++;
        mpu_consecutive_valid_samples = 0;
    }
}

HAL_StatusTypeDef MPU6050_StartPendingDMA(I2C_HandleTypeDef *hi2c, uint32_t now)
{
    HAL_StatusTypeDef status;

    if (mpu_acq_state != MPU_ACQ_REQUEST_PENDING) {
        return HAL_BUSY;
    }

    status = HAL_I2C_Mem_Read_DMA(hi2c,
                                  MPU6050_ADDR,
                                  MPU6050_DATA_REG,
                                  I2C_MEMADD_SIZE_8BIT,
                                  mpu_data,
                                  MPU6050_DMA_BUFFER_SIZE);
    if (status == HAL_OK) {
        mpu_active_slot = mpu_requested_slot;
        mpu_dma_start_tick = now;
        mpu_dma_start_count++;
        mpu_acq_state = MPU_ACQ_DMA_ACTIVE;
    } else if (status == HAL_BUSY) {
        mpu_hal_busy_count++;
    } else {
        mpu_last_i2c_error = HAL_I2C_GetError(hi2c);
        mpu_dma_error_count++;
        mpu_failed_sample_count++;
        mpu_consecutive_valid_samples = 0;
        mpu_recovery_requested = 1;
        mpu_acq_state = MPU_ACQ_ERROR;
    }

    return status;
}

void MPU6050_NotifyDmaComplete(void)
{
    if (mpu_acq_state == MPU_ACQ_DMA_ACTIVE) {
        mpu_ready_slot = mpu_active_slot;
        mpu_data_ready = 1;
        mpu_dma_complete_count++;
        mpu_acq_state = MPU_ACQ_DATA_READY;
    }
}

void MPU6050_NotifyI2CError(uint32_t error_code)
{
    mpu_last_i2c_error = error_code;
    mpu_dma_error_count++;
    mpu_failed_sample_count++;
    mpu_consecutive_valid_samples = 0;
    mpu_data_ready = 0;
    mpu_recovery_requested = 1;
    mpu_acq_state = MPU_ACQ_ERROR;
}

uint8_t MPU6050_CheckActiveTimeout(uint32_t now)
{
    if (mpu_acq_state == MPU_ACQ_DMA_ACTIVE &&
        (uint32_t)(now - mpu_dma_start_tick) >= MPU6050_DMA_TIMEOUT_MS) {
        mpu_timeout_count++;
        mpu_failed_sample_count++;
        mpu_consecutive_valid_samples = 0;
        mpu_data_ready = 0;
        mpu_recovery_requested = 1;
        mpu_acq_state = MPU_ACQ_TIMEOUT;
        return 1U;
    }
    return 0U;
}

void MPU6050_DropPendingSample(void)
{
    if (mpu_acq_state == MPU_ACQ_REQUEST_PENDING) {
        mpu_failed_sample_count++;
        mpu_stale_count++;
        mpu_late_sample_count++;
        mpu_acq_state = MPU_ACQ_IDLE;
    }
}

uint8_t MPU6050_TakeReadySample(uint32_t current_slot, uint32_t now)
{
    if (mpu_acq_state != MPU_ACQ_DATA_READY || !mpu_data_ready) {
        return 0U;
    }

    if (mpu_ready_slot != current_slot) {
        mpu_data_ready = 0;
        mpu_acq_state = MPU_ACQ_IDLE;
        mpu_late_sample_count++;
        return 0U;
    }

    mpu_data_ready = 0;
    mpu_acq_state = MPU_ACQ_CONSUMING;
    mpu_last_success_slot = mpu_ready_slot;
    mpu_last_valid_tick = now;
    mpu_consecutive_missed_samples = 0;
    if (mpu_consecutive_valid_samples < UINT32_MAX) {
        mpu_consecutive_valid_samples++;
    }
    return 1U;
}

void MPU6050_ReleaseConsumedSample(void)
{
    if (mpu_acq_state == MPU_ACQ_CONSUMING) {
        mpu_acq_state = MPU_ACQ_IDLE;
    }
}

void MPU6050_RecordMissedControlSample(void)
{
    mpu_failed_sample_count++;
    mpu_stale_count++;
    mpu_consecutive_valid_samples = 0;
    if (mpu_consecutive_missed_samples < UINT32_MAX) {
        mpu_consecutive_missed_samples++;
    }
}

void MPU6050_MarkRecoveryComplete(void)
{
    mpu_i2c_recovery_count++;
    mpu_recovery_requested = 0;
    mpu_data_ready = 0;
    mpu_consecutive_valid_samples = 0;
    mpu_last_success_slot = 0;
    mpu_filter_resync_required = 1;
    mpu_acq_state = MPU_ACQ_IDLE;
}

uint8_t MPU6050_IsBusPriorityActive(void)
{
    return (mpu_acq_state == MPU_ACQ_REQUEST_PENDING ||
            mpu_acq_state == MPU_ACQ_DMA_ACTIVE ||
            mpu_recovery_requested) ? 1U : 0U;
}

uint8_t MPU6050_IsMotorEnableAllowed(uint32_t now)
{
    return (!mpu_calibration_active &&
            mpu_last_valid_tick != 0U &&
            (uint32_t)(now - mpu_last_valid_tick) <= MPU6050_STALE_TIMEOUT_MS &&
            mpu_consecutive_valid_samples >= MPU6050_MOTOR_READY_SAMPLES) ? 1U : 0U;
}

void MPU6050_CalibrationProcessSample(const uint8_t *buffer)
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gy;
    int16_t gz;

    if (!mpu_calibration_active || buffer == NULL) {
        return;
    }

    ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    az = (int16_t)((buffer[4] << 8) | buffer[5]);
    gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    gz = (int16_t)((buffer[12] << 8) | buffer[13]);

    calibration_ax_sum += ax;
    calibration_ay_sum += ay;
    calibration_az_sum += az;
    calibration_gy_sum += gy;
    calibration_gz_sum += gz;
    calibration_sample_count++;

    if (calibration_sample_count >= 200U) {
        accel_bias_x = (float)calibration_ax_sum / calibration_sample_count;
        accel_bias_y = (float)calibration_ay_sum / calibration_sample_count;
        accel_bias_z = ((float)calibration_az_sum / calibration_sample_count) - 16384.0f;
        gyro_bias_y = (float)calibration_gy_sum / calibration_sample_count;
        gyro_bias_z = (float)calibration_gz_sum / calibration_sample_count;
        mpu_calibration_active = 0;
        flagCalibrationIsReady = 1;
    }
}
