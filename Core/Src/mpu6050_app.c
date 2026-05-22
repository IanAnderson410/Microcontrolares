#include "mpu6050_app.h"

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
    if (flagCalibrationIsReady == 0) {
        int32_t axS = 0, azS = 0, gyS = 0;
        int num_samples = 200;

        uint8_t buffer[14];

        for (int i = 0; i < num_samples; i++) {
            if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buffer, sizeof(buffer), 100) != HAL_OK) {
                Error_Handler();
            }

            axS += (int16_t)(buffer[0] << 8 | buffer[1]);
            azS += (int16_t)(buffer[4] << 8 | buffer[5]);
            gyS += (int16_t)(buffer[10] << 8 | buffer[11]);
            HAL_Delay(11);
        }

        accel_bias_x = (float)axS / num_samples;
        accel_bias_y = 0.0f;
        accel_bias_z = ((float)azS / num_samples) - 16384.0f;
        gyro_bias_x = 0.0f;
        gyro_bias_y = (float)gyS / num_samples;
        gyro_bias_z = 0.0f;

        flagCalibrationIsReady = 1;
    }
}
