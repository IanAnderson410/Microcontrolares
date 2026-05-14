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

        data = 0x04;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1A, 1, &data, 1, 100);

        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x19, 1, &data, 1, 100);
    }
}

void MPU6050_Calibrate(void)
{
    if (flagCalibrationIsReady == 0) {
        int32_t axS = 0, ayS = 0, azS = 0;
        int32_t gxS = 0, gyS = 0, gzS = 0;
        int num_samples = 200;

        uint8_t buffer[14];

        for (int i = 0; i < num_samples; i++) {
            if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buffer, 14, 100) != HAL_OK) {
                Error_Handler();
            }

            axS += (int16_t)(buffer[0] << 8 | buffer[1]);
            ayS += (int16_t)(buffer[2] << 8 | buffer[3]);
            azS += (int16_t)(buffer[4] << 8 | buffer[5]);
            gxS += (int16_t)(buffer[8] << 8 | buffer[9]);
            gyS += (int16_t)(buffer[10] << 8 | buffer[11]);
            gzS += (int16_t)(buffer[12] << 8 | buffer[13]);
            HAL_Delay(11);
        }

        accel_bias_x = (float)axS / num_samples;
        accel_bias_y = (float)ayS / num_samples;
        accel_bias_z = ((float)azS / num_samples) - 16384.0f;
        gyro_bias_y = (float)gyS / num_samples;
        (void)gxS;
        (void)gzS;

        flagCalibrationIsReady = 1;
    }
}
