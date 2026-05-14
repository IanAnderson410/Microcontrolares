#ifndef MPU6050_APP_H
#define MPU6050_APP_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ADDR (0x68 << 1)

extern I2C_HandleTypeDef hi2c1;
extern volatile uint8_t flagCalibrationIsReady;
extern float accel_bias_x;
extern float accel_bias_y;
extern float accel_bias_z;
extern float gyro_bias_x;
extern float gyro_bias_y;
extern float gyro_bias_z;

void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Calibrate(void);

#ifdef __cplusplus
}
#endif

#endif
