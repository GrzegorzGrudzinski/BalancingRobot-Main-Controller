#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include "main.h"
#include "config.h"
#include "mpu6050.h"

// acceleration in g, angular speed in deg/s
typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} IMU_Sample_t;

void  IMU_Init();
// void  IMU_Update(void);
uint8_t IMU_Update(void);             // 0 = ok, 1 = I2C error
void  IMU_GetSample(IMU_Sample_t *s); // last sensor reading

float IMU_GetPitchAngle(void);
float IMU_GetPitchRate(void);
float IMU_GetYawRate(void);

#endif