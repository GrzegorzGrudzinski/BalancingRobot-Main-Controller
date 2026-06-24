#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include "main.h"
#include "config.h"
#include "mpu6050.h"

void  IMU_Init();
void  IMU_Update(void);
float IMU_GetPitchAngle(void);
float IMU_GetPitchRate(void);
float IMU_GetYawRate(void);

#endif