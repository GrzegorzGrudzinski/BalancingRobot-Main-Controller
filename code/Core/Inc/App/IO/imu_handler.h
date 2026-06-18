#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include "main.h"
#include "config.h"
#include "mpu6050.h"

void  IMU_Init();
float IMU_GetAngle(void);

#endif