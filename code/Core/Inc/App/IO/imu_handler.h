#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include "main.h"
#include "config.h"
#include "mpu6050.h"
#include "pid.h" 

void IMU_Init(PID_t* pid);
float IMU_GetAngle(void);

#endif