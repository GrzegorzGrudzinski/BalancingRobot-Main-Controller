/*
  imu_handler.c
*/

#include "imu_handler.h"
#include "diag.h"

#include <stdio.h>


extern volatile uint8_t loop_flag;
extern I2C_HandleTypeDef IMU_I2C;

static MPU6050_t MPU6050;
static float offset = 0.0f;

void IMU_Init() {
  printf("BALANCING ROBOT - starting initialization \r\n");
  uint32_t mpu_init_time = HAL_GetTick(); // Init timeout timer
  
  while (MPU6050_Init(&IMU_I2C) == 1) {
    if (HAL_GetTick() - mpu_init_time > ONE_SEC) {
      printf("IMU initialization\r\n");
      mpu_init_time = HAL_GetTick(); // Reset timer
    }
  }
  printf("IMU - initialization complete \r\n");
    
  printf("Offset calibration in 3 sec...\r\n");
  uint32_t warmup_ticks = 0;
  
  // Filter warmup
  while (warmup_ticks < CALIB_WARMUP_TIME) {
    if (loop_flag) {
      loop_flag = 0;
      MPU6050_Read_All(&IMU_I2C, &MPU6050, CONTROL_DT); 
      warmup_ticks++;
    }
  }
  // Calibrate offset
  printf("Calibration start - DONT MOVE THE DEVICE!\n\r");

  float offset_sum = 0.0f;
  uint32_t calib_ticks = 0;
  loop_flag = 0;

  while (calib_ticks < CALIB_TIME) {
    if (loop_flag) {
      loop_flag = 0;
      MPU6050_Read_All(&IMU_I2C, &MPU6050, CONTROL_DT);
      offset_sum += MPU6050.KalmanAngleX;
      calib_ticks++;
    }
    // HAL_Delay(1);
  }
  // offset = offset_sum / (float)calib_ticks;
  offset = IMU_ANGLE_OFFSET;
  printf("Offset calibration complete. Offset = %.2f deg\r\n", offset);
  
  loop_flag = 0;
  HAL_Delay(50);
}

uint8_t IMU_Update(void) {
  if (MPU6050_Read_All(&IMU_I2C, &MPU6050, CONTROL_DT) != 0) {
    diag.imu_error_count++;
    return 1;
  }
  return 0;
}

void IMU_GetSample(IMU_Sample_t *s) {
  s->ax = (float)MPU6050.Ax;
  s->ay = (float)MPU6050.Ay;
  s->az = (float)MPU6050.Az; 
  s->gx = (float)MPU6050.Gx;
  s->gy = (float)MPU6050.Gy;
  s->gz = (float)MPU6050.Gz;
}

float IMU_GetPitchAngle(void) {
    return MPU6050.KalmanAngleX - offset;
}

float IMU_GetPitchRate(void) {
    return MPU6050.Gx;
}

float IMU_GetYawRate(void) {
    return MPU6050.Gy;
}