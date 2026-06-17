#include "imu_handler.h"
#include <stdio.h>


extern volatile uint8_t loop_flag;
extern I2C_HandleTypeDef IMU_I2C;

static MPU6050_t MPU6050;
static float offset = 0.0f;

void IMU_Init(PID_t* pid) {
  HAL_Delay(1000); // TEMP (for debugging)
  printf("BALANCING ROBOT - starting initialization \r\n");
  uint32_t mpu_init_time = HAL_GetTick(); // Init timeout timer
  
  while (MPU6050_Init(&IMU_I2C) == 1) {
    if (HAL_GetTick() - mpu_init_time > ONE_SEC) {
      printf("IMU initialization\r\n");
      mpu_init_time = HAL_GetTick(); // Reset timer
    }
  }
  printf("IMU - initialization complete \r\n");
  
  pidInit(pid, PID_KP, PID_KI, PID_KD, PID_MAX_OUT, PID_MAX_I, LOOP_TIME);
  
  printf("Offset calibration in 3 sec...\r\n");
  uint32_t warmup_ticks = 0;
  
  // Filter warmup
  while (warmup_ticks < CALIB_WARMUP_TIME) {
    if (loop_flag) {
      loop_flag = 0;
      MPU6050_Read_All(&IMU_I2C, &MPU6050); 
      warmup_ticks++;
    }
  }
  // Calibrate offset
  printf("Calibration start - DONT MOVE THE DEVICE!\n\r");

  float pid_offset_sum = 0.0f;
  uint32_t calib_ticks = 0;
  loop_flag = 0;

  while (calib_ticks < CALIB_TIME) {
    if (loop_flag) {
      loop_flag = 0;
      MPU6050_Read_All(&IMU_I2C, &MPU6050);
      pid_offset_sum += MPU6050.KalmanAngleX;
      calib_ticks++;
    }
    // HAL_Delay(1);
  }
  offset = pid_offset_sum / (float)calib_ticks;
  printf("Offset calibration complete. Offset = %.2f deg\r\n", offset);
  
  float angle = MPU6050.KalmanAngleX - offset;
  pid_reset(pid, angle);
  loop_flag = 0;
  HAL_Delay(50);
}

float IMU_GetAngle(void) {
    MPU6050_Read_All(&IMU_I2C, &MPU6050);
    return MPU6050.KalmanAngleX - offset;
}