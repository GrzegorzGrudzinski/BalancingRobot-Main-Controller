/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "usbd_cdc_if.h"
#include "mpu6050.h"
#include "dualFOC_commands.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USB_BLOCK_SIZE 0x200	//512 bytes

#define MOT1_UART huart2
// #define MOT2_UART huart1

#define IMU_I2C   hi2c2

#define LOOP_TIME 5
#define DEBUG_TIME 500

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
PID_t pid;

uint8_t error_flag = 0;    // Global error flag

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// _write override for the printf funct
int _write(int file, char *ptr, int len) {
    CDC_Transmit_FS((uint8_t*) ptr, len);
    return len;
}

void sensors_init(float* offset);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  float offset = 0.0f;
  sensors_init(&offset);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float target_angle = 00.0f;
  float angle = 0.0f;
  float output = 0.0f;

  uint32_t last_loop_time = HAL_GetTick();
  uint32_t last_debug_time = HAL_GetTick();

  while (1)
  {
    // Control loop
    if (HAL_GetTick() - last_loop_time >= LOOP_TIME) 
    {
      // last_loop_time = HAL_GetTick();
      last_loop_time += LOOP_TIME;
      
      // 1. Read MPU6050 data
      MPU6050_Read_All(&IMU_I2C, &MPU6050);
      angle = MPU6050.KalmanAngleX;
      angle -= offset;
      // if (fabsf(angle) <= 0.05) { // Target
      //   angle = 0;
      // }
      if (fabsf(angle) > 45.0f) // Out of range
      {
        send_motor_command(&MOT1_UART, 0x11, 0, 0);
        
        pid_reset(&pid);
        output = 0;
      }
      else { // Normal operation  
        // 2. Calculate PID
        output = run_pid(&pid, angle, target_angle);      // convert values to motor commands (angle -> linear vel) 
      
        // 3. Send data to ESCs
        send_motor_command(&MOT1_UART, 0x01, output, output);
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_GetTick() - last_debug_time >= DEBUG_TIME) {
      last_debug_time = HAL_GetTick();
      printf("Angle: %.2f\tOutput2: %.2f\n\r", angle, output);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void sensors_init(float* offset) {
    HAL_Delay(1000); // TEMP (for debugging)
  
  printf("BALANCING ROBOT - starting initialization \r\n");
  
  uint32_t mpu_init_time = HAL_GetTick(); // Init timeout timer
  
  while (MPU6050_Init(&IMU_I2C) == 1) {
    if (HAL_GetTick() - mpu_init_time > 1000) {
      printf("IMU initialization\r\n");
      mpu_init_time = HAL_GetTick(); // Reset timer
    }
  }
  printf("IMU - initialization complete \r\n");

  
  pidInit(&pid, 0.1, 0, 0, 1, 1, 10);
  
  printf("Offset calibration in 3 sec...\r\n");
  uint32_t warmup_start = HAL_GetTick();
  uint32_t warmup_loop = HAL_GetTick();
  
  while (HAL_GetTick() - warmup_start < 3000) {
    // Musimy czytać IMU co równe 5 ms (LOOP_TIME), żeby filtr poprawnie działał
    if (HAL_GetTick() - warmup_loop >= LOOP_TIME) {
      warmup_loop = HAL_GetTick();
      MPU6050_Read_All(&IMU_I2C, &MPU6050); // Czytamy "w próżnię", żeby filtr się ułożył
    }
  }
  // Calibrate offset
  printf("Calibration start - DONT MOVE THE DEVICE!\n\r");
  uint32_t pid_calib_start_time = HAL_GetTick();
  uint32_t pid_calib_measure_time = HAL_GetTick();
  float pid_offset_sum = 0.0f;
  // float pid_offset_mean = 0.0f;
  uint32_t pid_calib_samples = 0;
  while (HAL_GetTick() - pid_calib_start_time < 1000) {
    if (HAL_GetTick() - pid_calib_measure_time >= LOOP_TIME) {
      // pid_calib_measure_time = HAL_GetTick();
      pid_calib_measure_time += LOOP_TIME;
      MPU6050_Read_All(&IMU_I2C, &MPU6050);
      pid_offset_sum += MPU6050.KalmanAngleX;
      pid_calib_samples++;
    }
    // HAL_Delay(1);
  }
  *offset = pid_offset_sum / (float)pid_calib_samples;

  printf("Offset calibration complete. Offset = %.2f deg\r\n", *offset);
  
  pid_reset(&pid);
  HAL_Delay(50);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
