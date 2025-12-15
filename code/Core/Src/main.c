/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "usbd_cdc_if.h"
#include "process_commands.h"
#include "position_checker.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_BLOCK_SIZE 0x200	//512 bytes

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// MPU6050_t MPU6050;
PID_t pid;



// ESC data
FeedbackPacket_t esc1_data = {0}; // Data from ESC1

uint8_t rx1RawBuffer[sizeof(FeedbackPacket_t)]; // Raw buffer for receiving data
volatile uint8_t rx1_msg_recieved = 0; // message received flag


volatile uint8_t error_flag = 0; // Error flag
volatile uint8_t startup_flag = 1; // Startup flag - codition to run control loop

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  printf("BALANCING ROBOT - starting initialization \r\n");
  
  // while (MPU6050_Init(&hi2c1) == 1);
  pidInit(&pid, 2, 0.2, 0.2, 100); // Kp, Ki, Kd, timeSample in ms

  HAL_Delay(500);

  HAL_UART_Receive_IT(&huart2, rx1RawBuffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  const float desired_angle = 90.0f;
  float output = 0.0f;
  
  // 0. INIT
  printf("Waiting for ESC connection...\r\n");
  esc1_data.status = 0xFF; // Invalid initial state
  while(esc1_data.status != ESC_HELLO) {
    // Wait for ESC to power up
    send_motor_command(&huart2, CMD_HELLO, 0);
    HAL_Delay(100);  
  } 
  printf("ESC connected!\r\n");
  
  uint32_t init_start_time = HAL_GetTick(); // Init timeout timer
  uint32_t init_debug_time = HAL_GetTick();
  while (startup_flag) {
    // Wait for ESC connection

    // Ask for INIT every 100ms
    send_motor_command(&huart2, CMD_INIT, 0);
    HAL_Delay(100);

    if (esc1_data.status == IDLE) {
      send_motor_command(&huart2, CMD_START, 0);
      startup_flag = 0; // Exit startup loop
    }
    else if (esc1_data.status == FAULT_OVER){
      // Handle fault state
      send_motor_command(&huart2, CMD_CLR_FLT, 0);
      HAL_Delay(100);
    }
    
    // Debug info via USB
    if (HAL_GetTick() - init_debug_time >= 500) 
    {
      init_debug_time = HAL_GetTick();
      printf("STATE: %s | RPM: %.2f | CURR: %.2f\r\n", GetStateName(esc1_data.status), esc1_data.speed_rpm, esc1_data.current_Iq);
    }

    // Timeout after 5s
    if (HAL_GetTick() - init_start_time > 5000) {
      error_flag = 1; // Set error flag
      startup_flag = 0; // Exit startup loop
      break;
    }
  }

  // INIT ERROR HANDLING
  if (error_flag) {
    while(1) {
      // Halt here
      printf("ERROR: Failed to initialize ESC! Check connections and power supply.\r\n");
      HAL_Delay(1000);
    }
  }
  printf("BALANCING ROBOT - initialization complete \r\n");

  uint32_t last_loop_time = HAL_GetTick();
  uint32_t last_debug_time = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // Control loop
    if (HAL_GetTick() - last_loop_time >= 10) 
    {
      last_loop_time = HAL_GetTick();

      // 1. Read MPU6050 data
      // MPU6050_Read_All(&hi2c1, &MPU6050);
      // float angle = MPU6050.KalmanAngleX;
      float angle = 0.05f; // TEMP

      // 2. Compute PID
      output = run_pid(&pid, angle, desired_angle);      // convert values to motor commands (angle -> linear vel) 
      
      // 3. Send data to ESCs
      // send_motor_command(&huart2, CMD_SET_RPM, output);
    }
    
    // 4. Process data from ESCs
    if (rx1_msg_recieved) {
      //Check for errors from ESC
      if (esc1_data.speed_rpm > 5000.0f || esc1_data.speed_rpm < -5000.0f || 
        esc1_data.current_Iq > 10.0f || esc1_data.current_Iq < -10.0f) {
          // Handle error

          // send_motor_command_dma(&huart1, CMD_STOP, 0);
          send_motor_command(&huart2, CMD_STOP, 0);

          error_flag = 1; // Set error flag
        }
        if (esc1_data.status == FAULT_NOW) {
          // Handle fault state
          
          // send_motor_command_dma(&huart1, CMD_STOP, 0);
          send_motor_command(&huart2, CMD_STOP, 0);

          error_flag = 1; // Set error flag
        }

      rx1_msg_recieved = 0;
    }



    // 5. Debug info via USB 
    if (HAL_GetTick() - last_debug_time >= 200) 
    {
      last_debug_time = HAL_GetTick();
      if (error_flag) {
        printf("ERROR: Out of range values from ESC! Stopping motors.\r\n");
      }

      printf("STATE: %s | RPM: %.2f | CURR: %.2f | PID Output: %.2f\r\n", GetStateName(esc1_data.status), esc1_data.speed_rpm, esc1_data.current_Iq, output);
    }

    /* USER CODE END WHILE */
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
