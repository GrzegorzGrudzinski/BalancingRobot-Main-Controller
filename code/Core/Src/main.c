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
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "usbd_cdc_if.h"
#include "mpu6050.h"
#include "process_commands.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_BLOCK_SIZE 0x200	//512 bytes

#define MOT1_UART huart2
#define MOT2_UART huart1

#define IMU_I2C   hi2c2

#define LOOP_TIME 5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// =========================================================
//                    GLOBAL VARIABLES
// =========================================================
MPU6050_t MPU6050;
PID_t pid;

volatile uint8_t error_flag = 0;    // Global error flag

// =========================
//    ESC variables
// =========================
// Motor 1
volatile FeedbackPacket_t esc1_data = {0}; // Data from ESC1
uint8_t rx1RawBuffer[sizeof(FeedbackPacket_t)]; // Raw buffer for receiving data
volatile uint8_t rx1_msg_recieved = 0; // message received flag

// Motor 2
volatile FeedbackPacket_t esc2_data = {0}; // Data from ESC2
uint8_t rx2RawBuffer[sizeof(FeedbackPacket_t)]; // Raw buffer for receiving data
volatile uint8_t rx2_msg_recieved = 0; // message received flag

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

  // =========================================================
  //                    UART RX CALLBACK
  // =========================================================

// Interrupt callback for UART reception
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    static uint8_t rx2State_flag = 0; // 0 -looking for start byte, 1 - receiving data

    if (rx2State_flag == 0) {
      if (rx2RawBuffer[0] == 0xBB) { // Start byte found
        rx2State_flag = 1;

        HAL_UART_Receive_IT(huart, &rx2RawBuffer[1], sizeof(FeedbackPacket_t) - 1); // Receive rest of the packet
      }
    }
    else {
      // Full packet received
      FeedbackPacket_t* recieved_pkt = (FeedbackPacket_t*)rx2RawBuffer;

      //calcuate checksum and verify
      uint8_t checksum = calculate_checksum(rx2RawBuffer, sizeof(FeedbackPacket_t));
      if (recieved_pkt->checksum == checksum) {
        // Valid packet
        esc2_data = *recieved_pkt;
        rx2_msg_recieved = 1;
      }

      rx2State_flag = 0; // Reset state to look for next packet
      // Restart reception for next packet
      HAL_UART_Receive_IT(huart, rx2RawBuffer, 1);
    } 
  } // if USART1
  else if (huart->Instance == USART2)
  {
    static uint8_t rx1State_flag = 0; // 0 -looking for start byte, 1 - receiving data

    if (rx1State_flag == 0) {
      if (rx1RawBuffer[0] == 0xBB) { // Start byte found
        rx1State_flag = 1;

        HAL_UART_Receive_IT(huart, &rx1RawBuffer[1], sizeof(FeedbackPacket_t) - 1); // Receive rest of the packet
      }
    }
    else {
      // Full packet received
      FeedbackPacket_t* recieved_pkt = (FeedbackPacket_t*)rx1RawBuffer;

      //calcuate checksum and verify
      uint8_t checksum = calculate_checksum(rx1RawBuffer, sizeof(FeedbackPacket_t));
      if (recieved_pkt->checksum == checksum) {
        // Valid packet
        esc1_data = *recieved_pkt;
        rx1_msg_recieved = 1;
      }

      rx1State_flag = 0; // Reset state to look for next packet
      // Restart reception for next packet
      HAL_UART_Receive_IT(huart, rx1RawBuffer, 1);
    } 
  } // if USART2
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  // =========================================================
  //             ANGLE MEASUREMENTS INITIALIZATION
  // =========================================================

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


  pidInit(&pid, 10, 0, 0.5, LOOP_TIME); // Kp, Ki, Kd, timeSample in ms
  // Calibrate offset
  float offset = 0.0f;
  uint32_t pid_calib_start_time = HAL_GetTick();
  uint32_t pid_calib_measure_time = HAL_GetTick();
  float pid_offset_sum = 0.0f;
  float pid_offset_mean = 0.0f;
  uint32_t pid_calib_samples = 0;
  while (HAL_GetTick() - pid_calib_start_time < 2000) {
    if (HAL_GetTick() - pid_calib_measure_time >= 10) {
      pid_calib_measure_time = HAL_GetTick();
      MPU6050_Read_All(&IMU_I2C, &MPU6050);
      pid_offset_sum += MPU6050.KalmanAngleX;
      pid_calib_samples++;
    }
    HAL_Delay(1);
  }
  pid_offset_mean = pid_offset_sum / (float)pid_calib_samples;
  offset = pid_offset_mean;

  printf("Offset calibration complete\r\n");

  HAL_Delay(50);

  HAL_UART_Receive_IT(&MOT2_UART, rx2RawBuffer, 1);
  HAL_UART_Receive_IT(&MOT1_UART, rx1RawBuffer, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    
  // =========================================================
  //                 ESC INITIALIZATION
  // =========================================================

  /*******************************
        CONNECT WITH BOTH ESCs
  *******************************/
  // Wait for ESC connection
  printf("Waiting for ESC connection...\r\n");
  esc1_data.status = 0xFF; // Invalid initial state
  esc2_data.status = 0xFF; // Invalid initial state
 
  uint32_t esc_connection_timeout = HAL_GetTick();
  uint32_t esc_hello_timer = HAL_GetTick();
  uint8_t esc1_connected_flag = 0;
  uint8_t esc2_connected_flag = 0;
  
  while ( ( !esc1_connected_flag || !esc2_connected_flag ) && (HAL_GetTick() - esc_connection_timeout < 5000 ) ) {
    uint32_t now = HAL_GetTick();

    // update flags if the response came
    if (esc1_data.status == ESC_HELLO) esc1_connected_flag = 1;
    if (esc2_data.status == ESC_HELLO) esc2_connected_flag = 1;
        
    if ((now - esc_hello_timer >= 200)) {
      esc_hello_timer = HAL_GetTick(); // Reset licznika dla Mot1
      
      if (!esc1_connected_flag) send_motor_command(&MOT1_UART, CMD_HELLO, 0);
      if (!esc2_connected_flag) send_motor_command(&MOT2_UART, CMD_HELLO, 0);
    }
  }

  // Debug message after the connection
  if (!esc1_connected_flag || !esc2_connected_flag) {
      printf("ERROR: Connection Timeout! M1: %s, M2: %s\r\n", 
              esc1_connected_flag ? "OK" : "ERROR", esc2_connected_flag ? "OK" : "ERROR");
      error_flag = 1; // Zablokuj start
  } else {
      printf("Both ESCs connected!\r\n");
  }
  
  HAL_Delay(50);

  /********************************************
       CHECK FOR IDLE STATE AND CHANGE TO RUN

      TODO Check motor spinning dir and sync
   *********************************************/
  uint32_t init_start_time = HAL_GetTick(); // Init timeout timer
  uint32_t init_sent_timer = HAL_GetTick();
  uint32_t init_debug_time = HAL_GetTick();
  
  volatile uint8_t startup1_flag = 1; // Startup flag - codition to run control loop (mot 1)
  volatile uint8_t startup2_flag = 1; // Startup flag - codition to run control loop (mot 2)

  if (!error_flag) { 
    printf("Starting the motors... \r\n");
    
    while (startup1_flag || startup2_flag) {
      // Ask for INIT every 100ms
      if (HAL_GetTick() - init_sent_timer >= 100) {
        init_sent_timer = HAL_GetTick();
        
        // Motor 1
        if (startup1_flag) {
          // IDLE -> RUN
          if (esc1_data.status == IDLE) {
            send_motor_command(&MOT1_UART, CMD_START, 0);
            startup1_flag = 0; // Exit startup loop
          }
          // clear if FAULT_OVER
          else if (esc1_data.status == FAULT_OVER) {
            // Handle fault state
            send_motor_command(&MOT1_UART, CMD_CLR_FLT, 0);
          }
          else {
            send_motor_command(&MOT1_UART, CMD_INIT, 0);
          }
        }
    
        // Motor 2
        if (startup2_flag) {
          // IDLE -> RUN
          if (esc2_data.status == IDLE) {
            send_motor_command(&MOT2_UART, CMD_START, 0);
            startup2_flag = 0; // Exit startup loop
          }
          // clear if FAULT_OVER
          else if (esc2_data.status == FAULT_OVER) {
            // Handle fault state
            send_motor_command(&MOT2_UART, CMD_CLR_FLT, 0);
          }
          else {
            send_motor_command(&MOT2_UART, CMD_INIT, 0);
          }
        }        
      } // end if (timer 100 ms)

      // Timeout after 5s
      if (HAL_GetTick() - init_start_time > 5000) {
        error_flag = 1; // Set error flag
        startup1_flag = 0; // Exit startup loop
        startup2_flag = 0;
        break;
      }
      
      // Debug info via USB
      if (HAL_GetTick() - init_debug_time >= 500) 
      {
        init_debug_time = HAL_GetTick();
        printf("MOTOR 1: STATE: %s | RPM: %.2f | CURR q: %.2f | CURR d: %.2f\r\n", 
               GetStateName(esc1_data.status), esc1_data.speed_rpm, esc1_data.current_Iq, esc1_data.current_Id);
        printf("MOTOR 2: STATE: %s | RPM: %.2f | CURR q: %.2f | CURR d: %.2f\r\n\n", 
                GetStateName(esc2_data.status), esc2_data.speed_rpm, esc2_data.current_Iq, esc2_data.current_Id);
      } 
    } // end while (startup)
  } // end if (error flag)
  else 
  { // INIT ERROR HANDLING
    while(1) {
      // Halt here
      printf("ERROR: Failed to initialize ESC! Check connections and power supply.\r\n");

      HAL_Delay(1000);
      // TODO - Add blinking led at error
    }
  }


    /********************************************
                        TODO
              Here should be the code 
                checking motors sync
   *********************************************/
  
  printf("BALANCING ROBOT - initialization complete \r\n");

  /*******************************
            MAIN LOOP
  *******************************/
  float desired_angle = 00.0f;
  float angle = 0.0f;
  float output = 0.0f;

  uint32_t last_loop_time = HAL_GetTick();
  uint32_t last_debug_time = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // Control loop
    if (HAL_GetTick() - last_loop_time >= LOOP_TIME) 
    {
      last_loop_time = HAL_GetTick();

      // 1. Read MPU6050 data
      MPU6050_Read_All(&IMU_I2C, &MPU6050);
      angle = MPU6050.KalmanAngleX;
      angle -= offset;
      if (fabsf(angle) <= 0.5) {
        angle = 0;
      }
      // 2. Calculate PID
      output = run_pid(&pid, angle, desired_angle);      // convert values to motor commands (angle -> linear vel) 
      
      // 3. Send data to ESCs
      if (esc1_data.status == RUN && esc2_data.status == RUN) {
        send_motor_command(&MOT1_UART, CMD_SET_RPM, 700.0f);
        send_motor_command(&MOT2_UART, CMD_SET_RPM, 700.0f);
      }
    }
    
    // 4. Process data from ESCs
    process_feedback(&MOT1_UART, rx1_msg_recieved, esc1_data, error_flag);
    process_feedback(&MOT2_UART, rx2_msg_recieved, esc2_data, error_flag);

    // 5. Debug info via USB 
    if (HAL_GetTick() - last_debug_time >= 200) 
    {
      last_debug_time = HAL_GetTick();
      if (error_flag) {
        printf("ERROR: Out of range values from ESC! Stopping motors.\r\n");
      }

      /*
        printf("MOTOR 1: STATE: %s | RPM: %.2f | CURR q: %.2f | CURR d: %.2f \r\n", GetStateName(esc1_data.status), esc1_data.speed_rpm, esc1_data.current_Iq, esc1_data.current_Id);
        printf("MOTOR 2: STATE: %s | RPM: %.2f | CURR q: %.2f | CURR d: %.2f \r\n", GetStateName(esc2_data.status), esc2_data.speed_rpm, esc2_data.current_Iq, esc2_data.current_Id);
        
        printf("ANGLE: %.2f | PID: %.2f \r\n\n", angle, output);
      */

      char message[256];
      snprintf (message, sizeof(message), "MOTOR 1: STATE: %s | RPM: %.2f | CURR q: %.2f | CURR d: %.2f\n\
                  MOTOR 2: STATE: %s | RPM: %.2f | CURR q: %.2f | CURR d: %.2f\n\
                  ANGLE: %.2f | PID: %.2f \r\n\n", 
              GetStateName(esc1_data.status), esc1_data.speed_rpm, esc1_data.current_Iq, esc1_data.current_Id,
              GetStateName(esc2_data.status), esc2_data.speed_rpm, esc2_data.current_Iq, esc2_data.current_Id,
              angle, output);
      printf(message);
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

// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance == USART1)
//     {
//         HAL_UART_Receive_IT(huart, rx2RawBuffer, 1);
//     }
//     else if (huart->Instance == USART2)
//     {
//         HAL_UART_Receive_IT(huart, rx1RawBuffer, 1);
//     }
// }

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
