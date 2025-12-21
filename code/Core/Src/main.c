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

#define MOT1_UART huart2
#define MOT2_UART huart1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// MPU6050_t MPU6050;
PID_t pid;



// ESC data

// Motor 1
volatile FeedbackPacket_t esc1_data = {0}; // Data from ESC1

uint8_t rx1RawBuffer[sizeof(FeedbackPacket_t)]; // Raw buffer for receiving data
volatile uint8_t rx1_msg_recieved = 0; // message received flag

// Motor 2
volatile FeedbackPacket_t esc2_data = {0}; // Data from ESC2

uint8_t rx2RawBuffer[sizeof(FeedbackPacket_t)]; // Raw buffer for receiving data
volatile uint8_t rx2_msg_recieved = 0; // message received flag

// ESC communication flags
volatile uint8_t error_flag = 0; // Error flag
volatile uint8_t startup1_flag = 1; // Startup flag - codition to run control loop (mot 1)
volatile uint8_t startup2_flag = 1; // Startup flag - codition to run control loop (mot 2)

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
      // else {
      //   HAL_UART_Receive_IT(huart, rx2RawBuffer, 1);
      // }
    }
    else if (rx2State_flag == 1) {
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
      // else {
      //   HAL_UART_Receive_IT(huart, rx1RawBuffer, 1);
      // }
    }
    else if (rx1State_flag == 1) {
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

  printf("BALANCING ROBOT - starting initialization \r\n");
  
  // while (MPU6050_Init(&hi2c1) == 1);
  pidInit(&pid, 2, 0.2, 0.2, 10); // Kp, Ki, Kd, timeSample in ms

  HAL_Delay(500);

  HAL_UART_Receive_IT(&MOT2_UART, rx2RawBuffer, 1);
  HAL_UART_Receive_IT(&MOT1_UART, rx1RawBuffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  const float desired_angle = 90.0f;
  float output = 0.0f;
  
  // 0. INIT

  /*******************************
      ESC INITIALIZATION CODE 
  *******************************/

  // Wait for ESC connection
  printf("Waiting for ESC connection...\r\n");
  esc1_data.status = 0xFF; // Invalid initial state
  esc2_data.status = 0xFF; // Invalid initial state
 
  uint32_t esc_init_time = HAL_GetTick();
  while(esc1_data.status != ESC_HELLO) {
    if ((HAL_GetTick() - esc_init_time >= 100)) {
      esc_init_time = HAL_GetTick(); // Reset licznika dla Mot1
      send_motor_command(&MOT1_UART, CMD_HELLO, 0);
    }
  }
  printf("ESC1 connected!\r\n");
  HAL_Delay(10);

  esc_init_time = HAL_GetTick();
  while(esc2_data.status != ESC_HELLO) {
    if ((HAL_GetTick() - esc_init_time >= 100)) {
      esc_init_time = HAL_GetTick(); // Reset licznika dla Mot1
      send_motor_command(&MOT2_UART, CMD_HELLO, 0);
    }
  } 
  printf("ESC2 connected!\r\n");
    HAL_Delay(10);

  // Check for IDLE state
  uint32_t init_start_time = HAL_GetTick(); // Init timeout timer
  uint32_t init_debug_time = HAL_GetTick();
  while (startup1_flag || startup2_flag) {
    // Ask for INIT every 100ms

    // Motor 1
    if (startup1_flag) {
      send_motor_command(&MOT1_UART, CMD_INIT, 0);
      HAL_Delay(100);
  
      if (esc1_data.status == IDLE) {
        send_motor_command(&MOT1_UART, CMD_START, 0);
        HAL_Delay(200);
        startup1_flag = 0; // Exit startup loop
      }
      else if (esc1_data.status == FAULT_OVER) {
        // Handle fault state
        send_motor_command(&MOT1_UART, CMD_CLR_FLT, 0);
        HAL_Delay(100);
      }
    }

    // Motor 2
    if (startup2_flag) {
      send_motor_command(&MOT2_UART, CMD_INIT, 0);
      HAL_Delay(100);
  
      if (esc2_data.status == IDLE) {
        send_motor_command(&MOT2_UART, CMD_START, 0);
        HAL_Delay(200);
        startup2_flag = 0; // Exit startup loop
      }
      else if (esc2_data.status == FAULT_OVER) {
        // Handle fault state
        send_motor_command(&MOT2_UART, CMD_CLR_FLT, 0);
        HAL_Delay(100);
      }
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

    // Timeout after 5s
    if (HAL_GetTick() - init_start_time > 5000) {
      error_flag = 1; // Set error flag
      startup1_flag = 0; // Exit startup loop
      startup2_flag = 0;
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

  /*******************************
            MAIN LOOP
  *******************************/

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
      float angle = 90.05f; // TEMP

      // 2. Compute PID
      output = run_pid(&pid, angle, desired_angle);      // convert values to motor commands (angle -> linear vel) 
      
      // 3. Send data to ESCs
      if (esc1_data.status == RUN && esc2_data.status == RUN) {
        send_motor_command(&MOT1_UART, CMD_SET_RPM, 700.0f);
        send_motor_command(&MOT2_UART, CMD_SET_RPM, 700.0f);
      }
    }
    
    // 4. Process data from ESCs
    if (rx1_msg_recieved) {
      //Check for errors from ESC
      if (esc1_data.speed_rpm > 4000.0f || esc1_data.speed_rpm < -4000.0f || 
        esc1_data.current_Iq > 30.0f || esc1_data.current_Iq < -30.0f)
      {
          // Handle error

          // send_motor_command_dma(&MOT2_UART, CMD_STOP, 0);
          send_motor_command(&MOT1_UART, CMD_STOP, 0);
          error_flag = 1; // Set error flag
      }
      switch (esc1_data.status) {
        case FAULT_NOW:
          // Handle fault state
          error_flag = 1; // Set error flag
          break;
        case FAULT_OVER:
          // Handle fault state
          send_motor_command(&MOT1_UART, CMD_CLR_FLT, 0);
          break;
        case IDLE:
          // Restart motor if in IDLE
          send_motor_command(&MOT1_UART, CMD_START, 0);
          break;

        // case RUN:
        //   // Normal operation
        //   break;
        // case START:
        //   // Motor is starting
        //   break;
        // case STOP:
        //   // Motor is stopping
        //   break;

        case OFFSET_CALIB:
          // Calibration state
          HAL_Delay(200); // Wait for calibration
          break;
        case CHARGE_BOOT_CAP:
          // Charging capacitors
          HAL_Delay(200); // Wait for charging
          break;
        case ALIGNMENT:
          // Alignment state
          break;
        case SWITCH_OVER:
          // Switching from open loop to closed loop
          break;
        case ICLWAIT:
          // Waiting for ICL to clear
          break;
        case WAIT_STOP_MOTOR:
          // Waiting for motor to stop
          break;
        case OTF_DETECTION:
          // On-the-fly detection
          break;
        case OTF_BRAKE:
          // On-the-fly braking
          break;

        default:
          break;
      }
      rx1_msg_recieved = 0;
    }

    if (rx2_msg_recieved) {
      //Check for errors from ESC
      if (esc2_data.speed_rpm > 4000.0f || esc2_data.speed_rpm < -4000.0f || 
        esc2_data.current_Iq > 30.0f || esc2_data.current_Iq < -30.0f)
      {
          // Handle error

          // send_motor_command_dma(&MOT2_UART, CMD_STOP, 0);
          send_motor_command(&MOT2_UART, CMD_STOP, 0);
          error_flag = 1; // Set error flag
      }

      switch (esc2_data.status) {
        case FAULT_NOW:
          // Handle fault state
          error_flag = 1; // Set error flag
          break;
        case FAULT_OVER:
          // Handle fault state
          send_motor_command(&MOT2_UART, CMD_CLR_FLT, 0);
          break;
        case IDLE:
          // Restart motor if in IDLE
          send_motor_command(&MOT2_UART, CMD_START, 0);
          break;

        // case RUN:
        //   // Normal operation
        //   break;
        // case START:
        //   // Motor is starting
        //   break;
        // case STOP:
        //   // Motor is stopping
        //   break;

        case OFFSET_CALIB:
          // Calibration state
          HAL_Delay(200); // Wait for calibration
          break;
        case CHARGE_BOOT_CAP:
          // Charging capacitors
          HAL_Delay(200); // Wait for charging
          break;
        case ALIGNMENT:
          // Alignment state
          break;
        case SWITCH_OVER:
          // Switching from open loop to closed loop
          break;
        case ICLWAIT:
          // Waiting for ICL to clear
          break;
        case WAIT_STOP_MOTOR:
          // Waiting for motor to stop
          break;
        case OTF_DETECTION:
          // On-the-fly detection
          break;
        case OTF_BRAKE:
          // On-the-fly braking
          break;

        default:
          break;
      }
      rx2_msg_recieved = 0;
    }

    // 5. Debug info via USB 
    if (HAL_GetTick() - last_debug_time >= 200) 
    {
      last_debug_time = HAL_GetTick();
      if (error_flag) {
        printf("ERROR: Out of range values from ESC! Stopping motors.\r\n");
      }

      printf("MOTOR 1: STATE: %s | RPM: %.2f | CURR q: %.2f | CURR d: %.2f | PID Output: %.2f\r\n", GetStateName(esc1_data.status), esc1_data.speed_rpm, esc1_data.current_Iq, esc1_data.current_Id, output);
      printf("MOTOR 2: STATE: %s | RPM: %.2f | CURR q: %.2f | CURR d: %.2f | PID Output: %.2f\r\n", GetStateName(esc2_data.status), esc2_data.speed_rpm, esc2_data.current_Iq, esc2_data.current_Id, output);
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
