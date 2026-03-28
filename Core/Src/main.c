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
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "usbd_cdc_if.h"
#include "mpu6050.h"
#include "dualFOC_communication.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define USB_BLOCK_SIZE 0x200	//512 bytes

/*
  STM PA2 -> TX   ESP -> 16 (RX)  
  STM PA3 -> RX   ESP -> 17 (TX) (czarne)
*/
#define MOT1_UART huart2
// #define MOT2_UART huart1

#define IMU_I2C   hi2c2

#define LOOP_TIMER htim3

#define TIM_PERIOD_REG_VAL ((htim3.Init.Period+1)/1000) // ms

#define LOOP_TIME 5  
#define ONE_SEC (1000 / LOOP_TIME) // 
#define HALF_SEC (500 / LOOP_TIME) // 

#define DEBUG_TIME 500


typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_BALANCING,
    STATE_FALLEN
} RobotState_t;

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
float offset = 0.0f; // angle offset

uint8_t error_flag = 0;    // Global error flag

RobotState_t robot_state = STATE_INIT;

volatile uint8_t loop_flag = 0; // 

FeedbackPacket_t motor_feedback_data = {0}; // Data from ESC1

uint8_t rxRawBuffer[sizeof(FeedbackPacket_t)]; // Raw buffer for receiving data
volatile uint8_t rx_msg_recieved = 0; // message received flag


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
const char* GetStateName(RobotState_t s);

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&LOOP_TIMER);
  if (TIM_PERIOD_REG_VAL != LOOP_TIME) {
    /*  ERROR - WRONG LOOP TIME VALUE  */
    return -1;
  }
  sensors_init(&offset);
  HAL_UART_Receive_IT(&MOT1_UART, rxRawBuffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float target_angle = 00.0f;
  float angle = 0.0f;
  float output = 0.0f;

  uint32_t last_debug_time = HAL_GetTick();

  while (1)
  {
    // Control loop
    if (loop_flag) {    
      loop_flag = 0;
      // 1. Read MPU6050 data
      MPU6050_Read_All(&IMU_I2C, &MPU6050);
      angle = MPU6050.KalmanAngleX;
      angle -= offset;
      // if (fabsf(angle) <= 0.05) { // Target
      //   angle = 0;
      // }

      switch (robot_state) {
        case STATE_INIT: // check if the communication is ok - wait for the message from motor controller
          if (motor_feedback_data.status == CMD_HELLO_MOTOR) {
            static uint16_t init_timer = 0;
            if (++init_timer >= ONE_SEC/2) { // (send command every 500 ms)
              send_motor_command(&MOT1_UART, CMD_HELLO_MASTER, 0.0, 0.0);
              init_timer = 0;
              robot_state = STATE_IDLE;
            }
          }
          break;
        case STATE_IDLE: //  wait until everything is ready
          static uint16_t idle_timer = 0;
          if (++idle_timer >= ONE_SEC/2) { // (send command every 500 ms)
            send_motor_command(&MOT1_UART, CMD_INIT, 0.0, 0.0);
            idle_timer = 0;
          }
          if (motor_feedback_data.status == CMD_START) {
            robot_state = STATE_BALANCING;
          }
          break;
        case STATE_BALANCING: // normal operation
          if (fabsf(angle) > 45.0f){// Out of range
            robot_state = STATE_FALLEN;
            break;
          }
          // 2. Calculate PID
          output = run_pid(&pid, angle, target_angle);      // convert values to motor commands (angle -> linear vel)
          // 3. Send data to ESCs
          send_motor_command(&MOT1_UART, CMD_SET_VAL, output, output);
          break;
        case STATE_FALLEN:
          send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
          pid_reset(&pid);
          output = 0;

          // if (fabsf(angle) < 5.0f){ // Back in range
            // robot_state = STATE_BALANCING;
            // break;
          // }
          break;
        default:
          send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
          pid_reset(&pid);
          output = 0;
          break;
      }       
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_GetTick() - last_debug_time >= DEBUG_TIME) {
      last_debug_time = HAL_GetTick();
      printf("State: %s\tAngle: %.2f\tOutput2: %.2f\n\r", GetStateName(robot_state), angle, output);
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
    if (HAL_GetTick() - mpu_init_time > ONE_SEC) {
      printf("IMU initialization\r\n");
      mpu_init_time = HAL_GetTick(); // Reset timer
    }
  }
  printf("IMU - initialization complete \r\n");

  
  pidInit(&pid, 0.1, 0, 0, 1, 1, 10);
  
  printf("Offset calibration in 3 sec...\r\n");
  
  uint32_t warmup_ticks = 0;
  
  // Filter warmup
  while (warmup_ticks < ONE_SEC*3) {
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

  while (calib_ticks < ONE_SEC) {
    if (loop_flag) {
      loop_flag = 0;
      MPU6050_Read_All(&IMU_I2C, &MPU6050);
      pid_offset_sum += MPU6050.KalmanAngleX;
      calib_ticks++;
    }
    // HAL_Delay(1);
  }
  *offset = pid_offset_sum / (float)calib_ticks;

  printf("Offset calibration complete. Offset = %.2f deg\r\n", *offset);
  
  pid_reset(&pid);
  loop_flag = 0;
  HAL_Delay(50);
}

const char* GetStateName(RobotState_t s) {
  switch(s) {
    case STATE_INIT: return "INIT";
    case STATE_IDLE: return "IDLE";
    case STATE_BALANCING: return "BALANCING";
    case STATE_FALLEN: return "FALLEN";
    default: return "UNKNOWN";
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        loop_flag = 1; // Set main loop flag 
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    static uint8_t rxState_flag = 0; // 0 -looking for start byte, 1 - receiving data

    if (rxState_flag == 0) {
      if (rxRawBuffer[0] == 0xBB) { // Start byte found
        rxState_flag = 1;

        HAL_UART_Receive_IT(huart, &rxRawBuffer[1], sizeof(FeedbackPacket_t) - 1); // Receive rest of the packet
      }
      else {
        HAL_UART_Receive_IT(huart, rxRawBuffer, 1); 
      }
    }
    else if (rxState_flag == 1) {
      // Full packet received
      FeedbackPacket_t* recieved_pkt = (FeedbackPacket_t*)rxRawBuffer;

      //calcuate checksum and verify
      uint8_t checksum = calculate_checksum(rxRawBuffer, sizeof(FeedbackPacket_t));
      if (recieved_pkt->checksum == checksum) {
        // Valid packet
        motor_feedback_data = *recieved_pkt;
        rx_msg_recieved = 1;
      }

      rxState_flag = 0; // Reset state to look for next packet
      // Restart reception for next packet
      HAL_UART_Receive_IT(huart, rxRawBuffer, 1);
    } 
  } // if USART2
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
