#ifndef CONFIG_H_
#define CONFIG_H_

#include "control.h" // for controller types

//======================
//  Hardware
//======================

#define WHEEL_RADIUS  0.005f
#define ROBOT_WIDTH   0.020f
#define ROBOT_HEIGHT  0.020f

/*
  STM PA2 -> TX   ESP -> 16 (RX)  
  STM PA3 -> RX   ESP -> 17 (TX) (czarne)
*/
#define MOT1_UART huart2
// #define MOT2_UART huart1

#define IMU_I2C   hi2c2

#define LOOP_TIMER htim3

#define BUTTON_PIN_PORT GPIOA
#define BUTTON_PIN_NUM  GPIO_PIN_0

// pull up config: pressed = reset; idle = set;
#define BUTTON_PRESSED  GPIO_PIN_RESET
#define BUTTON_IDLE     GPIO_PIN_SET
//======================
//  Timers
//======================

#define TIM_PERIOD_REG_VAL ((htim3.Init.Period+1)/1000) // ms

#define LOOP_TIME   5  // 5 ms / 200 Hz
#define ONE_SEC     ( 1000 / LOOP_TIME ) // 
#define HALF_SEC    (  500 / LOOP_TIME ) // 

#define DEBUG_TIME 500

//======================
//  Control settings
//======================
#define CONTROLLER  CTRL_TYPE_CASCADE_PID

#define TARGET_ANGLE  0.0f;

#define MAX_ANGLE   60.0f

//===============================
//  PID CONTROLLER PARAMETERS
//===============================
#define PID_KP  0.5
#define PID_KI  0.0
#define PID_KD  0.0

#define PID_MAX_OUT 1.0
#define PID_MAX_I 1.0

//===============================
//  CASCADE PID CONTROLLER PARAMETERS
//===============================

// Position
#define CASCADE_POS_KP 0.3f
#define CASCADE_POS_KI 0.0f
#define CASCADE_POS_KD 0.0f
#define CASCADE_POS_MAX_OUT 0.5f 

// Velocity
#define CASCADE_VEL_KP 0.5f
#define CASCADE_VEL_KI 2.0f
#define CASCADE_VEL_KD 0.0f
#define CASCADE_VEL_MAX_OUT 10.0f

// Pitch Angle
#define CASCADE_PITCH_KP 10.0f
#define CASCADE_PITCH_KI 0.0f
#define CASCADE_PITCH_KD 0.5f
#define CASCADE_PITCH_MAX_OUT 1.0f 

// Yaw Angle
#define CASCADE_YAW_KP 0.1f
#define CASCADE_YAW_KI 0.0f
#define CASCADE_YAW_KD 0.0f
#define CASCADE_YAW_MAX_OUT 1.0f 


//======================
//  Calibration
//======================
#define CALIB_WARMUP_TIME ( ONE_SEC*3 )
#define CALIB_TIME         ONE_SEC

#endif /* CONFIG_H_ */
