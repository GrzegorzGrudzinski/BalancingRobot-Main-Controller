#ifndef CONFIG_H_
#define CONFIG_H_

// #include "control.h" // for controller types

//======================
//  Hardware
//======================

#define WHEEL_RADIUS  0.055f  //
#define ROBOT_WIDTH   0.200f  // .200 wheel-wheel; 0.160 frame
#define ROBOT_HEIGHT  0.205f  // .205 floor to top; wheels center to the top: 0.155f

/*
  STM PA2 -> TX   ESP -> 16 (RX)  
  STM PA3 -> RX   ESP -> 17 (TX) (czarne)
*/
#define MOT1_UART huart2 // for  comm. with motor driver
// #define MOT2_UART huart1

#define IMU_I2C   hi2c2  // i2c for imu readings (PB3 SDA; PB10 SCL)

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

#define DEBUG_TIME 50 // 500

// PC telemetry (from control loop)(200 Hz). 1 = every cycle, 2 = 100 Hz, 4 = 50 Hz...
#define TELEMETRY_DECIMATION 1

//======================
//  Control settings
//======================
#define CTRL_TYPE_PID          0
#define CTRL_TYPE_CASCADE_PID  1
#define CONTROLLER             CTRL_TYPE_CASCADE_PID

#define CONTROL_DT (LOOP_TIME * 0.001f) // diff time in s (time change triggered with timer interrupt)

#define TARGET_ANGLE  0.0f

#define MAX_ANGLE   40.0f

//===============================
//  PID CONTROLLER PARAMETERS
//===============================
#define PID_KP  0.5f
#define PID_KI  0.0f
#define PID_KD  0.0f

#define PID_MAX_OUT 1.0f
#define PID_MAX_I ( 0.3f * PID_MAX_OUT )

//===============================
//  CASCADE PID CONTROLLER PARAMETERS
//===============================

// Position
#define CASCADE_POS_KP 0.30f // 0.3f
#define CASCADE_POS_KI 0.0f
#define CASCADE_POS_KD 0.0f
#define CASCADE_POS_MAX_OUT 0.7f  // velocity 

// Velocity
#define CASCADE_VEL_KP 1.0f // 0.06f // 0.50f // 0.5f
#define CASCADE_VEL_KI 0.50f // 2.0f // 2.0f
#define CASCADE_VEL_KD 0.0f
#define CASCADE_VEL_MAX_OUT 5.0f // angle 

// Pitch Angle
#define CASCADE_PITCH_KP 0.80f // 2.0f
#define CASCADE_PITCH_KI 0.0f
#define CASCADE_PITCH_KD 0.04f //0.4f
#define CASCADE_PITCH_MAX_OUT 2.0f  // motors max input

// Yaw Angle
#define CASCADE_YAW_KP 0.0f // 0.1f
#define CASCADE_YAW_KI 0.0f
#define CASCADE_YAW_KD 0.0f
#define CASCADE_YAW_MAX_OUT 1.0f 


//======================
//  Calibration
//======================
#define CALIB_WARMUP_TIME ( ONE_SEC*3 )
#define CALIB_TIME         ONE_SEC

#endif /* CONFIG_H_ */
