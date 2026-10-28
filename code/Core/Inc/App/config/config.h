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

#define MOTOR_FB_TIMEOUT  (10*LOOP_TIME*1000) // 50000 -> ~10 cycles
#define STOP_ACK_TIMEOUT  ( ONE_SEC/10 ) // 100 ms for acknowledging STOP cmd

//======================
//  Calibration
//======================
#define CALIB_WARMUP_TIME ( ONE_SEC*3 )
#define CALIB_TIME         ONE_SEC

#define IMU_ANGLE_OFFSET -2.0f

#define START_ANGLE   3.0f          // [deg] max pitch during start
#define UPRIGHT_TIME  ( ONE_SEC/5 ) // 200 ms vertical before start


//======================
//  Control settings
//======================
#define CTRL_TYPE_PID          0
#define CTRL_TYPE_CASCADE_PID  1
#define CONTROLLER             CTRL_TYPE_CASCADE_PID

#define CONTROL_DT (LOOP_TIME * 0.001f) // diff time in s (time change triggered with timer interrupt)

#define TARGET_ANGLE  0.0f
#define MAX_ANGLE   40.0f // [deg]  // 25.0f

//===============================
//  CONTROLLER PARAMETERS
//===============================
#if CONTROLLER == CTRL_TYPE_CASCADE_PID
  // Position
  #define CASCADE_POS_KP 0.60f // was 0.40f
  #define CASCADE_POS_KI 0.0f
  #define CASCADE_POS_KD 0.0f
  #define CASCADE_POS_MAX_OUT 1.0f  // velocity 
  
  // Velocity
  #define CASCADE_VEL_KP 8.0f  // was 6.0f
  #define CASCADE_VEL_KI 1.00f // was 0.50f
  #define CASCADE_VEL_KD 0.0f // NOT USED
  #define CASCADE_VEL_MAX_OUT 8.0f // angle 

  // Pitch Angle
  #define CASCADE_PITCH_KP 1.20f // was 0.80f
  #define CASCADE_PITCH_KI 0.0f
  #define CASCADE_PITCH_KD 0.05f // was 0.04f
  #define CASCADE_PITCH_MAX_OUT 2.5f  // motors max input
  #define CASCADE_VEL_MAX_I  6.0f 

  // Yaw Angle
  #define CASCADE_YAW_KP 0.0f // 0.1f
  #define CASCADE_YAW_KI 0.0f
  #define CASCADE_YAW_KD 0.0f
  #define CASCADE_YAW_MAX_OUT 1.0f 

  #define CTRL_MAX_OUT CASCADE_PITCH_MAX_OUT
#else
  #define PID_KP  0.5f
  #define PID_KI  0.0f
  #define PID_KD  0.0f

  #define PID_MAX_OUT 1.0f
  #define PID_MAX_I ( 0.3f * PID_MAX_OUT )

  #define CTRL_MAX_OUT PID_MAX_OUT
#endif

#define SAT_TIMEOUT   (ONE_SEC/2) // 0.5 s of output saturated in one direction -> robot lifted or falling
#define MAX_VELOCITY  4.0f    // [m/s] faster only with the wheels in the air (or runaway)


//======================
//  OTHER
//======================
#define IMU_ERR_LIMIT 3 // 3 following I2C errors (15 ms) -> STOP




#endif /* CONFIG_H_ */
