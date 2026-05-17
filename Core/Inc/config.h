#ifndef CONFIG_H_
#define CONFIG_H_

//======================
//  Hardware
//======================

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

//======================
//  Timers
//======================

#define TIM_PERIOD_REG_VAL ((htim3.Init.Period+1)/1000) // ms

#define LOOP_TIME   5  // 5 ms / 200 Hz
#define ONE_SEC     ( 1000 / LOOP_TIME ) // 
#define HALF_SEC    (  500 / LOOP_TIME ) // 

#define DEBUG_TIME 500

//======================
//  State machine
//======================

typedef enum {
    STATE_STANDBY,
    STATE_INIT,
    STATE_IDLE,
    STATE_BALANCING,
    STATE_FALLEN,
    STATE_ERROR
} RobotState_t;

extern RobotState_t robot_state = STATE_STANDBY;

//======================
//  Balancing
//======================
extern float target_angle = 00.0f;

#define MAX_ANGLE   60.0f

#define PID_KP  0.5
#define PID_KI  0.0
#define PID_KD  0.0

#define PID_MAX_OUT 1.0
#define PID_MAX_I 1.0

//======================
//  Calibration
//======================
#define CALIB_WARMUP_TIME ( ONE_SEC*3 )
#define CALIB_TIME         ONE_SEC

#endif /* CONFIG_H_ */
