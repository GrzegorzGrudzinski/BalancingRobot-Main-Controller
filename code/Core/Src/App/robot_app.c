#include "robot_app.h"
#include "config.h"
#include "pid.h"
#include "button.h"
#include "imu_handler.h"
#include "dualFOC_communication.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>

const char* GetStateName(RobotState_t s) {
  switch(s) {
    case STATE_INIT: return "INIT";
    case STATE_IDLE: return "IDLE";
    case STATE_BALANCING: return "BALANCING";
    case STATE_FALLEN: return "FALLEN";
    case STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

PID_t pid;

void RobotInit() {
    IMU_Init(&pid);
}

extern UART_HandleTypeDef MOT1_UART;
extern volatile uint8_t loop_flag;
extern FeedbackPacket_t motor_feedback_data; // Data from ESC1

static RobotState_t robot_state = STATE_STANDBY;
static float target_angle = TARGET_ANGLE;
static float angle = 0.0f;
static float output = 0.0f;

void RobotUpdate() {
    if (loop_flag) {    
      loop_flag = 0;
      // 1. Read IMU data
      angle = IMU_GetAngle();
      // if (fabsf(angle) <= 0.05) { // Target
      //   angle = 0;
      // }

      switch (robot_state) {
        case STATE_STANDBY:
            if (ButtonIsPressed()) {
              pid_reset(&pid, angle);
              output = 0.0f;

              send_motor_command(&MOT1_UART, CMD_START_INIT, 0, 0);
            }
            if (motor_feedback_data.status == CMD_START_INIT) {
              robot_state = STATE_INIT;
            }
          break;

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
          if (fabsf(angle) > MAX_ANGLE){// Out of range
            robot_state = STATE_FALLEN;
            break;
          }

          // 2. Calculate PID
          output = run_pid(&pid, angle, target_angle);      // convert values to motor commands (angle -> linear vel)
          // 3. Send data to ESCs
          send_motor_command(&MOT1_UART, CMD_SET_VAL, output, output);

          if (motor_feedback_data.status == CMD_ERROR) {
            robot_state = STATE_ERROR;
          }
          else if (motor_feedback_data.status == CMD_STOP) {
            robot_state = STATE_STANDBY;
          }
          break;

        case STATE_FALLEN:
          send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
          pid_reset(&pid, angle);
          output = 0.0f;

          robot_state = STATE_STANDBY;
          break;

        case STATE_ERROR:
            if (ButtonIsPressed()) {
              send_motor_command(&MOT1_UART, CMD_CLR_FLT, 0.0, 0.0);
              pid_reset(&pid, angle);
              output = 0.0f;
            }

            if (motor_feedback_data.status == CMD_CLR_FLT) {
              robot_state = STATE_STANDBY;
            }
          break;

        default:
          send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
          pid_reset(&pid, angle);
          output = 0.0f;
          break;
      }       

      ButtonClear(); // clear button memory
    }
}

void RobotLog() {
    static uint32_t last_debug_time = 0;

    if (HAL_GetTick() - last_debug_time >= DEBUG_TIME) {
      last_debug_time = HAL_GetTick();
      printf("State: %s\tAngle: %.2f\tOutput2: %.2f\n\r", GetStateName(robot_state), angle, output);
    }
}