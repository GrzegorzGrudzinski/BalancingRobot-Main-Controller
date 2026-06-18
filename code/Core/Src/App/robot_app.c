#include "robot_app.h"
#include "config.h"
#include "controllers.h"
#include "control.h"
#include "button.h"
#include "imu_handler.h"
#include "dualFOC_communication.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>

////



extern UART_HandleTypeDef MOT1_UART;
extern volatile uint8_t loop_flag;
extern FeedbackPacket_t motor_feedback_data; // Data from ESC

Controller_t controller;
ControlState_t control_state;

static RobotState_t robot_state = STATE_STANDBY;
static float target_angle = TARGET_ANGLE;
static float angle = 0.0f;
static float out_l = 0.0f;
static float out_r = 0.0f;

////

static const char* GetStateName(RobotState_t s) {
  switch(s) {
    case STATE_INIT: return "INIT";
    case STATE_IDLE: return "IDLE";
    case STATE_BALANCING: return "BALANCING";
    case STATE_FALLEN: return "FALLEN";
    case STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

static void updateControlState(ControlState_t *state) {
    state->pitch      = IMU_GetAngle();
    // state->pitch_rate = IMU_GetPitchRate();
    // state->yaw_rate   = IMU_GetYawRate();

    // state->left_wheel_pos  = motor_feedback_data.motor1_pos;
    // state->right_wheel_pos = motor_feedback_data.motor2_pos;
    
    // float left_vel  = motor_feedback_data.motor1_vel;
    // float right_vel = motor_feedback_data.motor2_vel; 
    
    // state->velocity = (left_vel + right_vel) / 2.0f;
}

void RobotInit() {
    IMU_Init();

    controlSystemInit(&controller);

    controller.target_pitch = target_angle;
}

void RobotUpdate() {
    if (loop_flag) {    
      loop_flag = 0;
      // 1. Read IMU data
      updateControlState(&control_state);
      angle = control_state.pitch;

      switch (robot_state) {
        case STATE_STANDBY:
            if (ButtonIsPressed()) {
              controlReset(&controller, &control_state);
              out_l = 0.0f; out_r = 0.0f;

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

          // 2. Calculate 
          controlRun(&controller, &control_state, &out_l, &out_r);

          // 3. Send data to ESCs
          send_motor_command(&MOT1_UART, CMD_SET_VAL, out_l, out_r);

          if (motor_feedback_data.status == CMD_ERROR) {
            robot_state = STATE_ERROR;
          }
          else if (motor_feedback_data.status == CMD_STOP) {
            robot_state = STATE_STANDBY;
          }
          break;

        case STATE_FALLEN:
          send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
          controlReset(&controller, &control_state);
          out_l = 0.0f; out_r = 0.0f;

          robot_state = STATE_STANDBY;
          break;

        case STATE_ERROR:
            if (ButtonIsPressed()) {
              send_motor_command(&MOT1_UART, CMD_CLR_FLT, 0.0, 0.0);
              controlReset(&controller, &control_state);
              out_l = 0.0f; out_r = 0.0f;
            }

            if (motor_feedback_data.status == CMD_CLR_FLT) {
              robot_state = STATE_STANDBY;
            }
          break;

        default:
          send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
          controlReset(&controller, &control_state);
          out_l = 0.0f; out_r = 0.0f;
          break;
      }       

      ButtonClear(); // clear button memory
    }
}

void RobotLog() {
    static uint32_t last_debug_time = 0;

    if (HAL_GetTick() - last_debug_time >= DEBUG_TIME) {
      last_debug_time = HAL_GetTick();
      printf("State: %s\tAngle: %.2f\tOutput1: %.2f\tOutput2: %.2f\n\r", GetStateName(robot_state), angle, out_l, out_r);
    }
}