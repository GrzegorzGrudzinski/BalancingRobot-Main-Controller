/*
  robot_app.c
*/

#include "robot_app.h"
#include "config.h"
#include "controllers_config.h"
// #include "control_state.h"
#include "control.h"
#include "imu_handler.h"
#include "IO/button.h"
#include "dualFOC_communication.h"
#include "pc_telemetry.h"
#include "diag.h"

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>

////



extern UART_HandleTypeDef MOT1_UART;
extern volatile uint8_t loop_flag;
// extern FeedbackPacket_t fb; // Data from ESC
static MotorLinkInfo_t motor_link; // connection readings (seq / wiek / round trip)
static FeedbackPacket_t fb;        // data snapshot from ESC (once in every control loop)

Controller_t controller;
// ControlState_t control_state;
#define  CONTROL_STATE controller.observed_state
#define  TARGET_STATE controller.target_state

static RobotState_t robot_state = STATE_STANDBY;
static float target_angle = TARGET_ANGLE;
static float angle = 0.0f;
static float out_l = 0.0f;
static float out_r = 0.0f;

static TelemetryPacket_t telemetry_packet;

////

// static const char* GetStateName(RobotState_t s) {
//   switch(s) {
//     case STATE_INIT: return "INIT";
//     case STATE_IDLE: return "IDLE";
//     case STATE_BALANCING: return "BALANCING";
//     case STATE_FALLEN: return "FALLEN";
//     case STATE_ERROR: return "ERROR";
//     default: return "UNKNOWN";
//   }
// }

void RobotInit() {
    IMU_Init();
    controlSystemInit(&controller);
    TARGET_STATE.target_pitch = target_angle;
    // TARGET_STATE.target_yaw = target_yaw;

    Diag_Init();  // DWT cycle counter (time base in microseconds)
    memset(&diag, 0, sizeof(diag)); // Initialization-phase counters are not meaningful
}

TelemetryPacket_t* RobotUpdate() {
    if (loop_flag) {    
      loop_flag = 0;
      // 1. Read IMU data
      MotorFeedbackSnapshot(&fb, &motor_link);
      updateControlState(&CONTROL_STATE, &fb);
      angle = CONTROL_STATE.pitch;

      if ((robot_state == STATE_INIT || robot_state == STATE_IDLE) &&
          (fb.status == CMD_ERROR || fb.error != ERR_OK)) {
        robot_state = STATE_ERROR;
      }
      switch (robot_state) {
        case STATE_STANDBY:
            if (ButtonIsPressed()) {
              controlReset(&controller);
              out_l = 0.0f; out_r = 0.0f;

              send_motor_command(&MOT1_UART, CMD_START_INIT, 0, 0);
            }
            if (fb.status == CMD_START_INIT) {
              robot_state = STATE_INIT;
            }
          break;

        case STATE_INIT: // check if the communication is ok - wait for the message from motor controller
          if (fb.status == CMD_HELLO_MOTOR) {
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
          if (++idle_timer >= ONE_SEC/10) { // (send command every 200 ms)
            send_motor_command(&MOT1_UART, CMD_INIT, 0.0, 0.0);
            idle_timer = 0;
          }
          if (fb.status == CMD_START) {
            
            TARGET_STATE.target_position = CONTROL_STATE.position;

            robot_state = STATE_BALANCING;
            idle_timer = 0;
          }
          break;

        case STATE_BALANCING: // normal operation
          if (fabsf(angle) > MAX_ANGLE){// Out of range
            robot_state = STATE_FALLEN;
            break;
          }

          // 2. Calculate 
          controlRun(&controller, &out_l, &out_r);

          // 3. Send data to ESCs
          send_motor_command(&MOT1_UART, CMD_SET_VAL, out_l, out_r);
          
          if (fb.status == CMD_ERROR || fb.error != ERR_OK) {
            robot_state = STATE_ERROR;
          }
          else if (fb.status == CMD_STOP) {
            robot_state = STATE_STANDBY;
          }
          break;

        case STATE_FALLEN:
          send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
          controlReset(&controller);
          out_l = 0.0f; out_r = 0.0f;

          robot_state = STATE_STANDBY;
          break;

        case STATE_ERROR:
            if (ButtonIsPressed()) {
              send_motor_command(&MOT1_UART, CMD_CLR_FLT, 0.0, 0.0);
              controlReset(&controller);
              out_l = 0.0f; out_r = 0.0f;
            }

            if (fb.status == CMD_CLR_FLT) {
              robot_state = STATE_STANDBY;
            }
          break;

        default:
          send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
          controlReset(&controller);
          out_l = 0.0f; out_r = 0.0f;
          break;
      }       

      ButtonClear(); // clear button memory

      // TODO: add config info at startup (eg. return startup-specific val)
      create_telemetry_packet(
        &telemetry_packet, 
        &controller, 
        &fb, 
        &motor_link, 
        robot_state, 
        out_l, out_r
      );
      return &telemetry_packet;
    } // if (loop_flag)
    return NULL; // brak nowego cyklu -> nie ma czego wysyłać
}
