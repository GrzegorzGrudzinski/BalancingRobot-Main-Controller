/*
  robot_app.c
*/

#include "robot_app.h"
#include "config.h"
#include "controllers_config.h"
#include "control_state.h"
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

static MotorLinkInfo_t motor_link; // connection readings (seq / wiek / round trip)


static FeedbackPacket_t fb;       // data snapshot from ESC (once in every control loop)
static uint16_t hello_tx_seq = 0; // tx_seq ESP when sending HELLO_MASTER
static TelemetryPacket_t telemetry_packet;

Controller_t controller;
#define  CONTROL_STATE controller.observed_state
#define  TARGET_STATE controller.target_state

static RobotState_t robot_state = STATE_STANDBY;
static float target_angle = TARGET_ANGLE;
static float angle = 0.0f;
static float out_l = 0.0f;
static float out_r = 0.0f;

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
      // 1. Read and update data
      MotorFeedbackSnapshot(&fb, &motor_link);
      
      static uint8_t imu_err_streak = 0;
      if (updateControlState(&CONTROL_STATE, &fb) != 0) {
        if (imu_err_streak < UINT8_MAX) imu_err_streak++;
      } else {
        imu_err_streak = 0;
      }
    // TODO: updateTargetState(&TARGET_STATE, &pc_fb)

      angle = CONTROL_STATE.pitch;

      if ((robot_state == STATE_CONNECT || robot_state == STATE_INIT) &&
          (fb.status == CMD_ERROR || fb.error != ERR_OK)) {
        robot_state = STATE_ERROR;
      }
      switch (robot_state) {
        case STATE_STANDBY:{
            if (ButtonIsPressed()) {
              controlReset(&controller);
              out_l = 0.0f; out_r = 0.0f;

              send_motor_command(&MOT1_UART, CMD_START_INIT, 0, 0);
            }
            if (fb.status == CMD_START_INIT) {
              robot_state = STATE_CONNECT;
            }
          break;
        }

        case STATE_CONNECT:{ // check if the communication is ok - wait for the message from motor controller
          static uint16_t init_timer = 0;
          if (fb.status == CMD_HELLO_MOTOR) {
            if (++init_timer >= ONE_SEC/10) { // (100 ms)
              send_motor_command(&MOT1_UART, CMD_HELLO_MASTER, 0.0, 0.0);

              hello_tx_seq = fb.tx_seq; // newer frames = motor driver still in CONNECT

              init_timer = 0;
              robot_state = STATE_INIT;
            }
          }
          break;
        }

        case STATE_INIT:{ //  wait until everything is ready
          static uint16_t idle_timer = 0;
          static uint16_t upright_timer = 0;
          
          if (fb.status == CMD_HELLO_MOTOR && fb.tx_seq != hello_tx_seq) {
            idle_timer = 0;
            upright_timer = 0;
            robot_state = STATE_CONNECT;
            break;
          }

          // Robot has to be vertical to continue
          if (fabsf(angle) < START_ANGLE && imu_err_streak == 0) {
            if (upright_timer < UPRIGHT_TIME) upright_timer++;
          } else {
            upright_timer = 0;
          }

          if (upright_timer >= UPRIGHT_TIME && ++idle_timer >= ONE_SEC/50) { // every 20 ms
            send_motor_command(&MOT1_UART, CMD_INIT, 0.0f, 0.0f);
            idle_timer = 0;
          }
          if (fb.status == CMD_START) {
            controlReset(&controller); // update the controller
            idle_timer = 0;
            upright_timer = 0;
            robot_state = STATE_BALANCING;
          }
          break;
        }

        case STATE_BALANCING:{ // normal operation
          if (fabsf(angle) > MAX_ANGLE){// Out of range
            send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
            robot_state = STATE_FALLEN;
            break;
          }
          if (imu_err_streak >= IMU_ERR_LIMIT) { // imu error
            send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
            robot_state = STATE_ERROR;
            break;
          }
          if (motor_link.fb_age_us > MOTOR_FB_TIMEOUT) { 
            send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
            robot_state = STATE_ERROR;
            break;
          }

          // 2. Calculate 
          controlRun(&controller, CONTROL_DT, &out_l, &out_r);

          // 3. Send data to ESCs
          if (fb.status == CMD_ERROR || fb.error != ERR_OK) {
            robot_state = STATE_ERROR;
          }
          else if (fb.status == CMD_STOP) {
            robot_state = STATE_STANDBY;
          }
          else {
            send_motor_command(&MOT1_UART, CMD_SET_VAL, out_l, out_r);
          }
          break;
        }

        case STATE_FALLEN:{
          static uint16_t stop_timer = 0;
          out_l = 0.0f; out_r = 0.0f;
          send_motor_command(&MOT1_UART, CMD_STOP, 0, 0); // every cycle, until motor driver acks
          
          if (fb.status == CMD_STOP) {
            stop_timer = 0;
            controlReset(&controller);
            robot_state = STATE_STANDBY;
          }
          else if (++stop_timer >= STOP_ACK_TIMEOUT) {  // mot driver doesnt respond
            stop_timer = 0;
            robot_state = STATE_ERROR;
          }
          break;
        }

        case STATE_ERROR:{
          static uint8_t  clear_requested = 0;
          static uint16_t err_timer = 0;
          out_l = 0.0f; out_r = 0.0f;
            
          if (ButtonIsPressed()) {
            clear_requested = 1;
            controlReset(&controller);
          }
          if (++err_timer >= ONE_SEC/10) { // every 100 ms
            err_timer = 0;
            send_motor_command(&MOT1_UART, clear_requested ? CMD_CLR_FLT : CMD_STOP, 0.0f, 0.0f);
          }
          if (clear_requested && fb.status == CMD_CLR_FLT) {
            clear_requested = 0;
            err_timer = 0;
            robot_state = STATE_STANDBY;
          }
          break;
        }
        
        default:{
          send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
          controlReset(&controller);
          out_l = 0.0f; out_r = 0.0f;
          break;
        }
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
