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
#include "pc_command.h"
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
static float angle = 0.0f;
static float out_l = 0.0f;
static float out_r = 0.0f;

static uint16_t sat_timer = 0; // cycles with the output saturated in the same direction
static int8_t   sat_dir   = 0; // +1 / -1 / 0 in the last cycle
static uint8_t  target_sync = 0; // 1 = motor target_position not yet taken from a frame with wheel data

static uint16_t pc_rx_age = 0; // control cycles since the last valid PC frame
static uint8_t  pc_active = 0; // 1 = the PC sent a frame in this BALANCING run -> timeout armed


void RobotInit() {
    IMU_Init();
    controlSystemInit(&controller);
    targetReset(&TARGET_STATE, &CONTROL_STATE);

    diagInit();  // DWT cycle counter (time base in microseconds)
    diagResetCounters();
}

static void robotReset(void) {
    controlReset(&controller);
    targetReset(&TARGET_STATE, &CONTROL_STATE);
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

      
      // PC Commands
      PcCommand_t cmd;
      pcCommandBeginCycle();// only bytes received before this cycle (<= 255 B)
      while (pcCommandPoll(&cmd)) {
        pc_rx_age = 0;
        if (cmd.type == PC_CMD_SET_PARAM) {
          controlSetParam(&controller, cmd.param_id, cmd.value); // gains: any state
        }
        else if (robot_state == STATE_BALANCING && !target_sync) {
          pc_active = 1;
          pcProcessCMD(&cmd, &TARGET_STATE, &CONTROL_STATE);
        }
      }
      if (pc_rx_age < UINT16_MAX) pc_rx_age++;
      if (pc_active && pc_rx_age >= PC_TIMEOUT) { // link lost: stop where the robot is, keep balancing
        targetReset(&TARGET_STATE, &CONTROL_STATE);
        pc_active = 0;
      }

      angle = CONTROL_STATE.pitch;

      // check errors during startup
      if ((robot_state == STATE_STANDBY || robot_state == STATE_CONNECT || robot_state == STATE_INIT) &&
          (fb.status == CMD_ERROR || fb.error != ERR_OK)) {
        robot_state = STATE_ERROR;
      }
      switch (robot_state) {
        case STATE_STANDBY:{
            if (ButtonIsPressed()) {
              robotReset();
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
          
          if (ButtonIsPressed()) {  // cancel the start
            send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
            idle_timer = 0; upright_timer = 0;
            robot_state = STATE_STOP;   // waits for the CMD_STOP ack, then STANDBY
            break;
          }

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
            robotReset(); // update the controller
            target_sync = 1; // wait for real motor position data
            pc_active = 0;   // every BALANCING run starts without PC control
            idle_timer = 0;
            upright_timer = 0;
            sat_timer = 0; sat_dir = 0; // reset saturation history
            robot_state = STATE_BALANCING;
          }
          break;
        }

        case STATE_BALANCING:{ // normal operation
          if (fabsf(angle) > MAX_ANGLE ){ // Out of range
            send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
            robot_state = STATE_FALLEN;
            break;
          }
          if (ButtonIsPressed()){ // manual stop
            send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
            robot_state = STATE_STOP;
            break;
          }
          if (imu_err_streak >= IMU_ERR_LIMIT) { // imu error
            send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
            robot_state = STATE_ERROR;
            break;
          }
          if (motor_link.fb_age_us > MOTOR_FB_TIMEOUT) {  // Motor feedback timeout
            send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
            robot_state = STATE_ERROR;
            break;
          }
          // robot in the air: wheels too fast or pitch output saturated in one direction for too long
          float pitch_out = controller.debug.pitch_output; // pitch from the prev cycle
          int8_t dir = (pitch_out >= CTRL_MAX_OUT) ? 1 : (pitch_out <= -CTRL_MAX_OUT) ? -1 : 0;
          if (dir != 0 && dir == sat_dir) {
            if (sat_timer < UINT16_MAX) sat_timer++;
          } else {
            sat_timer = 0;
          }
          sat_dir = dir;

          if (fabsf(CONTROL_STATE.velocity) > MAX_VELOCITY || sat_timer >= SAT_TIMEOUT) {
            send_motor_command(&MOT1_UART, CMD_STOP, 0, 0);
            robot_state = STATE_ERROR;
            break;
          }

          // wait for the first frame with real wheel positions
          if (target_sync && fb.status == CMD_SET_VAL) {
            robotReset();
            target_sync = 0;
          }

          // 2. Calculate 
          controlRun(&controller, CONTROL_DT, &out_l, &out_r);

          // 3. Check motor status and send data to ESCs
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

        case STATE_FALLEN:
        case STATE_STOP:{
          static uint16_t stop_timer = 0;
          out_l = 0.0f; out_r = 0.0f;
          send_motor_command(&MOT1_UART, CMD_STOP, 0, 0); // every cycle, until motor driver acks
          
          if (fb.status == CMD_STOP) {
            stop_timer = 0;
            robotReset();
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
            robotReset();
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
          robotReset();
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

