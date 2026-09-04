/*
    pc_telemetry.c
*/

#include "pc_telemetry.h"
#include "dualFOC_communication.h" // for checksum calc 
#include "config.h"

#include "stdint.h"
#include "usbd_cdc_if.h"
    // printf("State: %s\tAngle: %.2f\tOutput1: %.2f\tOutput2: %.2f\n\r", GetStateName(robot_state), angle, out_l, out_r);
    // printf("State: %s\tERR: %s\tCMD_RCVD: %x\tAngle1: %.2f\tAngle2: %.2f\n\r", 

TelemetryPacket_t create_telemetry_packet 
  ( uint8_t robot_status, uint8_t motor_status,
    float angle_pitch, float angle_yaw,
    float mot1_vel, float mot1_pos, float mot1_A,
    float mot2_vel, float mot2_pos, float mot2_A )
{
    TelemetryPacket_t packet;
    packet.startByte = 0xCC;
    packet.robot_status = robot_status;
    packet.motor_status = motor_status;
    packet.mot1_vel = mot1_vel;
    packet.mot1_pos = mot1_pos;
    packet.mot1_A   = mot1_A;
    packet.mot2_vel = mot2_vel;
    packet.mot2_pos = mot2_pos;
    packet.mot2_A   = mot2_A;
    packet.checksum = calculate_checksum(&packet, sizeof(TelemetryPacket_t));

    return packet;
}

static void send_telemetry(TelemetryPacket_t* packet) {

    CDC_Transmit_FS((uint8_t*)packet, sizeof(TelemetryPacket_t));
}




void RobotLog(TelemetryPacket_t* packet) {
    if (packet == NULL) return;
    static uint32_t last_debug_time = 0;
    if (HAL_GetTick() - last_debug_time >= DEBUG_TIME) {
      last_debug_time = HAL_GetTick();
      send_telemetry(packet);
    }
}


// static const char* getErrorText(error_states_t state)
// {
//     switch (state) {
//         case ERR_OK:            return "OK";
//         case ERR_INIT:          return "INIT_FAILED";
//         case ERR_COMMUNICATION: return "COMMUNICATION_ERROR";
//         case ERR_INV_COMMAND:   return "INVALID_COMMAND";
//         case ERR_IS_NAN:        return "TARGET_IS_NAN";
//         case ERR_COM_TIMEOUT:   return "TIMEOUT";
//         case ERR_MOTOR_STALL:   return "MOTOR_STALLED";  
//         case ERR_DESYNC:        return "MOTORS_DESYNC"; 
//         case ERR_OPPOSITE_SPIN: return "OPPOSITE_SPIN"; 
//         default:                return "UNKNOWN_ERROR";
//     }
// }

// void RobotLog() {
//     static uint32_t last_debug_time = 0;

//     if (HAL_GetTick() - last_debug_time >= DEBUG_TIME) {
//       last_debug_time = HAL_GetTick();
//       if (motor_feedback_data.status != CMD_ERROR) {
//         printf("State: %s\tAngle: %.2f\tOutput1: %.2f\tOutput2: %.2f\n\r", GetStateName(robot_state), angle, out_l, out_r);
//       }
//       else {
//         printf("State: %s\tERR: %s\tCMD_RCVD: %x\tAngle1: %.2f\tAngle2: %.2f\n\r", 
//           GetStateName(robot_state), 
//           getErrorText((error_states_t)motor_feedback_data.motor1_vel), 
//           (unsigned int)motor_feedback_data.motor2_vel, 
//           motor_feedback_data.motor1_pos, 
//           motor_feedback_data.motor2_pos);
//       }
//     }
// }