#include "process_commands.h"
#include <stdio.h>
#include <string.h>


// extern volatile FeedbackPacket_t esc1_data; // Data from ESC1
// extern uint8_t rx1RawBuffer[sizeof(FeedbackPacket_t)]; // Raw buffer for receiving data
// extern volatile uint8_t rx1_msg_recieved; // message received flag

// extern volatile FeedbackPacket_t esc2_data; // Data from ESC1
// extern uint8_t rx2RawBuffer[sizeof(FeedbackPacket_t)]; // Raw buffer for receiving data
// extern volatile uint8_t rx2_msg_recieved; // message received flag


// Commands 
// CMD_INIT     0x01
// CMD_HELLO    0xFA
// CMD_START    0x10
// CMD_STOP     0x20
// CMD_SET_RPM  0x30  // Set speed
// CMD_SET_TRQ  0x40  // Set torque
// CMD_CLR_FLT  0x50  // Acknowledge faults

uint8_t calculate_checksum(void* data, size_t length) {
    uint8_t checksum = 0;
    uint8_t* byte = (uint8_t*)data;

    for (size_t i = 0; i < length-1; i++) {
        checksum ^= byte[i];
    }

    return checksum;
}


MotorPacket_t create_motor_packet(uint8_t command, float value) {
    MotorPacket_t packet;
    packet.startByte = 0xAA;
    packet.command = command;
    packet.value = value;
    packet.checksum = calculate_checksum(&packet, sizeof(MotorPacket_t));
    return packet;
}

void send_motor_command(UART_HandleTypeDef *huart, uint8_t command, float value) {
    static MotorPacket_t packet1;
    static MotorPacket_t packet2;
    MotorPacket_t* p_packet = NULL;
    
    if (huart->Instance == USART2) {
        p_packet = &packet1;
    } else {
        p_packet = &packet2;
    }
    *p_packet = create_motor_packet(command, value);
    
    // Only send if UART is ready
    uint8_t state = HAL_UART_GetState(huart);
    if ( (state == HAL_UART_STATE_READY || state == HAL_UART_STATE_BUSY_RX) ) {
        HAL_UART_Transmit_DMA(huart, (uint8_t*)p_packet, sizeof(MotorPacket_t));
    }
}


void process_feedback(UART_HandleTypeDef *huart, uint8_t rx_msg_recieved, FeedbackPacket_t esc_data, uint8_t* error_flag) {
  if (rx_msg_recieved) {
      //Check for errors from ESC
      if (esc_data.speed_rpm > 4000.0f || esc_data.speed_rpm < -4000.0f || 
        esc_data.current_Iq > 30.0f || esc_data.current_Iq < -30.0f)
      {
          // Handle error

          // send_motor_command_dma(huart, CMD_STOP, 0);
          send_motor_command(huart, CMD_STOP, 0);
          error_flag = 1; // Set error flag
      }

      switch (esc_data.status) {
        case FAULT_NOW:
          // Handle fault state
          error_flag = 1; // Set error flag
          break;
        case FAULT_OVER:
          // Handle fault state
          send_motor_command(huart, CMD_CLR_FLT, 0);
          break;
        case IDLE:
          // Restart motor if in IDLE
          send_motor_command(huart, CMD_START, 0);
          break;

        // case RUN:
        //   // Normal operation
        //   break;
        // case START:
        //   // Motor is starting
        //   break;
        // case STOP:
        //   // Motor is stopping
        //   break;

        case OFFSET_CALIB:
          // Calibration state
          // HAL_Delay(200); // Wait for calibration
          break;
        case CHARGE_BOOT_CAP:
          // Charging capacitors
          // HAL_Delay(200); // Wait for charging
          break;
        case ALIGNMENT:
          // Alignment state
          break;
        case SWITCH_OVER:
          // Switching from open loop to closed loop
          break;
        case ICLWAIT:
          // Waiting for ICL to clear
          break;
        case WAIT_STOP_MOTOR:
          // Waiting for motor to stop
          break;
        case OTF_DETECTION:
          // On-the-fly detection
          break;
        case OTF_BRAKE:
          // On-the-fly braking
          break;

        default:
          break;
      }
      rx_msg_recieved = 0;
    }
}










// Convert motor state enum to string for debugging
const char* GetStateName(uint8_t s) {
    switch(s) {
        case ICLWAIT: return "ICLWAIT";
        case IDLE: return "IDLE";
        case ALIGNMENT: return "ALIGNMENT";
        case CHARGE_BOOT_CAP: return "CHARGE_BOOT_CAP";
        case OFFSET_CALIB: return "OFFSET_CALIB";
        case START: return "START";
        case SWITCH_OVER: return "SWITCH_OVER";
        case RUN: return "RUN";
        case STOP: return "STOP";
        case FAULT_NOW: return "FAULT_NOW";
        case FAULT_OVER: return "FAULT_OVER";
        case WAIT_STOP_MOTOR: return "WAIT_STOP_MOTOR";
        case OTF_DETECTION: return "OTF_DETECTION";
        case OTF_BRAKE: return "OTF_BRAKE";
        
        default: return "OTHER";
    }
}
