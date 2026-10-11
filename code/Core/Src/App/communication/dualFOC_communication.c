/*
 dualFOC_communication.c
*/

#include "dualFOC_communication.h"
#include <stdio.h>
#include <string.h>


static uint8_t rxState_flag = 0; // 0 -looking for start byte, 1 - receiving data
// volatile uint32_t uart_error_count = 0;

void processRxReset(void) { rxState_flag = 0; }


uint8_t calculate_checksum(void* data, size_t length) {
    if(length==0 ) return 0;
    
    uint8_t checksum = 0;
    uint8_t* byte = (uint8_t*)data;

    for (size_t i = 0; i < length-1; i++) {
        checksum ^= byte[i];
    }

    return checksum;
}

MotorPacket_t create_motor_packet(uint8_t command, float value1, float value2) {
    MotorPacket_t packet;
    packet.startByte = 0xAA;
    packet.command = command;
    packet.value1 = value1;
    packet.value2 = value2;
    packet.checksum = calculate_checksum(&packet, sizeof(MotorPacket_t));
    return packet;
}

void send_motor_command(UART_HandleTypeDef *huart, uint8_t command, float mot1_value, float mot2_value) {
    static MotorPacket_t packet;
    
    // Only send if UART is ready
    uint8_t state = HAL_UART_GetState(huart);
    if ( (state == HAL_UART_STATE_READY || state == HAL_UART_STATE_BUSY_RX) ) {
        packet = create_motor_packet(command, mot1_value,mot2_value);
        HAL_UART_Transmit_DMA(huart, (uint8_t*)&packet, sizeof(MotorPacket_t));
    }
}

///

FeedbackPacket_t motor_feedback_data = {0}; 
uint8_t rxRawBuffer[sizeof(FeedbackPacket_t)]; 
volatile uint8_t rx_msg_recieved = 0;

void processRxPacket (UART_HandleTypeDef *huart) {
    if (rxState_flag == 0) {
      if (rxRawBuffer[0] == 0xBB) { // Start byte found
        rxState_flag = 1;

        HAL_UART_Receive_IT(huart, &rxRawBuffer[1], sizeof(FeedbackPacket_t) - 1); // Receive rest of the packet
      }
      else {
        HAL_UART_Receive_IT(huart, rxRawBuffer, 1); 
      }
    }
    else if (rxState_flag == 1) {
      // Full packet received
      FeedbackPacket_t* recieved_pkt = (FeedbackPacket_t*)rxRawBuffer;

      //calcuate checksum and verify
      uint8_t checksum = calculate_checksum(rxRawBuffer, sizeof(FeedbackPacket_t));
      if (recieved_pkt->checksum == checksum) {
        // Valid packet
        motor_feedback_data = *recieved_pkt;
        rx_msg_recieved = 1;
      }

      rxState_flag = 0; // Reset state to look for next packet
      // Restart reception for next packet
      HAL_UART_Receive_IT(huart, rxRawBuffer, 1);
    }
}