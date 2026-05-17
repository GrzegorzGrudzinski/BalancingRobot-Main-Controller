/*
 dualFOC_communication.c
*/

#include "dualFOC_communication.h"
#include <stdio.h>
#include <string.h>

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
