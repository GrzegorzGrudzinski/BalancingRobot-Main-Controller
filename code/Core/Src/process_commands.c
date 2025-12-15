#include "process_commands.h"
#include <stdio.h>
#include <string.h>

// Commands 
// CMD_INIT     0x01
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
    MotorPacket_t packet = create_motor_packet(command, value);
    // HAL_UART_Transmit(huart, (uint8_t*)&packet, sizeof(MotorPacket_t), 100);
    HAL_UART_Transmit_DMA(huart, (uint8_t*)&packet, sizeof(MotorPacket_t)); // OR IT
}

