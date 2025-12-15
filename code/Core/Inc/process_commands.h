#ifndef PROCESS_COMMANDS_H
#define PROCESS_COMMANDS_H

#include "main.h"

#define CMD_INIT     0x01
#define CMD_START    0x10
#define CMD_STOP     0x20
#define CMD_SET_RPM  0x30       // Set speed
// #define CMD_SET_TRQ  0x40    // Set torque
#define CMD_CLR_FLT  0x50       // Acknowledge faults


// Data frame (1 + 1 + 4 + 1 = 7 bytes)
typedef struct __attribute__((packed)) {
    uint8_t startByte;   // 0xAA
    uint8_t command;     // CMD_XX
    float   value;       // 
    uint8_t checksum;    // XOR 
} MotorPacket_t;

typedef struct __attribute__((packed)) {
    uint8_t startByte;    // 0xBB
    uint8_t status;       //
    float   speed_rpm;    // 
    float   current_Iq;   // 
    uint8_t checksum;     // XOR
} FeedbackPacket_t;

uint8_t calculate_checksum(void* data, size_t length);
MotorPacket_t create_motor_packet(uint8_t command, float value);
void send_motor_command(UART_HandleTypeDef *huart, uint8_t command, float value);
// void recieve_feedback(UART_HandleTypeDef *huart, FeedbackPacket_t* feedback, RX_Struct* rx_struct);
// void process_data(RX_Struct* rx_struct, const char *data, UART_HandleTypeDef *huart);

#endif // PROCESS_COMMANDS_H