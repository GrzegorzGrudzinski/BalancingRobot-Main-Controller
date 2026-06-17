#ifndef DUALFOC_COMMUNICATION_H
#define DUALFOC_COMMUNICATION_H

#include "main.h"

// Data frame (1 + 1 + 4 + 4 + 1 = 11 bytes)
typedef struct __attribute__((packed)) {
    uint8_t startByte;   // 0xAA
    uint8_t command;     // CMD_XX
    float   value1;       // 
    float   value2;       // 
    uint8_t checksum;    // XOR 
} MotorPacket_t;

// Feedback frame (1 + 1 + 4 + 4 + 1 = 11 bytes)
typedef struct __attribute__((packed)) {
    uint8_t   startByte;    // 0xBB
    uint8_t   status;       //
    float value1;
    float value2;
    uint8_t   checksum;     // XOR
} FeedbackPacket_t; // from motor

typedef enum {
    CMD_HELLO_MOTOR  = 0xFA, // HELLO FROM MOTOR
    CMD_HELLO_MASTER = 0xAF, // HELLO FROM MASTER
    CMD_START_INIT   = 0xDD, // Start init on motor driver
    CMD_INIT         = 0x01,
    CMD_START        = 0x10,
    CMD_SET_VAL      = 0x11, // Ustawienie prędkości
    CMD_STOP         = 0x20, // 0x20,
    CMD_CLR_FLT      = 0x50, // Kasowanie błędów
    CMD_RESET        = 0x51, // Reset     
    CMD_ERROR        = 0x60  // Error
} CommandType_t;

extern FeedbackPacket_t motor_feedback_data;
extern uint8_t rxRawBuffer[];

uint8_t calculate_checksum(void* data, size_t length);
MotorPacket_t create_motor_packet(uint8_t command, float value1, float value2);
void send_motor_command(UART_HandleTypeDef *huart, uint8_t command, float mot1_value, float mot2_value);
void processRxPacket (UART_HandleTypeDef *huart);


#endif // DUALFOC_COMMUNICATION_H