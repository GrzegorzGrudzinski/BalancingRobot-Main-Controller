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

// Feedback frame (1 + 1 + 4 + 1 = 7 bytes)
typedef struct __attribute__((packed)) {
    uint8_t   startByte;    // 0xBB
    uint8_t   status;       //
    float     value;    // 
    uint8_t   checksum;     // XOR
} FeedbackPacket_t;

typedef enum {
    CMD_HELLO_MOTOR  = 0xFA, // HELLO FROM MOTOR
    CMD_HELLO_MASTER = 0xAF, // HELLO FROM MASTER
    CMD_INIT    = 0xDD,
    CMD_START   = 0x10,
    CMD_SET_VAL = 0x11, // Ustawienie prędkości
    CMD_STOP    = 0x55, // 0x20,
    // CMD_SET_TRQ = 0x40, 
    CMD_CLR_FLT = 0x50  // Kasowanie błędów
} CommandType_t;

uint8_t calculate_checksum(void* data, size_t length);
MotorPacket_t create_motor_packet(uint8_t command, float value1, float value2);
void send_motor_command(UART_HandleTypeDef *huart, uint8_t command, float mot1_value, float mot2_value);
void process_feedback(UART_HandleTypeDef *huart, uint8_t rx_msg_recieved, FeedbackPacket_t esc_data, uint8_t* error_flag);
const char* GetStateName(uint8_t s);

// void recieve_feedback(UART_HandleTypeDef *huart, FeedbackPacket_t* feedback, RX_Struct* rx_struct);
// void process_data(RX_Struct* rx_struct, const char *data, UART_HandleTypeDef *huart);

#endif // DUALFOC_COMMUNICATION_H