#ifndef DUALFOC_COMMUNICATION_H
#define DUALFOC_COMMUNICATION_H

#include "main.h"

// Data frame (1 + 1 + 4 + 4 + 1 = 11 bytes)
typedef struct __attribute__((packed)) {
    uint8_t startByte;   // 0xAA
    uint8_t command;     // CMD_XX
    float   value1;      // 
    float   value2;      // 
    uint8_t checksum;    // XOR 
} MotorPacket_t;

// Feedback frame (1 + 1 + 6*4 + 1 = 27 bytes)
typedef struct __attribute__((packed)) {
    uint8_t   startByte; // 0xBB
    uint8_t   status;    //
    float motor1_vel;    // mot 1 speed
    float motor1_pos;    // mot 1 position
    float motor1_A;      // mot 1 current
    float motor2_vel;    // mot 2 speed
    float motor2_pos;    // mot 2 position
    float motor2_A;      // mot 2 current
    uint8_t checksum;    // XOR
} FeedbackPacket_t; // from motor

typedef enum CommandType_t{
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

typedef enum {
    ERR_OK,
    ERR_INIT,
    ERR_COMMUNICATION,
    ERR_INV_COMMAND,
    ERR_IS_NAN,
    ERR_COM_TIMEOUT,
    ERR_MOTOR_STALL,   
    ERR_DESYNC,        
    ERR_OPPOSITE_SPIN    
} error_states_t;

extern FeedbackPacket_t motor_feedback_data;
extern uint8_t rxRawBuffer[];

uint8_t calculate_checksum(void* data, size_t length);
MotorPacket_t create_motor_packet(uint8_t command, float value1, float value2);
void send_motor_command(UART_HandleTypeDef *huart, uint8_t command, float mot1_value, float mot2_value);
void processRxPacket (UART_HandleTypeDef *huart);
void processRxReset(void);

#endif // DUALFOC_COMMUNICATION_H