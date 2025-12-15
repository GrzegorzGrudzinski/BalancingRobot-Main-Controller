#ifndef PROCESS_COMMANDS_H
#define PROCESS_COMMANDS_H

#include "main.h"

#define CMD_INIT     0x01 
#define CMD_HELLO    0x0FA
#define ESC_HELLO    0xAF
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


// Feedback frame (1 + 1 + 4 + 4 + 1 = 11 bytes)
typedef struct __attribute__((packed)) {
    uint8_t startByte;    // 0xBB
    uint8_t status;       //
    float   speed_rpm;    // 
    float   current_Iq;   // 
    uint8_t checksum;     // XOR
} FeedbackPacket_t;

// Motor States from Motor Control Library
typedef enum
{
  ICLWAIT = 12,         /**< The system is waiting for ICL deactivation. Is not possible
                          *  to run the motor if ICL is active. While the ICL is active
                          *  the state is forced to #ICLWAIT; when ICL become inactive the
                          *  state is moved to #IDLE. */
  IDLE = 0,             /**< The state machine remains in this state as long as the
                          *  application is not controlling the motor. This state is exited
                          *  when the application sends a motor command or when a fault occurs. */
  ALIGNMENT = 2,        /**< The encoder alignment procedure that will properly align the
                          *  the encoder to a set mechanical angle is being executed. */
  CHARGE_BOOT_CAP = 16, /**< The gate driver boot capacitors are being charged. */
  OFFSET_CALIB = 17,    /**< The offset of motor currents and voltages measurement cirtcuitry
                          *  are being calibrated. */
  START = 4,            /**< The motor start-up is procedure is being executed. */
  SWITCH_OVER = 19,     /**< Transition between the open loop startup procedure and
                          *  closed loop operation */
  RUN = 6,              /**< The state machien remains in this state as long as the
                          *  application is running (controlling) the motor. This state
                          *  is exited when the application isues a stop command or when
                          *  a fault occurs. */
  STOP = 8,             /**< The stop motor procedure is being executed. */
  FAULT_NOW = 10,       /**< The state machine is moved from any state directly to this
                          *  state when a fault occurs. The next state can only be
                          *  #FAULT_OVER. */
  FAULT_OVER = 11,       /*!< The state machine transitions from #FAULT_NOW to this state
                          *   when there is no active fault condition anymore. It remains
                          * in this state until either a new fault condition occurs (in which
                          * case it goes back to the #FAULT_NOW state) or the application
                          * acknowledges the faults (in which case it goes to the #IDLE state).
                          */
  WAIT_STOP_MOTOR = 20,  /**< Temporisation to make sure the motor is stopped. */
  OTF_DETECTION = 21,  /**< Temporisation to make sure the motor is stopped. */
  OTF_BRAKE = 22  /**< Temporisation to make sure the motor is stopped. */
} Motor_State_t;

uint8_t calculate_checksum(void* data, size_t length);
MotorPacket_t create_motor_packet(uint8_t command, float value);
void send_motor_command(UART_HandleTypeDef *huart, uint8_t command, float value);
const char* GetStateName(uint8_t s);

// void recieve_feedback(UART_HandleTypeDef *huart, FeedbackPacket_t* feedback, RX_Struct* rx_struct);
// void process_data(RX_Struct* rx_struct, const char *data, UART_HandleTypeDef *huart);

#endif // PROCESS_COMMANDS_H