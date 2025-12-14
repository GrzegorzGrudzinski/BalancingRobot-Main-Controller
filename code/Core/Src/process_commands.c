#include "process_commands.h"
#include <stdio.h>
#include <string.h>

// Commands 
#define CMD_INIT     0x01
#define CMD_START    0x10
#define CMD_STOP     0x20
#define CMD_SET_RPM  0x30  // Sterowanie prędkością
// #define CMD_SET_TRQ  0x40  // Sterowanie momentem (lepsze do balansu!)
#define CMD_CLR_FLT  0x50  // Kasowanie błędów

// Data frame (1 + 1 + 4 + 1 = 7 bytes)
typedef struct __attribute__((packed)) {
    uint8_t startByte;   // 0xAA
    uint8_t command;     // CMD_XX
    float   value;       // 
    uint8_t checksum;    // 
} MotorPacket_t;

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
    HAL_UART_Transmit(huart, (uint8_t*)&packet, sizeof(MotorPacket_t), 100);
    // HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&pkt, sizeof(MotorPacket_t)); // OR IT
}

void process_data(RX_Struct* rx_struct, const char *data, size_t length, UART_HandleTypeDef *huart) {

    HAL_UART_Transmit(huart, (uint8_t *)data, strlen(data), 100);
    printf("Sent: %s", data); // %s wypisze też \n z data
    
    // Timeout
    uint32_t start_tick = HAL_GetTick();
    uint8_t timeout_occurred = 1;

 // Czekamy max 1000ms na flagę rx_struct->msg_received
    while((HAL_GetTick() - start_tick) < 1000)
    {
        if(rx_struct->msg_received == 1)
        {
            timeout_occurred = 0;
            break; // Wyszliśmy z pętli czekania, bo mamy dane
        }
    }

    // 3. Obsługa wyniku
    if(timeout_occurred == 0)
    {
        // Mamy odpowiedź!
        printf("STATUS: OK | Received: %s", rx_struct->rx_buffer);
        
        // Resetujemy bufor i flagę na następny raz
        rx_struct->msg_received = 0;
        rx_struct->rx_index = 0;
        memset(rx_struct->rx_buffer, 0, sizeof(rx_struct->rx_buffer));
    }
    else
    {
        printf("STATUS: TIMEOUT | [No response from ESC]\r\n");
    }

}