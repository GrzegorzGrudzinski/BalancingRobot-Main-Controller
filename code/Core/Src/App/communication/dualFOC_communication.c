/*
 dualFOC_communication.c
*/

#include "dualFOC_communication.h"
#include "diag.h"

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

// ---- connection stats ----
static uint16_t cmd_seq = 0;                  // seq of last processed command
static uint32_t cmd_tx_cycles[256];           // DWT time of every last cmd (index = seq & 0xFF)
static volatile uint32_t fb_rx_cycles = 0;    // DWT time last poprawnego feedback
static volatile uint16_t link_rtt_us = 0;     // last time command -> feedback
static uint16_t last_ack_seq = 0;
static uint16_t last_esp_tx_seq = 0;
static uint8_t  have_esp_tx_seq = 0;

MotorPacket_t create_motor_packet(uint8_t command, float value1, float value2, uint16_t seq) {
    MotorPacket_t packet;
    packet.startByte = 0xAA;
    packet.command = command;
    packet.seq = seq;
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
        cmd_seq++;
        cmd_tx_cycles[cmd_seq & 0xFF] = Diag_Cycles();
      
        packet = create_motor_packet(command, mot1_value,mot2_value, cmd_seq);
        HAL_UART_Transmit_DMA(huart, (uint8_t*)&packet, sizeof(MotorPacket_t));
    }
    else {
        diag.cmd_drop_count++; // UART busy - cmd not sent
    }
}

///

FeedbackPacket_t motor_feedback_data = {0}; 
uint8_t rxRawBuffer[sizeof(FeedbackPacket_t)]; 
volatile uint8_t rx_msg_received = 0;

void MotorFeedbackSnapshot(FeedbackPacket_t *dst, MotorLinkInfo_t *link) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *dst = motor_feedback_data;
    uint32_t rx_cycles = fb_rx_cycles;
    uint16_t rtt = link_rtt_us;
    uint16_t seq = cmd_seq;
    __set_PRIMASK(primask);

    if (link != NULL) {
        link->cmd_seq   = seq;
        link->rtt_us    = rtt;
        link->fb_age_us = Diag_CyclesToUs16(Diag_Cycles() - rx_cycles);
    }
}

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
      FeedbackPacket_t* received_pkt = (FeedbackPacket_t*)rxRawBuffer;

      //calcuate checksum and verify
      uint8_t checksum = calculate_checksum(rxRawBuffer, sizeof(FeedbackPacket_t));
      if (received_pkt->checksum == checksum) {
        // Valid packet
        motor_feedback_data = *received_pkt;
        rx_msg_received = 1;
        diag.feedback_rx_count++;

        uint32_t now = Diag_Cycles();

        // lost feedback frames = gap in ESP counter
        if (have_esp_tx_seq) {
            uint16_t gap = (uint16_t)(received_pkt->tx_seq - last_esp_tx_seq - 1U);
            if (gap != 0 && gap < 1000U) diag.feedback_lost_count += gap;
        }
        last_esp_tx_seq = received_pkt->tx_seq;
        have_esp_tx_seq = 1;

        // time from sending command to recieving feedback
        // round trip only if ack has changed
        if (received_pkt->rx_seq != last_ack_seq) {
            last_ack_seq = received_pkt->rx_seq;
            if ((uint16_t)(cmd_seq - received_pkt->rx_seq) < 256U) { // 256 - history window
                link_rtt_us = Diag_CyclesToUs16(now - cmd_tx_cycles[received_pkt->rx_seq & 0xFF]);
            }
        }
        fb_rx_cycles = now;
      }
      else {
        diag.feedback_crc_error_count++; // wrong checksum
      }

      rxState_flag = 0; // Reset state to look for next packet
      // Restart reception for next packet
      HAL_UART_Receive_IT(huart, rxRawBuffer, 1);
    }
}