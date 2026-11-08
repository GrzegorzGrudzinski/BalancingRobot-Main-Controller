#include <string.h>
#include "pc_command.h"

#define RX_BUF_SIZE 256u   // power of two bc of buffer
_Static_assert((RX_BUF_SIZE & (RX_BUF_SIZE - 1u)) == 0u, "RX_BUF_SIZE must be a power of two");

static volatile uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0; // written only by the USB ISR
static volatile uint16_t rx_tail = 0; // written only by the main loop
static uint16_t rx_end = 0;           // main loop only: parse bytes up to here in this cycle

static uint8_t frame[PC_FRAME_SIZE];
static uint8_t frame_len = 0;

uint8_t crc8(const uint8_t *data, uint32_t len) {
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

void pcRxPush(const uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        uint16_t next = (rx_head + 1u) & (RX_BUF_SIZE - 1u); // same as % 256, but without dividing
        if (next == rx_tail) return;        // buffer full: drop the rest
        rx_buf[rx_head] = buf[i];
        rx_head = next;
    }
}

void pcCommandBeginCycle(void) {
    rx_end = rx_head; // bytes arriving later wait for the next cycle
}


uint8_t pcCommandPoll(PcCommand_t *cmd) {
    while (rx_tail != rx_end) {
        uint8_t byte = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1u) & (RX_BUF_SIZE - 1u); // same as % 256, but without dividing

        if (frame_len == 0 && byte != PC_FRAME_START) continue; // wait for a start byte
        frame[frame_len++] = byte;
        if (frame_len < PC_FRAME_SIZE) continue;

        frame_len = 0;
        if (crc8(frame, PC_FRAME_SIZE - 1) != frame[PC_FRAME_SIZE - 1]) {
            // bad frame: the next start byte may be inside it 
            for (uint8_t i = 1; i < PC_FRAME_SIZE; i++) {
                if (frame[i] == PC_FRAME_START) {
                    frame_len = PC_FRAME_SIZE - i;
                    memmove(frame, &frame[i], frame_len);
                    break;
                }
            }
            continue;
        }
        cmd->type     = frame[1];
        cmd->seq      = frame[2];
        cmd->param_id = frame[3];
        memcpy(&cmd->value, &frame[4], sizeof(float));
        return 1;   // one frame per call, the rest stays in the buffer
    }
    return 0;
}
