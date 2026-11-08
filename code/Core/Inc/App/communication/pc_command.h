#ifndef PC_COMMAND_H_
#define PC_COMMAND_H_

#include <stdint.h>

#define PC_FRAME_START 0xDD
#define PC_FRAME_SIZE  9   // start, type, seq, param_id, value (float), crc8

typedef enum {
    PC_CMD_HEARTBEAT = 0x01,
    PC_CMD_STOP      = 0x02,
    PC_CMD_SET_PARAM = 0x10,
    PC_CMD_SET_TARGET= 0x20,
} PcCmdType_t;

typedef enum {   // SET_TARGET ids
    PC_PARAM_TARGET_PITCH   = 0x40,
    PC_PARAM_TARGET_YAW     = 0x41,
    PC_PARAM_TARGET_VEL     = 0x42,
    PC_PARAM_TARGET_POS     = 0x43,
} PcTargetId_t;

typedef struct {
    uint8_t type;      // PcCmdType_t
    uint8_t seq;
    uint8_t param_id;  // SET_PARAM: CtrlParamId_t, SET_TARGET: PcTargetId_t
    float   value;
} PcCommand_t;

void    pcRxPush(const uint8_t *buf, uint32_t len); // USB ISR: copy bytes, nothing else
void    pcCommandBeginCycle(void);                  // main loop, once per cycle: snapshot of received bytes
uint8_t pcCommandPoll(PcCommand_t *cmd);            // main loop: 1 = one valid frame in cmd
uint8_t crc8(const uint8_t *data, uint32_t len);    // poly 0x07, init 0x00

#endif /* PC_COMMAND_H_ */
