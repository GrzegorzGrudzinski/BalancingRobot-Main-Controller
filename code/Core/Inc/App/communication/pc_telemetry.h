#ifndef PC_TELEMETRY_H
#define PC_TELEMETRY_H

#include "main.h"


// Feedback frame (1 + 1 + 4 + 4 + 4 + 4 + 1 = 19 bytes)
typedef struct __attribute__((packed)) {
    uint8_t startByte;    // 0xBB
    uint8_t status;       //
    float   value1;       // speed
    float   value2;       // speed
    float   value3;       // position
    float   value4;       // position
    uint8_t checksum;     // XOR
} TelemetryPacket_t;

TelemetryPacket_t create_telemetry_packet(uint8_t status,
    float val1,float val2,float val3,float val4); 
// void send_telemetry(TelemetryPacket_t* packet);
void RobotLog(TelemetryPacket_t* packet);

#endif // PC_TELEMETRY_H