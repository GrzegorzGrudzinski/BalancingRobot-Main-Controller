#ifndef PC_TELEMETRY_H
#define PC_TELEMETRY_H

#include "main.h"


// Feedback frame (1 + 2*1 + 8*4 + 1 = 36 bytes)
typedef struct __attribute__((packed)) {
    uint8_t startByte;      // 0xCC
    uint8_t robot_status;   //
    uint8_t motor_status;   //
    float   angle_pitch;     // 
    float   angle_yaw;      // 
    float   mot1_vel;       // 
    float   mot1_pos;       // 
    float   mot1_A;         // 
    float   mot2_vel;       // 
    float   mot2_pos;       // 
    float   mot2_A;         // 
    uint8_t checksum;       // XOR
} TelemetryPacket_t;

TelemetryPacket_t create_telemetry_packet (
    uint8_t robot_status, uint8_t motor_status,
    float angle_pitch, float angle_yaw,
    float mot1_vel, float mot1_pos, float mot1_A,
    float mot2_vel, float mot2_pos, float mot2_A
); 
// void send_telemetry(TelemetryPacket_t* packet);
void RobotLog(TelemetryPacket_t* packet);

#endif // PC_TELEMETRY_H