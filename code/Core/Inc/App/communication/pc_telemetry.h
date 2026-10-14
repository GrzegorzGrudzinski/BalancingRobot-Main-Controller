/*
    pc_telemetry.h
*/

#ifndef PC_TELEMETRY_H
#define PC_TELEMETRY_H

#include "main.h"
#include "control.h"
#include "imu_handler.h"
#include "dualFOC_communication.h"

#define TELEMETRY_START_BYTE 0xCC
#define TELEMETRY_VERSION 4

/**
 * STM32 -> PC telemetry frame (little-endian, packed, 164 B).
 * One frame = one control cycle = everything needed to reconstruct
 * the controller's decisions.
 *
 * Changing ANY field or its order here (or in ControlState_t /
 * TargetState_t / ControlDebug_t / IMU_Sample_t, which are embedded
 * as raw float blocks) requires incrementing TELEMETRY_VERSION
 * and updating the PC parser (pc/telemetry_frame.py).
 */
typedef struct __attribute__((packed)) {
    // ---- header (9 B) ----
    uint8_t  startByte;      // 0xCC
    uint8_t  version;       // TELEMETRY_VERSION
    uint32_t sequence;       // Incremented by 1 each control cycle (gaps = dropped frames)
    uint32_t timestamp_ms;   // HAL_GetTick()

    // ---- diagnostics (20 B) ----
    uint8_t  robot_state;    // RobotState_t (STM32)
    uint8_t  motor_status;   // FeedbackPacket_t.status (CMD_XX)
    uint8_t  motor_error;    // FeedbackPacket_t.error (error_states_t, ESP)
    uint16_t loop_overrun_count;
    uint16_t uart_error_count;
    uint16_t feedback_rx_count;
    uint16_t feedback_crc_error_count;
    uint16_t feedback_lost_count;  // Gaps in esp_tx_seq
    uint16_t cmd_drop_count;       // Commands not sent (UART busy)
    uint16_t imu_error_count;
    uint16_t telemetry_drop_count;

    // ---- STM32 <-> ESP link (10 B) ----
    uint16_t cmd_seq;       // Sequence number of the last command sent to the ESP
    uint16_t ack_seq;       // Sequence number of the last command processed by the ESP (FeedbackPacket_t.rx_seq)
    uint16_t esp_tx_seq;    // ESP's own frame counter (FeedbackPacket_t.tx_seq)
    uint16_t fb_age_us;     // Age of the feedback used in this control cycle
    uint16_t rtt_us;        // Last measured command-to-feedback round-trip time

    // ---- raw IMU data (24 B): ax ay az [g], gx gy gz [deg/s] ----
    IMU_Sample_t imu;

    // ---- observed state (32 B) ----
    // pitch, pitch_rate, yaw, yaw_rate, position, velocity, left_wheel_pos, right_wheel_pos
    ControlState_t observed;

    // ---- wheels (16 B) ----
    float mot1_vel;
    float mot2_vel;
    float mot1_A;
    float mot2_A;

    // ---- controller (16 + 28 B) ----
    TargetState_t  target;  // target_pitch, target_yaw, target_position, target_velocity
    ControlDebug_t debug;   // position_output, velocity_output, pitch_setpoint, pitch_output, i_position, i_velocity, i_pitch

    // ---- command sent to the motor controller (8 B) ----
    float out_l;
    float out_r;

    uint8_t checksum;       // XOR of all preceding bytes
} TelemetryPacket_t;

_Static_assert(sizeof(TelemetryPacket_t) == 164, "TelemetryPacket_t size changed - update pc/telemetry_frame.py");

// Populates the frame with the current control cycle's state
void create_telemetry_packet(
    TelemetryPacket_t *packet,
    const Controller_t *ctrl,
    const FeedbackPacket_t *fb,
    const MotorLinkInfo_t *link,
    uint8_t robot_state,
    float out_l, float out_r
);

// Sends the frame over USB CDC (NULL = no new frame in this iteration)
void RobotLog(const TelemetryPacket_t* packet);

#endif // PC_TELEMETRY_H