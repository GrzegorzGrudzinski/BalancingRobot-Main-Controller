/**
 * pc_telemetry.c
 */

#include "pc_telemetry.h"
#include "dualFOC_communication.h" // For checksum calculation
#include "config.h"
#include "diag.h"

#include <stdint.h>
#include <string.h>
#include "usbd_cdc_if.h"

void create_telemetry_packet(
    TelemetryPacket_t *p,
    const Controller_t *ctrl,
    const FeedbackPacket_t *fb,
    const MotorLinkInfo_t *link,
    uint8_t robot_state,
    float out_l, float out_r)
{
    static uint32_t sequence = 0;

    p->startByte    = TELEMETRY_START_BYTE;
    p->version = TELEMETRY_VERSION;
    p->sequence     = sequence++;
    p->timestamp_ms = HAL_GetTick();

    // Diagnostics
    p->robot_state  = robot_state;
    p->motor_status = fb->status;
    p->motor_error  = fb->error;
    p->loop_overrun_count       = diag.loop_overrun_count;
    p->uart_error_count         = diag.uart_error_count;
    p->feedback_rx_count        = diag.feedback_rx_count;
    p->feedback_crc_error_count = diag.feedback_crc_error_count;
    p->feedback_lost_count      = diag.feedback_lost_count;
    p->cmd_drop_count           = diag.cmd_drop_count;
    p->imu_error_count          = diag.imu_error_count;
    p->telemetry_drop_count     = diag.telemetry_drop_count;

    // STM32 <-> ESP link
    p->cmd_seq     = link->cmd_seq;
    p->ack_seq     = fb->rx_seq;
    p->esp_tx_seq  = fb->tx_seq;
    p->fb_age_us   = link->fb_age_us;
    p->rtt_us      = link->rtt_us;

    // Sensors / observed state
    IMU_Sample_t imu;
    IMU_GetSample(&imu);
    memcpy(&p->imu,      &imu,                  sizeof(IMU_Sample_t));
    memcpy(&p->observed, &ctrl->observed_state, sizeof(ControlState_t));

    p->mot1_vel = fb->motor1_vel;
    p->mot2_vel = fb->motor2_vel;
    p->mot1_A   = fb->motor1_A;
    p->mot2_A   = fb->motor2_A;

    // Controller
    memcpy(&p->target, &ctrl->target_state, sizeof(TargetState_t));
    memcpy(&p->debug,  &ctrl->debug,        sizeof(ControlDebug_t));

    p->out_l = out_l;
    p->out_r = out_r;

    p->checksum = calculate_checksum(p, sizeof(TelemetryPacket_t));
}

// Two buffers: CDC_Transmit_FS retains a pointer until the transfer completes,
// so an in-flight frame must not be overwritten. If USB is busy, the new frame
// is dropped (counted in diag and visible as a gap in `sequence`).
static uint8_t tx_buf[2][sizeof(TelemetryPacket_t)];
static uint8_t tx_idx = 0;

void RobotLog(const TelemetryPacket_t* packet) {
    if (packet == NULL) return;
    if ((packet->sequence % TELEMETRY_DECIMATION) != 0) return;

    memcpy(tx_buf[tx_idx], packet, sizeof(TelemetryPacket_t));
    if (CDC_Transmit_FS(tx_buf[tx_idx], sizeof(TelemetryPacket_t)) == USBD_OK) {
        tx_idx ^= 1;
    }
    else {
        diag.telemetry_drop_count++;
    }
}