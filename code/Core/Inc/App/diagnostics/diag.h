#ifndef DIAG_H
#define DIAG_H

#include <stdint.h>
#include "main.h"   // CMSIS: DWT, CoreDebug, SystemCoreClock

typedef struct {
    volatile uint16_t loop_overrun_count;       // Tick arrived before previous one was handled
    volatile uint16_t uart_error_count;         // HAL UART errors on the ESP interface
    volatile uint16_t feedback_rx_count;        // Valid feedback frames from ESP
    volatile uint16_t feedback_crc_error_count; // Frames rejected due to checksum mismatch
    volatile uint16_t feedback_lost_count;      // Gaps in ESP tx_seq (frames dropped on the wire)
    volatile uint16_t cmd_drop_count;           // Commands not sent (UART busy)
    volatile uint16_t imu_error_count;          // Failed I2C reads from MPU6050
    volatile uint16_t telemetry_drop_count;     // Telemetry frames not sent (USB busy)
} Diag_t;

extern Diag_t diag;

// Free-running CPU cycle counter (DWT) - time base with sub-microsecond resolution
void Diag_Init(void);
static inline uint32_t Diag_Cycles(void) { return DWT->CYCCNT; }

// Cycle delta -> microseconds, saturated to 16 bits
static inline uint16_t Diag_CyclesToUs16(uint32_t cycles) {
    uint32_t us = cycles / (SystemCoreClock / 1000000U);
    return (us > UINT16_MAX) ? UINT16_MAX : (uint16_t)us;
}

#endif /* DIAG_H */